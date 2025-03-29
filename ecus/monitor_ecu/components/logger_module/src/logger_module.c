/*
 * logger_module.c
 * Implementação do Módulo de Registro de Logs para a ECU de Monitoramento e Diagnóstico.
 * Projeto acadêmico para Mestrado Profissional em Engenharia Elétrica - UnB.
 * Adaptado para conformidade com MISRA C:2012 e padrões automotivos (ISO 11898).
 *
 * Este módulo utiliza o RTC DS3231 (via ds3231.h) para obter timestamps sincronizados,
 * integra com o módulo SD Storage para salvamento com rotação de arquivos, implementa
 * mecanismos de flush periódico, gravação assíncrona e monitoramento, e oferece funções para exportação
 * dos logs em formatos padronizados (CSV, JSON) utilizando as funções já existentes no módulo sd_storage_module.
 * As configurações (incluindo as chaves RTC_SDA, RTC_SCL, RTC_I2C_PORT e MAX_LOG_FILE_SIZE) são lidas do arquivo
 * "config.ini" utilizado pelo módulo sd_storage_module.
 */

#include "logger_module.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "sd_storage_module.h"
#include "ds3231.h"
#include "i2cdev.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <inttypes.h>
#include <stdlib.h>

#define TAG "LOGGER_MODULE"

/* Buffer interno de logs */
static LoggerEntry_t log_buffer[LOGGER_MAX_ENTRIES];
/* Índice atual para inserção no buffer */
static uint32_t log_index = 0U;
/* Mutex para proteção do buffer de log */
static SemaphoreHandle_t log_mutex = NULL;
/* Nível mínimo de log; mensagens com nível inferior serão descartadas */
static logger_level_t current_log_level = LOGGER_LEVEL_INFO;
/* Diretório padrão para gravação dos logs no SD Card */
static char sd_log_directory[MAX_FILENAME_LENGTH] = "logs";
/* Callback para notificações de alerta */
static logger_alert_callback_t alert_callback = NULL;

/* Handles para tarefas */
static TaskHandle_t persistent_flush_task_handle = NULL;
static TaskHandle_t async_write_task_handle = NULL;
static TaskHandle_t monitor_task_handle = NULL;

/* Parâmetros de configuração do RTC lidos do arquivo "config.ini" (utilizado pelo sd_storage_module) */
static i2c_dev_t rtc_dev;         /* Estrutura para comunicação I2C com o DS3231 */
static i2c_port_t rtc_i2c_port = I2C_NUM_0;
static gpio_num_t rtc_sda_gpio = 21;
static gpio_num_t rtc_scl_gpio = 22;
/* Tamanho máximo de arquivo de log, lido do config.ini */
static uint32_t max_log_file_size = DEFAULT_MAX_LOG_FILE_SIZE;

/* Protótipos de funções internas */
static void persistent_flush_task(void *arg);
static void async_write_task(void *arg);
static void monitor_task(void *arg);

/**
 * @brief Tarefa de flush periódico dos logs críticos para NVS.
 *
 * Essa tarefa é executada a cada LOGGER_PERSISTENCE_PERIOD_MS e salva os logs críticos
 * (WARNING e CRITICAL) na NVS para garantir a preservação dos dados essenciais.
 *
 * @param arg Parâmetro não utilizado.
 */
static void persistent_flush_task(void *arg)
{
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(LOGGER_PERSISTENCE_PERIOD_MS));
        if (!logger_module_save_critical_logs_to_nvs())
        {
            ESP_LOGE(TAG, "Falha ao salvar logs críticos na NVS durante o flush periódico.");
        }
        else
        {
            ESP_LOGI(TAG, "Flush periódico de logs críticos realizado com sucesso.");
        }
    }
}

/**
 * @brief Tarefa de gravação assíncrona dos logs no SD Card.
 *
 * Processa as requisições enfileiradas para gravação assíncrona e salva os dados utilizando
 * o mecanismo de rotação do módulo de armazenamento em SD Card.
 *
 * @param arg Parâmetro não utilizado.
 */
static void async_write_task(void *arg)
{
    char *data = NULL;
    for (;;)
    {
        if (xQueueReceive(sd_storage_module_get_async_write_queue(), &data, portMAX_DELAY) == pdPASS)
        {
            if (data != NULL)
            {
                if (!sd_storage_module_write_with_rotation(sd_log_directory, "logs", data))
                {
                    ESP_LOGE(TAG, "Falha ao salvar log assíncrono: %s", data);
                }
                else
                {
                    ESP_LOGI(TAG, "Log assíncrono salvo com sucesso.");
                }
                free(data);
            }
        }
    }
}

/**
 * @brief Tarefa de monitoramento e autocorreção do módulo de log.
 *
 * Periodicamente verifica parâmetros críticos, como espaço livre no SD Card e falhas persistentes na gravação,
 * acionando procedimentos corretivos se necessário.
 *
 * @param arg Parâmetro não utilizado.
 */
static void monitor_task(void *arg)
{
    struct statvfs vfs;
    uint32_t free_space = 0;
    uint32_t error_count = 0U;
    const uint32_t ERROR_THRESHOLD = 5U;
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(LOGGER_MONITOR_PERIOD_MS));
        if (statvfs(MOUNT_POINT, &vfs) == 0)
        {
            free_space = (uint32_t)(vfs.f_bsize * vfs.f_bfree);
            ESP_LOGI(TAG, "Monitor Logger: Espaço livre no SD Card: %u bytes", free_space);
            if (free_space < FREE_SPACE_THRESHOLD_DEFAULT)
            {
                logger_module_log_alert(LOGGER_LEVEL_CRITICAL, "Espaço livre crítico no SD Card detectado pelo Logger!");
                if (!logger_module_save_logs_to_sd())
                {
                    error_count++;
                    ESP_LOGE(TAG, "Erro persistente na gravação de logs (contagem: %u).", error_count);
                }
                else
                {
                    error_count = 0;
                }
                if (error_count >= ERROR_THRESHOLD)
                {
                    ESP_LOGE(TAG, "Erro persistente na gravação de logs. Procedimento de autocorreção acionado.");
                    error_count = 0;
                }
            }
        }
        else
        {
            logger_module_log_alert(LOGGER_LEVEL_WARNING, "Falha ao obter informações do sistema de arquivos no Logger!");
        }
    }
}

/**
 * @brief Inicia a tarefa de flush periódico dos logs críticos para NVS.
 *
 * Cria a tarefa que periodicamente salva os logs críticos na NVS para preservação.
 */
void logger_module_start_persistent_flush_task(void)
{
    if (persistent_flush_task_handle == NULL)
    {
        if (xTaskCreate(persistent_flush_task, "Logger_Flush_Task", 4096, NULL, 5, &persistent_flush_task_handle) != pdPASS)
        {
            ESP_LOGE(TAG, "Falha ao criar a tarefa de flush persistente.");
        }
        else
        {
            ESP_LOGI(TAG, "Tarefa de flush persistente iniciada com sucesso.");
        }
    }
}

/**
 * @brief Inicia a tarefa de gravação assíncrona dos logs no SD Card.
 *
 * Cria a tarefa que processa a fila de requisições de gravação assíncrona.
 */
void logger_module_start_async_write_task(void)
{
    if (xTaskCreate(async_write_task, "Logger_Async_Write_Task", 4096, NULL, 5, &async_write_task_handle) != pdPASS)
    {
        ESP_LOGE(TAG, "Falha ao criar a tarefa de gravação assíncrona.");
    }
    else
    {
        ESP_LOGI(TAG, "Tarefa de gravação assíncrona iniciada com sucesso.");
    }
}

/**
 * @brief Envia uma requisição de gravação assíncrona dos logs para o SD Card.
 *
 * A requisição é enfileirada e processada pela tarefa de gravação assíncrona.
 *
 * @param data Dados a serem gravados.
 * @return true se a requisição for enfileirada com sucesso, false caso contrário.
 */
bool logger_module_async_write(const char *data)
{
    char *data_copy = NULL;
    if (data == NULL)
    {
        ESP_LOGE(TAG, "Dados nulos para gravação assíncrona.");
        return false;
    }
    data_copy = strdup(data);
    if (data_copy == NULL)
    {
        ESP_LOGE(TAG, "Falha ao alocar memória para dados assíncronos.");
        return false;
    }
    if (xQueueSend(sd_storage_module_get_async_write_queue(), &data_copy, portMAX_DELAY) != pdPASS)
    {
        ESP_LOGE(TAG, "Falha ao enfileirar requisição de gravação assíncrona.");
        free(data_copy);
        return false;
    }
    return true;
}

/**
 * @brief Inicializa o módulo de log.
 *
 * Inicializa o buffer de log, os recursos de sincronização, carrega a configuração do arquivo
 * "config.ini", configura o RTC DS3231 e inicia as tarefas de flush persistente, gravação assíncrona
 * e monitoramento do módulo.
 *
 * @return true se a inicialização for bem-sucedida, false caso contrário.
 */
bool logger_module_init(void)
{
    log_index = 0U;
    log_mutex = xSemaphoreCreateMutex();
    if (log_mutex == NULL)
    {
        ESP_LOGE(TAG, "Falha ao criar mutex do logger.");
        return false;
    }
    if (!logger_module_load_config())
    {
        ESP_LOGW(TAG, "Configuração do logger não carregada; usando parâmetros padrão.");
    }
    if (!logger_module_configure_rtc())
    {
        ESP_LOGE(TAG, "Falha ao configurar o RTC DS3231.");
        return false;
    }
    logger_module_start_persistent_flush_task();
    logger_module_start_async_write_task();
    logger_module_start_monitor_task();
    ESP_LOGI(TAG, "Módulo de log inicializado com sucesso.");
    return true;
}

/**
 * @brief Registra uma mensagem de log.
 *
 * Se o nível da mensagem for inferior ao nível mínimo configurado, a mensagem é descartada.
 * Utiliza o RTC DS3231 para obter um timestamp sincronizado.
 *
 * @param level Nível da mensagem (LOGGER_LEVEL_INFO, LOGGER_LEVEL_WARNING ou LOGGER_LEVEL_CRITICAL).
 * @param format String de formato (estilo printf).
 * @param ... Argumentos para formatação.
 */
void logger_module_log(logger_level_t level, const char *format, ... )
{
    if ((format == NULL) || (level < current_log_level))
    {
        return;
    }
    if (xSemaphoreTake(log_mutex, portMAX_DELAY) == pdTRUE)
    {
        va_list args;
        va_start(args, format);
        log_buffer[log_index].timestamp = logger_module_get_rtc_timestamp();
        log_buffer[log_index].level = level;
        (void)vsnprintf(log_buffer[log_index].message, LOGGER_MSG_MAX_SIZE, format, args);
        va_end(args);
        if ((level == LOGGER_LEVEL_WARNING || level == LOGGER_LEVEL_CRITICAL) && (alert_callback != NULL))
        {
            alert_callback(&log_buffer[log_index]);
        }
        log_index = (log_index + 1U) % LOGGER_MAX_ENTRIES;
        xSemaphoreGive(log_mutex);
    }
}

/**
 * @brief Registra uma mensagem de alerta.
 *
 * Prepara a mensagem com o prefixo "ALERTA:" e a registra com o nível informado.
 *
 * @param level Nível do alerta (LOGGER_LEVEL_WARNING ou LOGGER_LEVEL_CRITICAL).
 * @param message Mensagem descritiva do alerta.
 */
void logger_module_log_alert(logger_level_t level, const char *message)
{
    if (message == NULL)
    {
        return;
    }
    char alert_message[LOGGER_MSG_MAX_SIZE];
    (void)snprintf(alert_message, LOGGER_MSG_MAX_SIZE, "ALERTA: %s", message);
    logger_module_log(level, "%s", alert_message);
}

/**
 * @brief Registra um callback para notificações de alerta.
 *
 * Permite que módulos externos sejam notificados imediatamente quando um alerta for registrado.
 *
 * @param callback Função callback que receberá a entrada de log de alerta.
 */
void logger_module_register_alert_callback(logger_alert_callback_t callback)
{
    alert_callback = callback;
    ESP_LOGI(TAG, "Callback de alerta registrado com sucesso.");
}

/**
 * @brief Imprime todas as entradas de log armazenadas no buffer.
 *
 * Itera sobre o buffer de log e imprime cada entrada utilizando ESP_LOGI.
 */
void logger_module_print_logs(void)
{
    if (xSemaphoreTake(log_mutex, portMAX_DELAY) == pdTRUE)
    {
        for (uint32_t i = 0U; i < LOGGER_MAX_ENTRIES; i++)
        {
            if (log_buffer[i].timestamp != 0U)
            {
                ESP_LOGI(TAG, "[%" PRIu32 " ms] Nível: %d - %s",
                         log_buffer[i].timestamp,
                         (int)log_buffer[i].level,
                         log_buffer[i].message);
            }
        }
        xSemaphoreGive(log_mutex);
    }
}

/**
 * @brief Envia os logs armazenados para um sistema externo.
 *
 * Essa função é um placeholder para futura integração com protocolos como MQTT.
 */
void logger_module_send_logs(void)
{
    ESP_LOGI(TAG, "Envio de logs via MQTT não implementado.");
}

/**
 * @brief Define o nível mínimo de log a ser registrado.
 *
 * Mensagens com nível inferior ao definido serão descartadas.
 *
 * @param level Nível mínimo de log.
 */
void logger_module_set_log_level(logger_level_t level)
{
    current_log_level = level;
    ESP_LOGI(TAG, "Nível mínimo de log configurado para %d.", (int)level);
}

/**
 * @brief Configura o diretório padrão para gravação dos logs no SD Card.
 *
 * Este diretório será utilizado pelas funções de salvamento para persistência dos logs.
 *
 * @param dirname Nome do diretório.
 */
void logger_module_set_sd_directory(const char *dirname)
{
    if ((dirname != NULL) && (strlen(dirname) < MAX_FILENAME_LENGTH))
    {
        (void)snprintf(sd_log_directory, MAX_FILENAME_LENGTH, "%s", dirname);
        ESP_LOGI(TAG, "Diretório de logs configurado para: %s", sd_log_directory);
    }
    else
    {
        ESP_LOGE(TAG, "Nome do diretório de logs inválido.");
    }
}

/**
 * @brief Salva todas as entradas de log do buffer no SD Card.
 *
 * Itera sobre o buffer interno e utiliza o mecanismo de gravação com rotação do módulo
 * de armazenamento em SD Card para persistir os logs.
 *
 * @return true se os logs forem salvos com sucesso, false caso contrário.
 */
bool logger_module_save_logs_to_sd(void)
{
    bool status = true;
    char log_entry[LOGGER_MSG_MAX_SIZE + 32];
    if (xSemaphoreTake(log_mutex, portMAX_DELAY) == pdTRUE)
    {
        for (uint32_t i = 0U; i < LOGGER_MAX_ENTRIES; i++)
        {
            if (log_buffer[i].timestamp != 0U)
            {
                (void)snprintf(log_entry, sizeof(log_entry), "%u,%d,%s",
                               log_buffer[i].timestamp,
                               (int)log_buffer[i].level,
                               log_buffer[i].message);
                if (!sd_storage_module_write_with_rotation(sd_log_directory, "logs", log_entry))
                {
                    ESP_LOGE(TAG, "Falha ao salvar log no SD: %s", log_entry);
                    status = false;
                }
            }
        }
        xSemaphoreGive(log_mutex);
    }
    else
    {
        ESP_LOGE(TAG, "Falha ao adquirir mutex do logger para salvar logs no SD.");
        status = false;
    }
    return status;
}

/**
 * @brief Carrega as configurações do módulo de log a partir do arquivo "config.ini".
 *
 * Lê os parâmetros de configuração, incluindo as chaves RTC_SDA, RTC_SCL, RTC_I2C_PORT e
 * o tamanho máximo dos arquivos de log (MAX_LOG_FILE_SIZE) a partir do arquivo "config.ini"
 * localizado no diretório de montagem do SD Card.
 *
 * @return true se a configuração for carregada com sucesso, false caso contrário.
 */
bool logger_module_load_config(void)
{
    FILE *file;
    char config_line[128];
    char config_path[MAX_FILENAME_LENGTH];

    (void)snprintf(config_path, MAX_FILENAME_LENGTH, "%s/%s", MOUNT_POINT, "config.ini");
    file = fopen(config_path, "r");
    if (file == NULL)
    {
        ESP_LOGW(TAG, "Arquivo de configuração %s não encontrado.", config_path);
        return false;
    }
    while (fgets(config_line, sizeof(config_line), file) != NULL)
    {
        if (strncmp(config_line, "RTC_SDA=", 8) == 0)
        {
            rtc_sda_gpio = (gpio_num_t)atoi(&config_line[8]);
        }
        else if (strncmp(config_line, "RTC_SCL=", 8) == 0)
        {
            rtc_scl_gpio = (gpio_num_t)atoi(&config_line[8]);
        }
        else if (strncmp(config_line, "RTC_I2C_PORT=", 13) == 0)
        {
            rtc_i2c_port = (i2c_port_t)atoi(&config_line[13]);
        }
        else if (strncmp(config_line, "MAX_LOG_FILE_SIZE=", 18) == 0)
        {
            max_log_file_size = (uint32_t)atoi(&config_line[18]);
        }
    }
    fclose(file);
    ESP_LOGI(TAG, "Configuração do logger carregada a partir de %s.", config_path);
    return true;
}

/**
 * @brief Configura o RTC DS3231 utilizando a biblioteca ds3231.h.
 *
 * Inicializa a comunicação I2C com o RTC DS3231 utilizando os parâmetros obtidos do arquivo
 * de configuração.
 *
 * @return true se o RTC for configurado com sucesso, false caso contrário.
 */
bool logger_module_configure_rtc(void)
{
    esp_err_t ret = ds3231_init_desc(&rtc_dev, rtc_i2c_port, rtc_sda_gpio, rtc_scl_gpio);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao inicializar descritor do DS3231: %s", esp_err_to_name(ret));
        return false;
    }
    struct tm time_now;
    ret = ds3231_get_time(&rtc_dev, &time_now);
    ds3231_free_desc(&rtc_dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao obter tempo do DS3231: %s", esp_err_to_name(ret));
        return false;
    }
    ESP_LOGI(TAG, "RTC DS3231 configurado com sucesso. Tempo atual: %04d-%02d-%02d %02d:%02d:%02d",
             time_now.tm_year + 1900, time_now.tm_mon + 1, time_now.tm_mday,
             time_now.tm_hour, time_now.tm_min, time_now.tm_sec);
    return true;
}

/**
 * @brief Obtém um timestamp sincronizado com o RTC DS3231.
 *
 * Utiliza a biblioteca ds3231.h para obter o horário atual e converte para timestamp
 * (milissegundos desde a época Unix). Em caso de falha, retorna 0.
 *
 * @return uint32_t Timestamp em milissegundos.
 */
uint32_t logger_module_get_rtc_timestamp(void)
{
    uint32_t timestamp = 0U;
    i2c_dev_t dev;
    struct tm time_now;
    esp_err_t ret = ds3231_init_desc(&dev, rtc_i2c_port, rtc_sda_gpio, rtc_scl_gpio);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao inicializar descritor do DS3231 para timestamp: %s", esp_err_to_name(ret));
        return 0U;
    }
    ret = ds3231_get_time(&dev, &time_now);
    ds3231_free_desc(&dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao obter tempo do DS3231 para timestamp: %s", esp_err_to_name(ret));
        return 0U;
    }
    timestamp = (uint32_t)mktime(&time_now) * 1000U;
    return timestamp;
}

/**
 * @brief Salva os logs críticos (WARNING e CRITICAL) na NVS para preservação.
 *
 * Esta função salva os logs críticos do buffer na NVS, garantindo que dados essenciais
 * não sejam perdidos em caso de falha do sistema.
 *
 * @return true se os logs críticos forem salvos com sucesso, false caso contrário.
 */
bool logger_module_save_critical_logs_to_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t err;
    if (xSemaphoreTake(log_mutex, portMAX_DELAY) == pdTRUE)
    {
        err = nvs_open("logger_storage", NVS_READWRITE, &handle);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Falha ao abrir NVS para logs críticos: %s", esp_err_to_name(err));
            xSemaphoreGive(log_mutex);
            return false;
        }
        err = nvs_set_blob(handle, "critical_logs", log_buffer, sizeof(log_buffer));
        if (err == ESP_OK)
        {
            nvs_commit(handle);
        }
        nvs_close(handle);
        xSemaphoreGive(log_mutex);
        return (err == ESP_OK);
    }
    ESP_LOGE(TAG, "Falha ao adquirir mutex para salvar logs críticos.");
    return false;
}

/**
 * @brief Carrega os logs críticos salvos da NVS.
 *
 * @return true se os logs críticos forem carregados com sucesso, false caso contrário.
 */
bool logger_module_load_critical_logs_from_nvs(void)
{
    nvs_handle_t handle;
    size_t required_size = sizeof(log_buffer);
    esp_err_t err = nvs_open("logger_storage", NVS_READONLY, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao abrir NVS para carregar logs críticos: %s", esp_err_to_name(err));
        return false;
    }
    err = nvs_get_blob(handle, "critical_logs", log_buffer, &required_size);
    nvs_close(handle);
    return (err == ESP_OK);
}

/**
 * @brief Exporta os logs armazenados no buffer para um arquivo CSV.
 *
 * Utiliza as funções do módulo SD Storage para salvar os logs em formato CSV,
 * facilitando a integração com ferramentas externas.
 *
 * @return true se os logs forem exportados com sucesso, false caso contrário.
 */
bool logger_module_export_logs_csv(void)
{
    char csv_entry[LOGGER_MSG_MAX_SIZE + 64];
    bool status = true;
    if (xSemaphoreTake(log_mutex, portMAX_DELAY) == pdTRUE)
    {
        for (uint32_t i = 0U; i < LOGGER_MAX_ENTRIES; i++)
        {
            if (log_buffer[i].timestamp != 0U)
            {
                (void)snprintf(csv_entry, sizeof(csv_entry), "%u,%d,%s",
                               log_buffer[i].timestamp,
                               (int)log_buffer[i].level,
                               log_buffer[i].message);
                if (!sd_storage_module_write_with_rotation(sd_log_directory, "logs", csv_entry))
                {
                    ESP_LOGE(TAG, "Falha ao exportar log em CSV: %s", csv_entry);
                    status = false;
                }
            }
        }
        xSemaphoreGive(log_mutex);
    }
    else
    {
        ESP_LOGE(TAG, "Falha ao adquirir mutex do logger para exportar logs em CSV.");
        status = false;
    }
    return status;
}

/**
 * @brief Exporta os logs armazenados no buffer para um arquivo JSON.
 *
 * Utiliza as funções do módulo SD Storage para salvar os logs em formato JSON,
 * facilitando a integração com ferramentas externas.
 *
 * @return true se os logs forem exportados com sucesso, false caso contrário.
 */
bool logger_module_export_logs_json(void)
{
    bool status = true;
    char json_entry[LOGGER_MSG_MAX_SIZE + 128];
    if (xSemaphoreTake(log_mutex, portMAX_DELAY) == pdTRUE)
    {
        for (uint32_t i = 0U; i < LOGGER_MAX_ENTRIES; i++)
        {
            if (log_buffer[i].timestamp != 0U)
            {
                (void)snprintf(json_entry, sizeof(json_entry),
                               "{\"timestamp\":%u,\"level\":%d,\"message\":\"%s\"}",
                               log_buffer[i].timestamp,
                               (int)log_buffer[i].level,
                               log_buffer[i].message);
                if (!sd_storage_module_write_with_rotation(sd_log_directory, "logs", json_entry))
                {
                    ESP_LOGE(TAG, "Falha ao exportar log em JSON: %s", json_entry);
                    status = false;
                }
            }
        }
        xSemaphoreGive(log_mutex);
    }
    else
    {
        ESP_LOGE(TAG, "Falha ao adquirir mutex do logger para exportar logs em JSON.");
        status = false;
    }
    return status;
}

/**
 * @brief Atualiza dinamicamente o tamanho máximo dos arquivos de log.
 *
 * Permite alterar o parâmetro MAX_LOG_FILE_SIZE sem reinicializar a ECU.
 *
 * @param max_size Novo tamanho máximo do arquivo de log em bytes.
 */
void logger_module_set_max_log_file_size(uint32_t max_size)
{
    max_log_file_size = max_size;
    ESP_LOGI(TAG, "Tamanho máximo de arquivo de log atualizado para %u bytes.", max_log_file_size);
}

/**
 * @brief Inicia a tarefa de monitoramento e autocorreção do módulo de log.
 *
 * Cria a tarefa que periodicamente verifica a saúde do Logger e aciona procedimentos corretivos se necessário.
 */
void logger_module_start_monitor_task(void)
{
    if (xTaskCreate(monitor_task, "Logger_Monitor_Task", 4096, NULL, 5, &monitor_task_handle) != pdPASS)
    {
        ESP_LOGE(TAG, "Falha ao criar a tarefa de monitoramento do Logger.");
    }
    else
    {
        ESP_LOGI(TAG, "Tarefa de monitoramento do Logger iniciada com sucesso.");
    }
}

#ifdef __cplusplus
}
#endif
