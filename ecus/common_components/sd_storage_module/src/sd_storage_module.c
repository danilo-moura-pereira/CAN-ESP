/*
 * sd_storage_module.c
 * Implementação do Módulo de Armazenamento em SD Card para a ECU de Monitoramento e Diagnóstico.
 * Projeto acadêmico para Mestrado Profissional em Engenharia Elétrica - UnB.
 * Adaptado para conformidade com MISRA C:2012 e padrões automotivos (ISO 11898).
 */

#include "sd_storage_module.h"
#include "esp_log.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"
#include "esp32/rom/rtc_wdt.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <dirent.h>
#include <limits.h>
#include <time.h>

#define TAG "SD_STORAGE_MODULE"

/* Parâmetros padrão (caso a configuração não seja carregada) */
static sd_config_t sd_config = {
    .mosi_pin = DEFAULT_MOSI_PIN_GPIO,
    .miso_pin = DEFAULT_MOSI_PIN_GPIO,
    .sclk_pin = DEFAULT_MOSI_PIN_GPIO,
    .cs_pin = DEFAULT_MOSI_PIN_GPIO,
    .max_log_file_size = DEFAULT_MAX_LOG_FILE_SIZE,
    .free_space_threshold = FREE_SPACE_THRESHOLD_DEFAULT
};

/* Diretório padrão para gravações */
static char default_directory[MAX_FILENAME_LENGTH] = "logs";

/* Variáveis de controle */
static SemaphoreHandle_t sd_mutex = NULL;
static bool sd_initialized = false;
static sdmmc_card_t *sd_card = NULL;

/* Fila de escrita assíncrona */
static QueueHandle_t async_write_queue = NULL;

/* Callbacks */
static sd_storage_write_callback_t write_callback = NULL;
static sd_storage_free_space_callback_t free_space_callback = NULL;

/* Protótipos de funções internas */
static void sd_storage_module_monitor_task(void *arg);
static void sd_storage_module_watchdog_task(void *arg);
static void sd_storage_module_async_write_task(void *arg);

/*==============================================================================*/
/*                    FUNÇÕES DE CONFIGURAÇÃO DINÂMICA                        */
/*==============================================================================*/

/**
 * @brief Carrega configurações do arquivo INI (CONFIG_FILE_PATH).
 *
 * O arquivo deve conter linhas no formato "chave=valor" na seção [SDCard],
 * por exemplo:
 *   mosi_pin=23
 *   miso_pin=19
 *   sclk_pin=18
 *   cs_pin=5
 *   max_log_file_size=10240
 *   free_space_threshold=51200
 *
 * @return true se as configurações foram carregadas, false caso contrário.
 */
bool sd_storage_module_load_config(void)
{
    FILE *file = fopen(CONFIG_FILE_PATH, "r");
    char line[MAX_CONFIG_LINE_LENGTH];
    if (file == NULL)
    {
        ESP_LOGW(TAG, "Arquivo de configuração '%s' não encontrado. Usando valores padrão.", CONFIG_FILE_PATH);
        return false;
    }
    while (fgets(line, sizeof(line), file) != NULL)
    {
        /* Ignora comentários e linhas em branco */
        if ((line[0] == '#') || (line[0] == ';') || (line[0] == '\n'))
        {
            continue;
        }
        char key[32];
        int value;
        if (sscanf(line, "%31[^=]=%d", key, &value) == 2)
        {
            if (strcmp(key, "mosi_pin") == 0)
            {
                sd_config.mosi_pin = value;
            }
            else if (strcmp(key, "miso_pin") == 0)
            {
                sd_config.miso_pin = value;
            }
            else if (strcmp(key, "sclk_pin") == 0)
            {
                sd_config.sclk_pin = value;
            }
            else if (strcmp(key, "cs_pin") == 0)
            {
                sd_config.cs_pin = value;
            }
            else if (strcmp(key, "max_log_file_size") == 0)
            {
                sd_config.max_log_file_size = (uint32_t)value;
            }
            else if (strcmp(key, "free_space_threshold") == 0)
            {
                sd_config.free_space_threshold = (uint32_t)value;
            }
        }
    }
    fclose(file);
    ESP_LOGI(TAG, "Configuração carregada do arquivo '%s'.", CONFIG_FILE_PATH);
    return true;
}

/**
 * @brief Aplica uma nova configuração ao módulo de armazenamento.
 *
 * @param config Ponteiro para a nova configuração.
 */
void sd_storage_module_apply_config(const sd_config_t *config)
{
    if (config == NULL)
    {
        ESP_LOGE(TAG, "Configuração inválida.");
        return;
    }
    sd_config = *config;
    ESP_LOGI(TAG, "Nova configuração aplicada: MOSI=%d, MISO=%d, SCLK=%d, CS=%d, MaxLogSize=%u, FreeSpaceThreshold=%u",
             sd_config.mosi_pin, sd_config.miso_pin, sd_config.sclk_pin, sd_config.cs_pin,
             sd_config.max_log_file_size, sd_config.free_space_threshold);
}

/*==============================================================================*/
/*                       FUNÇÕES PRINCIPAIS DO SD CARD                        */
/*==============================================================================*/

/**
 * @brief Inicializa o módulo de armazenamento em SD Card.
 *
 * Configura o barramento SPI com os pinos definidos na configuração, monta o sistema
 * de arquivos e inicializa os recursos internos.
 *
 * @return true se a inicialização for bem-sucedida, false caso contrário.
 */
bool sd_storage_module_init(void)
{
    esp_err_t ret;

    if (sd_initialized)
    {
        ESP_LOGW(TAG, "SD Card já inicializado.");
        return true;
    }

    sd_mutex = xSemaphoreCreateMutex();
    if (sd_mutex == NULL)
    {
        ESP_LOGE(TAG, "Falha ao criar mutex do SD Card.");
        return false;
    }

    /* Tenta carregar a configuração do arquivo INI */
    (void)sd_storage_module_load_config();

    ESP_LOGI(TAG, "Inicializando SD Card...");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = sd_config.mosi_pin,
        .miso_io_num = sd_config.miso_pin,
        .sclk_io_num = sd_config.sclk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao inicializar barramento SPI: %s", esp_err_to_name(ret));
        return false;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = sd_config.cs_pin;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &sd_card);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao montar SD Card: %s", esp_err_to_name(ret));
        return false;
    }

    sd_initialized = true;
    ESP_LOGI(TAG, "SD Card inicializado com sucesso. Sistema de arquivos montado em %s", MOUNT_POINT);
    return true;
}

/**
 * @brief Finaliza o uso do SD Card e desmonta o sistema de arquivos.
 */
void sd_storage_module_deinit(void)
{
    if (!sd_initialized)
    {
        return;
    }
    esp_vfs_fat_sdcard_unmount(MOUNT_POINT, sd_card);
    spi_bus_free(SDSPI_HOST_DEFAULT().slot);
    if (sd_mutex != NULL)
    {
        vSemaphoreDelete(sd_mutex);
        sd_mutex = NULL;
    }
    sd_initialized = false;
    ESP_LOGI(TAG, "SD Card desmontado e módulo finalizado.");
}

/**
 * @brief Grava dados no SD Card em um arquivo especificado.
 *
 * Abre o arquivo em modo "append" e grava a string fornecida, seguida de uma nova linha.
 *
 * @param filename Nome do arquivo.
 * @param data Dados a serem gravados.
 * @return true se a gravação for bem-sucedida, false caso contrário.
 */
bool sd_storage_module_write(const char *filename, const char *data)
{
    char path[MAX_FILENAME_LENGTH];

    if ((filename == NULL) || (data == NULL))
    {
        ESP_LOGE(TAG, "Parâmetros inválidos para gravação.");
        return false;
    }
    if (!sd_initialized)
    {
        ESP_LOGE(TAG, "SD Card não inicializado.");
        return false;
    }
    (void)snprintf(path, MAX_FILENAME_LENGTH, "%s/%s", MOUNT_POINT, filename);
    if (xSemaphoreTake(sd_mutex, portMAX_DELAY) == pdTRUE)
    {
        FILE *file = fopen(path, "a");
        if (file == NULL)
        {
            ESP_LOGE(TAG, "Falha ao abrir arquivo %s", path);
            xSemaphoreGive(sd_mutex);
            return false;
        }
        fprintf(file, "%s\n", data);
        fclose(file);
        xSemaphoreGive(sd_mutex);
        ESP_LOGI(TAG, "Dados gravados com sucesso em %s", path);
        if (write_callback != NULL)
        {
            write_callback(filename, data);
        }
        return true;
    }
    return false;
}

/**
 * @brief Lê um arquivo do SD Card e copia seu conteúdo para um buffer.
 *
 * @param filename Nome do arquivo.
 * @param buffer Buffer para armazenar os dados.
 * @param max_len Tamanho máximo do buffer.
 * @return true se a leitura for bem-sucedida, false caso contrário.
 */
bool sd_storage_module_read(const char *filename, char *buffer, size_t max_len)
{
    char path[MAX_FILENAME_LENGTH];

    if ((filename == NULL) || (buffer == NULL) || (max_len == 0U))
    {
        ESP_LOGE(TAG, "Parâmetros inválidos para leitura.");
        return false;
    }
    if (!sd_initialized)
    {
        ESP_LOGE(TAG, "SD Card não inicializado.");
        return false;
    }
    (void)snprintf(path, MAX_FILENAME_LENGTH, "%s/%s", MOUNT_POINT, filename);
    if (xSemaphoreTake(sd_mutex, portMAX_DELAY) == pdTRUE)
    {
        FILE *file = fopen(path, "r");
        if (file == NULL)
        {
            ESP_LOGE(TAG, "Falha ao abrir arquivo %s", path);
            xSemaphoreGive(sd_mutex);
            return false;
        }
        if (fgets(buffer, (int)max_len, file) == NULL)
        {
            ESP_LOGE(TAG, "Falha ao ler arquivo %s", path);
            fclose(file);
            xSemaphoreGive(sd_mutex);
            return false;
        }
        fclose(file);
        xSemaphoreGive(sd_mutex);
        ESP_LOGI(TAG, "Arquivo %s lido com sucesso.", path);
        return true;
    }
    return false;
}

/*==============================================================================*/
/*                FUNÇÕES DE MONITORAMENTO, WATCHDOG E GRAVAÇÃO ASSÍNCRONA       */
/*==============================================================================*/

/**
 * @brief Tarefa de monitoramento do SD Card.
 *
 * Verifica periodicamente a operação do SD Card, o espaço livre e dispara
 * a limpeza automática dos logs quando o espaço estiver crítico.
 */
static void sd_storage_module_monitor_task(void *arg)
{
    char test_buffer[32];
    struct statvfs vfs;
    uint32_t free_space = 0;
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(MONITOR_PERIOD_MS));
        if (!sd_storage_module_read(TEST_FILENAME, test_buffer, sizeof(test_buffer)))
        {
            ESP_LOGW(TAG, "Falha ao ler arquivo de teste. Tentando remontar SD Card...");
            sd_storage_module_deinit();
            if (!sd_storage_module_init())
            {
                ESP_LOGE(TAG, "Erro ao remontar SD Card. Nova tentativa em breve.");
            }
            else
            {
                ESP_LOGI(TAG, "SD Card remontado com sucesso.");
            }
        }
        else
        {
            ESP_LOGI(TAG, "SD Card operando normalmente.");
        }
        if (statvfs(MOUNT_POINT, &vfs) == 0)
        {
            free_space = (uint32_t)(vfs.f_bsize * vfs.f_bfree);
            ESP_LOGI(TAG, "Espaço livre: %u bytes", free_space);
            if (free_space < sd_config.free_space_threshold)
            {
                ESP_LOGW(TAG, "Espaço livre crítico: %u bytes", free_space);
                if (free_space_callback != NULL)
                {
                    free_space_callback(free_space);
                }
                (void)sd_storage_module_cleanup_logs(default_directory);
            }
        }
        else
        {
            ESP_LOGE(TAG, "Falha ao obter informações do sistema de arquivos.");
        }
    }
}

/**
 * @brief Tarefa de Watchdog que alimenta o Task Watchdog e o RTC WDT.
 */
static void sd_storage_module_watchdog_task(void *arg)
{
    esp_err_t ret;
    ret = esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao inicializar Task Watchdog: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
    }
    ret = esp_task_wdt_add(NULL);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao adicionar tarefa ao Watchdog: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "Task Watchdog inicializado (timeout: %u s).", WDT_TIMEOUT_SECONDS);
    rtc_wdt_protect_off();
    rtc_wdt_init();
    rtc_wdt_set_length(WDT_TIMEOUT_SECONDS * 1000U);
    rtc_wdt_enable();
    ESP_LOGI(TAG, "RTC WDT configurado (timeout: %u ms).", WDT_TIMEOUT_SECONDS * 1000U);
    for (;;)
    {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief Inicia a tarefa de monitoramento do SD Card.
 */
void sd_storage_module_start_monitor_task(void)
{
    if (xTaskCreate(sd_storage_module_monitor_task, "SD_Monitor_Task", 4096, NULL, 5, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "Falha ao criar tarefa de monitoramento.");
    }
    else
    {
        ESP_LOGI(TAG, "Tarefa de monitoramento iniciada.");
    }
}

/**
 * @brief Inicia a tarefa de Watchdog do SD Card.
 */
void sd_storage_module_start_watchdog_task(void)
{
    if (xTaskCreate(sd_storage_module_watchdog_task, "SD_Watchdog_Task", 2048, NULL, 10, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "Falha ao criar tarefa do Watchdog.");
    }
    else
    {
        ESP_LOGI(TAG, "Tarefa do Watchdog iniciada com sucesso.");
    }
}

/**
 * @brief Tarefa de gravação assíncrona.
 *
 * Processa as requisições de escrita enfileiradas.
 */
static void sd_storage_module_async_write_task(void *arg)
{
    sd_async_write_req_t req;
    for (;;)
    {
        if (xQueueReceive(async_write_queue, &req, portMAX_DELAY) == pdPASS)
        {
            if (!sd_storage_module_write_with_rotation(req.dirname, req.file_prefix, req.data))
            {
                ESP_LOGE(TAG, "Falha ao gravar requisição assíncrona.");
            }
            else
            {
                ESP_LOGI(TAG, "Requisição assíncrona gravada com sucesso.");
            }
        }
    }
}

/**
 * @brief Envia uma requisição de gravação assíncrona.
 */
bool sd_storage_module_async_write(const char *dirname, const char *file_prefix, const char *data)
{
    sd_async_write_req_t req;
    int ret;
    const char *target_dir = (dirname != NULL) ? dirname : default_directory;

    if ((target_dir == NULL) || (file_prefix == NULL) || (data == NULL))
    {
        ESP_LOGE(TAG, "Parâmetros inválidos para escrita assíncrona.");
        return false;
    }
    (void)snprintf(req.dirname, MAX_FILENAME_LENGTH, "%s", target_dir);
    (void)snprintf(req.file_prefix, MAX_FILENAME_LENGTH, "%s", file_prefix);
    (void)snprintf(req.data, ASYNC_WRITE_MAX_DATA_LENGTH, "%s", data);
    if (async_write_queue == NULL)
    {
        async_write_queue = xQueueCreate(10, sizeof(sd_async_write_req_t));
        if (async_write_queue == NULL)
        {
            ESP_LOGE(TAG, "Falha ao criar fila assíncrona.");
            return false;
        }
    }
    ret = xQueueSend(async_write_queue, &req, portMAX_DELAY);
    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Falha ao enfileirar requisição assíncrona.");
        return false;
    }
    return true;
}

/**
 * @brief Inicia a tarefa de gravação assíncrona.
 */
void sd_storage_module_start_async_write_task(void)
{
    if (async_write_queue == NULL)
    {
        async_write_queue = xQueueCreate(10, sizeof(sd_async_write_req_t));
        if (async_write_queue == NULL)
        {
            ESP_LOGE(TAG, "Falha ao criar fila assíncrona.");
            return;
        }
    }
    if (xTaskCreate(sd_storage_module_async_write_task, "SD_Async_Write_Task", 4096, NULL, 5, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "Falha ao criar tarefa de escrita assíncrona.");
    }
    else
    {
        ESP_LOGI(TAG, "Tarefa de escrita assíncrona iniciada.");
    }
}

/*==============================================================================*/
/*         FUNÇÕES DE CONFIGURAÇÃO DINÂMICA DE DIRETÓRIOS E CALLBACK             */
/*==============================================================================*/

/**
 * @brief Registra callback para notificação de gravação.
 */
void sd_storage_module_register_write_callback(sd_storage_write_callback_t callback)
{
    write_callback = callback;
    ESP_LOGI(TAG, "Callback de escrita registrado.");
}

/**
 * @brief Define dinamicamente o diretório padrão para gravação.
 */
void sd_storage_module_set_default_directory(const char *dirname)
{
    if ((dirname != NULL) && (strlen(dirname) < MAX_FILENAME_LENGTH))
    {
        (void)snprintf(default_directory, MAX_FILENAME_LENGTH, "%s", dirname);
        ESP_LOGI(TAG, "Diretório padrão definido: %s", default_directory);
    }
    else
    {
        ESP_LOGE(TAG, "Nome de diretório padrão inválido.");
    }
}

/*==============================================================================*/
/*   FUNÇÕES DE GERENCIAMENTO DE DIRETÓRIOS, ROTAÇÃO, LIMPEZA E LOG CÍRCULADO       */
/*==============================================================================*/

/**
 * @brief Cria um diretório no SD Card, se não existir.
 */
bool sd_storage_module_create_directory(const char *dirname)
{
    struct stat st;
    char path[MAX_FILENAME_LENGTH];

    if (dirname == NULL)
    {
        ESP_LOGE(TAG, "Nome do diretório nulo.");
        return false;
    }
    (void)snprintf(path, MAX_FILENAME_LENGTH, "%s/%s", MOUNT_POINT, dirname);
    if (stat(path, &st) == 0)
    {
        return true;
    }
    else
    {
        if (mkdir(path, 0777) != 0)
        {
            ESP_LOGE(TAG, "Falha ao criar diretório %s.", path);
            return false;
        }
        ESP_LOGI(TAG, "Diretório %s criado com sucesso.", path);
        return true;
    }
}

/**
 * @brief Define o tamanho máximo configurável para um arquivo de log.
 */
void sd_storage_module_set_max_file_size(uint32_t max_size)
{
    sd_config.max_log_file_size = max_size;
    ESP_LOGI(TAG, "Tamanho máximo de arquivo configurado para %u bytes.", max_size);
}

/**
 * @brief Grava dados em um arquivo com rotação automática.
 *
 * Se o arquivo atual ultrapassar o tamanho máximo, um novo arquivo é criado com sufixo
 * baseado no timestamp.
 */
bool sd_storage_module_write_with_rotation(const char *dirname, const char *file_prefix, const char *data)
{
    char base_path[MAX_FILENAME_LENGTH];
    char file_path[MAX_FILENAME_LENGTH];
    FILE *file = NULL;
    long current_size = 0;
    int ret;

    if ((dirname == NULL) || (file_prefix == NULL) || (data == NULL))
    {
        ESP_LOGE(TAG, "Parâmetros inválidos para gravação com rotação.");
        return false;
    }
    ret = snprintf(base_path, MAX_FILENAME_LENGTH, "%s/%s", MOUNT_POINT, dirname);
    if (ret < 0)
    {
        ESP_LOGE(TAG, "Erro ao montar caminho do diretório.");
        return false;
    }
    if (!sd_storage_module_create_directory(dirname))
    {
        ESP_LOGE(TAG, "Diretório %s não pode ser criado.", base_path);
        return false;
    }
    ret = snprintf(file_path, MAX_FILENAME_LENGTH, "%s/%s%s", base_path, file_prefix, LOG_FILE_EXT);
    if (ret < 0)
    {
        ESP_LOGE(TAG, "Erro ao montar caminho do arquivo.");
        return false;
    }
    if (xSemaphoreTake(sd_mutex, portMAX_DELAY) == pdTRUE)
    {
        file = fopen(file_path, "r+");
        if (file != NULL)
        {
            if (fseek(file, 0, SEEK_END) != 0)
            {
                ESP_LOGE(TAG, "Erro ao buscar fim do arquivo %s.", file_path);
                fclose(file);
                xSemaphoreGive(sd_mutex);
                return false;
            }
            current_size = ftell(file);
            if (current_size < 0)
            {
                ESP_LOGE(TAG, "Erro ao obter tamanho do arquivo %s.", file_path);
                fclose(file);
                xSemaphoreGive(sd_mutex);
                return false;
            }
            fclose(file);
        }
        xSemaphoreGive(sd_mutex);
    }
    else
    {
        ESP_LOGE(TAG, "Falha ao adquirir mutex para verificar tamanho do arquivo.");
        return false;
    }
    if ((unsigned long)current_size >= sd_config.max_log_file_size)
    {
        uint32_t timestamp = esp_log_timestamp();
        ret = snprintf(file_path, MAX_FILENAME_LENGTH, "%s/%s_%u%s", base_path, file_prefix, timestamp, LOG_FILE_EXT);
        if (ret < 0)
        {
            ESP_LOGE(TAG, "Erro ao montar novo nome do arquivo para rotação.");
            return false;
        }
    }
    if (xSemaphoreTake(sd_mutex, portMAX_DELAY) == pdTRUE)
    {
        file = fopen(file_path, "a");
        if (file == NULL)
        {
            ESP_LOGE(TAG, "Falha ao abrir arquivo %s para gravação.", file_path);
            xSemaphoreGive(sd_mutex);
            return false;
        }
        fprintf(file, "%s\n", data);
        fclose(file);
        xSemaphoreGive(sd_mutex);
        ESP_LOGI(TAG, "Dados gravados com sucesso em %s", file_path);
        if (write_callback != NULL)
        {
            write_callback(file_path, data);
        }
        return true;
    }
    return false;
}

/**
 * @brief Realiza a limpeza automática de arquivos antigos no diretório especificado,
 *        caso o espaço livre esteja abaixo do limiar configurado.
 *
 * Percorre o diretório e remove os arquivos mais antigos até que o espaço livre seja
 * superior ao limiar.
 *
 * @param dirname Diretório dos arquivos de log.
 * @return true se a limpeza for bem-sucedida, false caso contrário.
 */
bool sd_storage_module_cleanup_logs(const char *dirname)
{
    DIR *dir;
    struct dirent *entry;
    char path[MAX_FILENAME_LENGTH];
    struct stat file_stat;
    uint32_t free_space;
    struct statvfs vfs;

    if (dirname == NULL)
    {
        ESP_LOGE(TAG, "Nome do diretório nulo para limpeza.");
        return false;
    }
    if (statvfs(MOUNT_POINT, &vfs) != 0)
    {
        ESP_LOGE(TAG, "Erro ao obter informações do sistema de arquivos.");
        return false;
    }
    free_space = (uint32_t)(vfs.f_bsize * vfs.f_bfree);
    if (free_space >= sd_config.free_space_threshold)
    {
        return true;
    }
    (void)snprintf(path, MAX_FILENAME_LENGTH, "%s/%s", MOUNT_POINT, dirname);
    dir = opendir(path);
    if (dir == NULL)
    {
        ESP_LOGE(TAG, "Erro ao abrir diretório %s para limpeza.", path);
        return false;
    }
    while (free_space < sd_config.free_space_threshold)
    {
        DIR *inner_dir = opendir(path);
        struct dirent *oldest = NULL;
        time_t oldest_time = INT_MAX;
        if (inner_dir == NULL)
        {
            ESP_LOGE(TAG, "Erro ao reabrir diretório %s para limpeza.", path);
            closedir(dir);
            return false;
        }
        while ((entry = readdir(inner_dir)) != NULL)
        {
            if (entry->d_type == DT_REG)
            {
                char file_path[MAX_FILENAME_LENGTH];
                (void)snprintf(file_path, MAX_FILENAME_LENGTH, "%s/%s", path, entry->d_name);
                if (stat(file_path, &file_stat) == 0)
                {
                    if (file_stat.st_mtime < oldest_time)
                    {
                        oldest_time = file_stat.st_mtime;
                        oldest = entry;
                    }
                }
            }
        }
        closedir(inner_dir);
        if (oldest == NULL)
        {
            ESP_LOGW(TAG, "Nenhum arquivo encontrado para remoção em %s.", path);
            break;
        }
        else
        {
            char oldest_path[MAX_FILENAME_LENGTH];
            (void)snprintf(oldest_path, MAX_FILENAME_LENGTH, "%s/%s", path, oldest->d_name);
            if (remove(oldest_path) == 0)
            {
                ESP_LOGI(TAG, "Arquivo %s removido para liberar espaço.", oldest_path);
            }
            else
            {
                ESP_LOGE(TAG, "Erro ao remover arquivo %s.", oldest_path);
                break;
            }
        }
        if (statvfs(MOUNT_POINT, &vfs) == 0)
        {
            free_space = (uint32_t)(vfs.f_bsize * vfs.f_bfree);
        }
        else
        {
            ESP_LOGE(TAG, "Erro ao atualizar informações do sistema de arquivos.");
            break;
        }
    }
    closedir(dir);
    return (free_space >= sd_config.free_space_threshold);
}

/*==============================================================================*/
/*                    FUNÇÕES DE TIMESTAMP E FORMATOS PADRONIZADOS             */
/*==============================================================================*/

/**
 * @brief Obtém o timestamp atual formatado no padrão "YYYY-MM-DD HH:MM:SS".
 *
 * Utiliza time() e localtime_r() para formatar o tempo atual.
 *
 * @param buffer Buffer onde o timestamp será armazenado.
 * @param max_len Tamanho máximo do buffer.
 * @return true se obtido com sucesso, false caso contrário.
 */
bool sd_storage_module_get_formatted_timestamp(char *buffer, size_t max_len)
{
    time_t now = time(NULL);
    struct tm timeinfo;
    if (localtime_r(&now, &timeinfo) == NULL)
    {
        ESP_LOGE(TAG, "Erro ao obter tempo local.");
        return false;
    }
    if (strftime(buffer, max_len, "%Y-%m-%d %H:%M:%S", &timeinfo) == 0)
    {
        ESP_LOGE(TAG, "Erro ao formatar o timestamp.");
        return false;
    }
    return true;
}

/**
 * @brief Grava uma entrada de log em formato CSV.
 *
 * Prepara a linha CSV com timestamp e envia para escrita com rotação.
 *
 * @param dirname Diretório de gravação.
 * @param file_prefix Prefixo do arquivo.
 * @param csv_entry Linha CSV (sem timestamp).
 * @return true se a gravação ocorrer com sucesso, false caso contrário.
 */
bool sd_storage_module_write_csv(const char *dirname, const char *file_prefix, const char *csv_entry)
{
    char timestamp[32];
    char formatted_entry[ASYNC_WRITE_MAX_DATA_LENGTH + 64];

    if (!sd_storage_module_get_formatted_timestamp(timestamp, sizeof(timestamp)))
    {
        ESP_LOGE(TAG, "Erro ao obter timestamp para CSV.");
        return false;
    }
    (void)snprintf(formatted_entry, sizeof(formatted_entry), "%s,%s", timestamp, csv_entry);
    return sd_storage_module_write_with_rotation(dirname, file_prefix, formatted_entry);
}

/**
 * @brief Grava uma entrada de log em formato JSON.
 *
 * Cria objeto JSON com timestamp e dados, e grava com rotação automática.
 *
 * @param dirname Diretório de gravação.
 * @param file_prefix Prefixo do arquivo.
 * @param json_entry Entrada JSON (sem timestamp).
 * @return true se a gravação ocorrer com sucesso, false caso contrário.
 */
bool sd_storage_module_write_json(const char *dirname, const char *file_prefix, const char *json_entry)
{
    char timestamp[32];
    char formatted_entry[ASYNC_WRITE_MAX_DATA_LENGTH + 128];

    if (!sd_storage_module_get_formatted_timestamp(timestamp, sizeof(timestamp)))
    {
        ESP_LOGE(TAG, "Erro ao obter timestamp para JSON.");
        return false;
    }
    (void)snprintf(formatted_entry, sizeof(formatted_entry),
                   "{\"timestamp\":\"%s\",\"data\":%s}", timestamp, json_entry);
    return sd_storage_module_write_with_rotation(dirname, file_prefix, formatted_entry);
}

/*==============================================================================*/
/*                            FIM DAS IMPLEMENTAÇÕES                           */
/*==============================================================================*/
