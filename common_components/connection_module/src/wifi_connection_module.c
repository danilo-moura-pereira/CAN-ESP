/**
 * @file wifi_connection_module.c
 * @brief Implementação do Módulo de Conectividade Wi‑Fi para a rede CAN‑ESP.
 *
 * Este módulo gerencia a conexão Wi‑Fi do ESP32 utilizando parâmetros lidos do arquivo "config.ini"
 * (via sd_storage_module). Implementa monitoramento ativo da conexão, uma tarefa de reconexão com
 * política de backoff progressivo e persiste informações da última conexão bem-sucedida na NVS.
 * Desenvolvido em conformidade com MISRA C:2012 e ISO 11898 para redes CAN de veículos elétricos reais.
 *
 * @note A configuração é lida a partir do arquivo "config.ini" no SD Card.
 */

#include "wifi_connection_module.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "sd_storage_module.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define TAG "WIFI_CONN_MODULE"

/* Tamanho máximo da linha de configuração */
#define CONFIG_LINE_MAX_LEN 128U
#define CONFIG_FILE "config.ini"

/* Event Group bits para monitoramento da conexão */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

/* Delay inicial para reconexão (em ms) e fator de multiplicação para backoff */
#define WIFI_RECONNECT_DELAY_MS 1000U
#define WIFI_BACKOFF_FACTOR     2U

/* Mutex para proteção dos parâmetros de configuração Wi‑Fi */
static SemaphoreHandle_t wifi_config_mutex = NULL;

/* Estrutura interna para armazenar a configuração Wi‑Fi */
static wifi_config_params_t wifi_config_params = {
    .ssid = "DEFAULT_SSID",
    .password = "DEFAULT_PASS",
    .channel = 1,
    .authmode = WIFI_AUTH_WPA2_PSK,
    .auto_connect = true,
    .maximum_retry = 5U
};

/* Contador de tentativas de reconexão */
static uint32_t connection_retry_count = 0U;

/* Event group para monitoramento da conexão */
#include "freertos/event_groups.h"
static EventGroupHandle_t wifi_event_group = NULL;

/* Protótipos de funções internas */
static esp_err_t wifi_event_handler(void *arg, esp_event_base_t event_base,
                                      int32_t event_id, void *event_data);
static bool wifi_connection_module_load_config_from_file(void);
static void wifi_reconnect_task(void *arg);
static bool wifi_store_connection_info(const wifi_connection_info_t *info);
static bool wifi_load_connection_info(wifi_connection_info_t *info);

/**
 * @brief Handler de eventos Wi‑Fi e IP.
 *
 * Trata eventos relacionados à conexão Wi‑Fi, como início, desconexão e obtenção de IP.
 *
 * @param arg Parâmetro não utilizado.
 * @param event_base Base do evento.
 * @param event_id Identificador do evento.
 * @param event_data Dados do evento.
 * @return ESP_OK para indicar sucesso.
 */
static esp_err_t wifi_event_handler(void *arg, esp_event_base_t event_base,
                                      int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                ESP_LOGI(TAG, "Wi‑Fi iniciado, tentando conectar...");
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGW(TAG, "Desconectado do Wi‑Fi.");
                xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
                /* A tarefa de reconexão cuidará do backoff progressivo */
                break;
            default:
                break;
        }
    }
    else if (event_base == IP_EVENT)
    {
        if (event_id == IP_EVENT_STA_GOT_IP)
        {
            connection_retry_count = 0;
            ESP_LOGI(TAG, "Conectado ao Wi‑Fi. IP atribuído.");
            xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
            /* Persiste informações da última conexão */
            {
                wifi_event_sta_got_ip_t *event = (wifi_event_sta_got_ip_t *) event_data;
                wifi_connection_info_t conn_info;
                (void)strncpy(conn_info.ssid, wifi_config_params.ssid, sizeof(conn_info.ssid) - 1);
                snprintf(conn_info.ip, sizeof(conn_info.ip), IPSTR, IP2STR(&event->ip_info.ip));
                (void)wifi_store_connection_info(&conn_info);
            }
        }
    }
    return ESP_OK;
}

/**
 * @brief Carrega os parâmetros de configuração Wi‑Fi do arquivo de configuração.
 *
 * Lê o arquivo CONFIG_FILE_PATH (no diretório de montagem do SD Card) e extrai os valores para as chaves:
 * - WIFI_SSID
 * - WIFI_PASSWORD
 * - WIFI_CHANNEL
 * - WIFI_AUTHMODE
 * - WIFI_AUTO_CONNECT
 * - WIFI_MAXIMUM_RETRY
 *
 * @return true se a configuração for carregada com sucesso, false caso contrário.
 */
static bool wifi_connection_module_load_config_from_file(void)
{
    char config_line[CONFIG_LINE_MAX_LEN];
    char config_path[MAX_FILENAME_LENGTH];
    FILE *file;

    (void)snprintf(config_path, MAX_FILENAME_LENGTH, "%s/%s", MOUNT_POINT, CONFIG_FILE);
    file = fopen(config_path, "r");
    if (file == NULL)
    {
        ESP_LOGW(TAG, "Arquivo de configuração %s não encontrado, utilizando valores padrão.", config_path);
        return false;
    }

    while (fgets(config_line, CONFIG_LINE_MAX_LEN, file) != NULL)
    {
        /* Remove quebra de linha */
        char *newline = strchr(config_line, '\n');
        if (newline != NULL)
        {
            *newline = '\0';
        }
        if (strncmp(config_line, "WIFI_SSID=", 10) == 0)
        {
            (void)strncpy(wifi_config_params.ssid, config_line + 10, sizeof(wifi_config_params.ssid) - 1);
        }
        else if (strncmp(config_line, "WIFI_PASSWORD=", 14) == 0)
        {
            (void)strncpy(wifi_config_params.password, config_line + 14, sizeof(wifi_config_params.password) - 1);
        }
        else if (strncmp(config_line, "WIFI_CHANNEL=", 13) == 0)
        {
            wifi_config_params.channel = (uint8_t)atoi(config_line + 13);
        }
        else if (strncmp(config_line, "WIFI_AUTHMODE=", 14) == 0)
        {
            wifi_config_params.authmode = (uint8_t)atoi(config_line + 14);
        }
        else if (strncmp(config_line, "WIFI_AUTO_CONNECT=", 18) == 0)
        {
            if (strncasecmp(config_line + 18, "true", 4) == 0)
            {
                wifi_config_params.auto_connect = true;
            }
            else
            {
                wifi_config_params.auto_connect = false;
            }
        }
        else if (strncmp(config_line, "WIFI_MAXIMUM_RETRY=", 19) == 0)
        {
            wifi_config_params.maximum_retry = (uint32_t)atoi(config_line + 19);
        }
    }
    fclose(file);
    ESP_LOGI(TAG, "Configuração Wi‑Fi carregada a partir de %s.", config_path);
    return true;
}

/**
 * @brief Inicializa a conectividade Wi‑Fi utilizando os parâmetros do arquivo de configuração.
 *
 * Configura o Wi‑Fi em modo STA, registra os handlers de eventos e inicia o driver Wi‑Fi.
 *
 * @return true se a inicialização for bem-sucedida, false caso contrário.
 */
bool wifi_connection_module_init(void)
{
    esp_err_t ret;

    /* Inicializa NVS */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao inicializar NVS: %s", esp_err_to_name(ret));
        return false;
    }

    /* Cria o event group para monitoramento da conexão */
    wifi_event_group = xEventGroupCreate();
    if (wifi_event_group == NULL)
    {
        ESP_LOGE(TAG, "Falha ao criar event group para Wi‑Fi.");
        return false;
    }

    if (wifi_config_mutex == NULL)
    {
        wifi_config_mutex = xSemaphoreCreateMutex();
        if (wifi_config_mutex == NULL)
        {
            ESP_LOGE(TAG, "Falha ao criar mutex de configuração Wi‑Fi.");
            return false;
        }
    }

    /* Carrega configuração Wi‑Fi do arquivo */
    (void)wifi_connection_module_load_config_from_file();

    /* Inicializa a pilha TCP/IP */
    ESP_ERROR_CHECK(esp_netif_init());

    /* Cria a interface Wi‑Fi STA */
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_WIFI_STA();
    esp_netif_t *sta_netif = esp_netif_new(&netif_cfg);
    if (sta_netif == NULL)
    {
        ESP_LOGE(TAG, "Falha ao criar interface de rede Wi‑Fi.");
        return false;
    }
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao inicializar Wi‑Fi: %s", esp_err_to_name(ret));
        return false;
    }

    /* Registra handlers de eventos */
    ret = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao registrar handler de eventos Wi‑Fi: %s", esp_err_to_name(ret));
        return false;
    }
    ret = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao registrar handler de eventos IP: %s", esp_err_to_name(ret));
        return false;
    }

    /* Configuração Wi‑Fi */
    wifi_config_t wifi_config = {0};
    (void)strncpy((char *)wifi_config.sta.ssid, wifi_config_params.ssid, sizeof(wifi_config.sta.ssid) - 1);
    (void)strncpy((char *)wifi_config.sta.password, wifi_config_params.password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.channel = wifi_config_params.channel;
    wifi_config.sta.threshold.authmode = wifi_config_params.authmode;
    wifi_config.sta.auto_connect = wifi_config_params.auto_connect;

    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao definir modo STA: %s", esp_err_to_name(ret));
        return false;
    }
    ret = esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao configurar Wi‑Fi: %s", esp_err_to_name(ret));
        return false;
    }
    ret = esp_wifi_start();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao iniciar Wi‑Fi: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "Conectividade Wi‑Fi iniciada. SSID: %s, Canal: %u, Auto Connect: %s, Maximum Retry: %u",
             wifi_config_params.ssid, wifi_config_params.channel,
             wifi_config_params.auto_connect ? "true" : "false", wifi_config_params.maximum_retry);

    /* Inicia a tarefa de reconexão avançada */
    wifi_connection_module_start_reconnect_task();

    return true;
}

/**
 * @brief Tarefa de reconexão Wi‑Fi com backoff progressivo.
 *
 * Monitora a conexão Wi‑Fi e, em caso de desconexão, tenta reconectar utilizando delay progressivo.
 *
 * @param arg Parâmetro não utilizado.
 */
static void wifi_reconnect_task(void *arg)
{
    uint32_t delay_ms = WIFI_RECONNECT_DELAY_MS;
    EventBits_t bits;
    for (;;)
    {
        /* Aguarda por evento de desconexão ou timeout para reconectar */
        bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(delay_ms));
        if ((bits & WIFI_CONNECTED_BIT) == 0)
        {
            ESP_LOGW(TAG, "Reconectando Wi‑Fi após %u ms...", delay_ms);
            esp_wifi_connect();
            connection_retry_count++;
            /* Aplica backoff progressivo */
            delay_ms *= WIFI_BACKOFF_FACTOR;
            if (connection_retry_count >= wifi_config_params.maximum_retry)
            {
                ESP_LOGE(TAG, "Número máximo de tentativas de reconexão atingido.");
                /* Reinicializa a conexão (pode-se implementar lógica de reinicialização do módulo se necessário) */
                connection_retry_count = 0;
                delay_ms = WIFI_RECONNECT_DELAY_MS;
            }
        }
        else
        {
            ESP_LOGI(TAG, "Conexão Wi‑Fi estabelecida.");
            connection_retry_count = 0;
            delay_ms = WIFI_RECONNECT_DELAY_MS;
        }
    }
}

/**
 * @brief Inicia a tarefa de reconexão Wi‑Fi com backoff progressivo.
 */
void wifi_connection_module_start_reconnect_task(void)
{
    if (xTaskCreate(wifi_reconnect_task, "WiFi_Reconnect_Task", 4096, NULL, 5, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "Falha ao criar a tarefa de reconexão Wi‑Fi.");
    }
    else
    {
        ESP_LOGI(TAG, "Tarefa de reconexão Wi‑Fi iniciada com sucesso.");
    }
}

/**
 * @brief Persiste as informações da última conexão Wi‑Fi bem-sucedida na NVS.
 *
 * Salva os dados (SSID e IP) na NVS para que possam ser recuperados posteriormente.
 *
 * @param info Ponteiro para a estrutura com as informações de conexão.
 * @return true se os dados forem salvos com sucesso, false caso contrário.
 */
bool wifi_connection_module_store_connection_info(const wifi_connection_info_t *info)
{
    if (info == NULL)
    {
        return false;
    }
    nvs_handle_t handle;
    esp_err_t err = nvs_open("wifi_conn", NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao abrir NVS para conexão Wi‑Fi: %s", esp_err_to_name(err));
        return false;
    }
    err = nvs_set_blob(handle, "conn_info", info, sizeof(wifi_connection_info_t));
    if (err == ESP_OK)
    {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao salvar informações de conexão: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "Informações de conexão salvas com sucesso.");
    return true;
}

/**
 * @brief Carrega as informações da última conexão Wi‑Fi bem-sucedida da NVS.
 *
 * Recupera os dados salvos na NVS e os copia para a estrutura fornecida.
 *
 * @param info Ponteiro para a estrutura que receberá os dados de conexão.
 * @return true se os dados forem carregados com sucesso, false caso contrário.
 */
bool wifi_connection_module_load_connection_info(wifi_connection_info_t *info)
{
    if (info == NULL)
    {
        return false;
    }
    nvs_handle_t handle;
    size_t required_size = sizeof(wifi_connection_info_t);
    esp_err_t err = nvs_open("wifi_conn", NVS_READONLY, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao abrir NVS para carregar informações de conexão: %s", esp_err_to_name(err));
        return false;
    }
    err = nvs_get_blob(handle, "conn_info", info, &required_size);
    nvs_close(handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao carregar informações de conexão: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "Informações de conexão carregadas com sucesso: SSID=%s, IP=%s", info->ssid, info->ip);
    return true;
}

/**
 * @brief Finaliza a conectividade Wi‑Fi e libera os recursos.
 *
 * Encerra a conexão Wi‑Fi, desconfigura os handlers de eventos e libera a interface.
 *
 * @return true se a operação for bem-sucedida, false caso contrário.
 */
bool wifi_connection_module_deinit(void)
{
    esp_err_t ret = esp_wifi_stop();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao parar Wi‑Fi: %s", esp_err_to_name(ret));
        return false;
    }
    ret = esp_wifi_deinit();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao desinicializar Wi‑Fi: %s", esp_err_to_name(ret));
        return false;
    }
    ESP_LOGI(TAG, "Conectividade Wi‑Fi finalizada com sucesso.");
    return true;
}

/**
 * @brief Obtém a configuração Wi‑Fi atualmente em uso.
 *
 * Copia os parâmetros de configuração atuais para a estrutura fornecida.
 *
 * @param[out] config Ponteiro para a estrutura que receberá os parâmetros.
 * @return true se a operação for bem-sucedida, false caso contrário.
 */
bool wifi_connection_module_get_config(wifi_config_params_t *config)
{
    if (config == NULL)
    {
        return false;
    }
    if (xSemaphoreTake(wifi_config_mutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        *config = wifi_config_params;
        xSemaphoreGive(wifi_config_mutex);
        return true;
    }
    ESP_LOGW(TAG, "Falha ao adquirir mutex para acessar configuração Wi‑Fi.");
    return false;
}

#ifdef __cplusplus
}
#endif
