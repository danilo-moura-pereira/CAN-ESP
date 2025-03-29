/**
 * @file monitor_ecu.c
 * @brief Implementação da ECU de Monitoramento e Diagnóstico para o projeto CAN-ESP.
 *
 * Este componente integra os módulos críticos do sistema CAN-ESP, incluindo Wi-Fi, MQTT, ESP-MESH,
 * Routing, OTA, SD Storage, Diagnosis, Logger e Alert, para viabilizar o funcionamento da ECU de 
 * Monitoramento e Diagnóstico (nó raiz) em veículos elétricos reais. Além do fluxo OTA (com retry, 
 * rollback, atualização dinâmica de parâmetros e otimização de tarefas) e da externalização das constantes 
 * de retry a partir do arquivo "config.ini", este módulo implementa também o fluxo de aquisição de dados 
 * da rede CAN, capturando as mensagens que transitam na rede e atualizando periodicamente os dados diagnósticos.
 *
 * Os parâmetros configuráveis externalizados (com prefixo MONITOR_) incluem:
 *   - MONITOR_MAX_RETRY_COUNT
 *   - MONITOR_RETRY_DELAY_MS
 *   - MONITOR_CONFIG_CHECK_INTERVAL_MS
 *   - MONITOR_DIAG_PERSIST_INTERVAL_MS
 *   - MONITOR_CAN_RECEIVE_TIMEOUT_MS
 *   - MONITOR_DIAG_ACQ_INTERVAL_MS
 *   - MONITOR_COMM_INTERVAL_MS
 *
 * @note A função monitor_ecu_init() deve ser chamada durante a inicialização do sistema.
 */

#include "monitor_ecu.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wifi_connection_module.h"
#include "mqtt_connection_module.h"
#include "esp_mesh_connection_module.h"
#include "routing_module.h"
#include "ota_module.h"
#include "sd_storage_module.h"
#include "diagnosis_module.h"
#include "logger_module.h"
#include <string.h>
#include <stdio.h>

/* Tag para logs */
#define TAG "MONITOR_ECU"

/* Definições para tamanho máximo do arquivo de configuração */
#define MAX_CONFIG_SIZE           1024U

/* Parâmetros de retry já externalizados */
static uint32_t g_monitor_max_retry_count = 3U;
static uint32_t g_monitor_retry_delay_ms  = 2000U;

/* Novos parâmetros configuráveis (com prefixo MONITOR_) */
static uint32_t g_monitor_config_check_interval_ms = 300000U; /* Intervalo para config_update_task */
static uint32_t g_monitor_diag_persist_interval_ms = 60000U;     /* Intervalo para persistência diagnóstica */
static uint32_t g_monitor_can_receive_timeout_ms = 10U;          /* Timeout para CAN receive */
static uint32_t g_monitor_diag_acq_interval_ms = 1000U;          /* Intervalo para diagnosis_acquisition_task */
static uint32_t g_monitor_comm_interval_ms = 1000U;              /* Intervalo para communication_task */

/* Definições para tasks otimizadas */
#define OTA_TASK_STACK_SIZE       3072U
#define OTA_TASK_PRIORITY         3U

#define COMM_TASK_STACK_SIZE      3072U
#define COMM_TASK_PRIORITY        4U

#define CONFIG_TASK_STACK_SIZE    2048U
#define CONFIG_TASK_PRIORITY      2U

#define CAN_ACQ_TASK_STACK_SIZE   3072U
#define CAN_ACQ_TASK_PRIORITY     3U

#define DIAG_ACQ_TASK_STACK_SIZE  3072U
#define DIAG_ACQ_TASK_PRIORITY    3U

/* Estrutura para armazenar estatísticas de aquisição CAN */
typedef struct
{
    uint32_t total_messages_received;
} CanAcquisitionStats_t;

static CanAcquisitionStats_t can_stats = { 0 };

/* Variável para controle do último log persistente dos diagnósticos (em ms) */
static uint32_t last_diag_persist_time_ms = 0U;

/**
 * @brief Carrega os parâmetros configuráveis do módulo monitor_ecu a partir do arquivo "config.ini".
 *
 * Lê as seguintes chaves com prefixo MONITOR_:
 *   - MONITOR_MAX_RETRY_COUNT
 *   - MONITOR_RETRY_DELAY_MS
 *   - MONITOR_CONFIG_CHECK_INTERVAL_MS
 *   - MONITOR_DIAG_PERSIST_INTERVAL_MS
 *   - MONITOR_CAN_RECEIVE_TIMEOUT_MS
 *   - MONITOR_DIAG_ACQ_INTERVAL_MS
 *   - MONITOR_COMM_INTERVAL_MS
 *
 * Caso algum valor não seja encontrado ou seja inválido (zero), o sistema mantém o valor padrão.
 */
static void load_monitor_parameters(void)
{
    uint32_t config_size = 0U;
    uint8_t *config_data = sd_storage_module_read_file("config.ini", &config_size);

    if ((config_data == NULL) || (config_size == 0U))
    {
        ESP_LOGW(TAG, "Arquivo 'config.ini' não encontrado. Usando valores padrão para parâmetros MONITOR_.");
        return;
    }

    char buffer[MAX_CONFIG_SIZE + 1];
    size_t copy_size = (config_size < MAX_CONFIG_SIZE) ? config_size : MAX_CONFIG_SIZE;
    (void)memcpy(buffer, config_data, copy_size);
    buffer[copy_size] = '\0';

    char *line = strtok(buffer, "\n");
    uint32_t value = 0U;
    while (line != NULL)
    {
        if (sscanf(line, "MONITOR_MAX_RETRY_COUNT=%u", &value) == 1)
        {
            if (value > 0U) { g_monitor_max_retry_count = value; }
            else { ESP_LOGW(TAG, "Valor inválido para MONITOR_MAX_RETRY_COUNT: %u. Mantendo padrão: %u", value, g_monitor_max_retry_count); }
        }
        else if (sscanf(line, "MONITOR_RETRY_DELAY_MS=%u", &value) == 1)
        {
            if (value > 0U) { g_monitor_retry_delay_ms = value; }
            else { ESP_LOGW(TAG, "Valor inválido para MONITOR_RETRY_DELAY_MS: %u. Mantendo padrão: %u", value, g_monitor_retry_delay_ms); }
        }
        else if (sscanf(line, "MONITOR_CONFIG_CHECK_INTERVAL_MS=%u", &value) == 1)
        {
            if (value > 0U) { g_monitor_config_check_interval_ms = value; }
            else { ESP_LOGW(TAG, "Valor inválido para MONITOR_CONFIG_CHECK_INTERVAL_MS: %u. Mantendo padrão: %u", value, g_monitor_config_check_interval_ms); }
        }
        else if (sscanf(line, "MONITOR_DIAG_PERSIST_INTERVAL_MS=%u", &value) == 1)
        {
            if (value > 0U) { g_monitor_diag_persist_interval_ms = value; }
            else { ESP_LOGW(TAG, "Valor inválido para MONITOR_DIAG_PERSIST_INTERVAL_MS: %u. Mantendo padrão: %u", value, g_monitor_diag_persist_interval_ms); }
        }
        else if (sscanf(line, "MONITOR_CAN_RECEIVE_TIMEOUT_MS=%u", &value) == 1)
        {
            if (value > 0U) { g_monitor_can_receive_timeout_ms = value; }
            else { ESP_LOGW(TAG, "Valor inválido para MONITOR_CAN_RECEIVE_TIMEOUT_MS: %u. Mantendo padrão: %u", value, g_monitor_can_receive_timeout_ms); }
        }
        else if (sscanf(line, "MONITOR_DIAG_ACQ_INTERVAL_MS=%u", &value) == 1)
        {
            if (value > 0U) { g_monitor_diag_acq_interval_ms = value; }
            else { ESP_LOGW(TAG, "Valor inválido para MONITOR_DIAG_ACQ_INTERVAL_MS: %u. Mantendo padrão: %u", value, g_monitor_diag_acq_interval_ms); }
        }
        else if (sscanf(line, "MONITOR_COMM_INTERVAL_MS=%u", &value) == 1)
        {
            if (value > 0U) { g_monitor_comm_interval_ms = value; }
            else { ESP_LOGW(TAG, "Valor inválido para MONITOR_COMM_INTERVAL_MS: %u. Mantendo padrão: %u", value, g_monitor_comm_interval_ms); }
        }
        line = strtok(NULL, "\n");
    }

    ESP_LOGI(TAG, "Parâmetros MONITOR_ carregados: MAX_RETRY_COUNT=%u, RETRY_DELAY_MS=%u, CONFIG_CHECK_INTERVAL_MS=%u, DIAG_PERSIST_INTERVAL_MS=%u, CAN_RECEIVE_TIMEOUT_MS=%u, DIAG_ACQ_INTERVAL_MS=%u, COMM_INTERVAL_MS=%u",
             g_monitor_max_retry_count, g_monitor_retry_delay_ms, g_monitor_config_check_interval_ms,
             g_monitor_diag_persist_interval_ms, g_monitor_can_receive_timeout_ms,
             g_monitor_diag_acq_interval_ms, g_monitor_comm_interval_ms);
    sd_storage_module_free_buffer(config_data);
}

/**
 * @brief Callback para eventos OTA.
 *
 * Processa os eventos do fluxo OTA.
 *
 * @param status Status da atualização OTA.
 * @param ecu_id Identificador da ECU.
 * @param data Dados adicionais (pode ser NULL).
 */
static void ota_event_handler(ota_status_t status, const char *ecu_id, const void *data)
{
    (void)data;
    switch (status)
    {
        case OTA_STATUS_UPDATE_AVAILABLE:
            ESP_LOGI(TAG, "OTA update available for ECU: %s", ecu_id);
            break;
        case OTA_STATUS_DOWNLOADING:
            ESP_LOGI(TAG, "OTA downloading firmware for ECU: %s", ecu_id);
            break;
        case OTA_STATUS_DISTRIBUTING:
            ESP_LOGI(TAG, "OTA distributing firmware to ECU: %s", ecu_id);
            break;
        case OTA_STATUS_APPLYING:
            ESP_LOGI(TAG, "OTA applying firmware update on ECU: %s", ecu_id);
            break;
        case OTA_STATUS_SUCCESS:
            ESP_LOGI(TAG, "OTA update SUCCESS for ECU: %s", ecu_id);
            break;
        case OTA_STATUS_FAILURE:
            ESP_LOGE(TAG, "OTA update FAILURE for ECU: %s", ecu_id);
            break;
        case OTA_STATUS_ROLLBACK:
            ESP_LOGW(TAG, "OTA update ROLLBACK for ECU: %s", ecu_id);
            break;
        default:
            ESP_LOGW(TAG, "OTA unknown status (%d) for ECU: %s", status, ecu_id);
            break;
    }
}

/**
 * @brief Realiza o rollback da atualização OTA.
 *
 * Aciona rollback em caso de falha irreversível.
 *
 * @param ecu_id Identificador da ECU.
 * @return true se o rollback for bem-sucedido; false, caso contrário.
 */
static bool perform_rollback(const char *ecu_id)
{
    bool rollback_result = ota_module_rollback_update(ecu_id);
    if (rollback_result)
    {
        ESP_LOGI(TAG, "Rollback efetuado com sucesso para ECU: %s", ecu_id);
    }
    else
    {
        ESP_LOGE(TAG, "Rollback falhou para ECU: %s", ecu_id);
    }
    return rollback_result;
}

/**
 * @brief Task de aquisição de mensagens CAN.
 *
 * Captura continuamente as mensagens que transitam na rede CAN-ESP utilizando CAN_ESP_ReceiveMessage().
 * Atualiza estatísticas e registra os dados em nível DEBUG, utilizando identificador CAN estendido (29 bits).
 *
 * @param pvParameters Parâmetro da task (não utilizado).
 */
static void can_acquisition_task(void *pvParameters)
{
    (void)pvParameters;
    CanEspMessage_t msg;
    uint8_t priority = 0;
    uint16_t ecu_id = 0;
    uint16_t command_id = 0;

    for (;;)
    {
        if (CAN_ESP_ReceiveMessage(&msg, pdMS_TO_TICKS(g_monitor_can_receive_timeout_ms)) == CAN_ESP_OK)
        {
            can_stats.total_messages_received++;
            /* Decodifica o identificador estendido de 29 bits */
            priority = (uint8_t)((msg.id >> 26) & 0x07U);
            ecu_id = (uint16_t)((msg.id >> 16) & 0x03FFU);
            command_id = (uint16_t)(msg.id & 0xFFFFU);
            ESP_LOGD(TAG, "CAN Acquisition: Msg received - Ext ID: 0x%08X, Priority: %u, ECU ID: 0x%03X, Command: 0x%04X, Length: %u, Total: %u",
                     msg.id, priority, ecu_id, command_id, msg.length, can_stats.total_messages_received);
            /* Processamento adicional pode ser adicionado aqui */
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/**
 * @brief Task de aquisição e persistência dos dados diagnósticos.
 *
 * Periodicamente, invoca diagnosis_module_update() para coletar os dados diagnósticos da rede CAN.
 * Se os dados indicarem uma condição anormal ou se o intervalo de persistência for atingido, gera
 * um resumo e o armazena de forma assíncrona utilizando logger_module_async_write().
 *
 * @param pvParameters Parâmetro da task (não utilizado).
 */
static void diagnosis_acquisition_task(void *pvParameters)
{
    (void)pvParameters;
    DiagnosisData_t diag;
    uint32_t current_time_ms = 0U;
    for (;;)
    {
        if (diagnosis_module_update(&diag))
        {
            ESP_LOGI(TAG, "Diagnosis Update: Bus Load: %" PRIu32 "%%, TX Errors: %" PRIu32 ", RX Errors: %" PRIu32,
                     diag.bus_load, diag.can_diag.tx_error_counter, diag.can_diag.rx_error_counter);

            current_time_ms = (uint32_t)(esp_timer_get_time() / 1000U);
            if (diag.abnormal || (current_time_ms - last_diag_persist_time_ms >= g_monitor_diag_persist_interval_ms))
            {
                char diag_summary[256];
                (void)snprintf(diag_summary, sizeof(diag_summary),
                               "Diag Summary: Time=%u ms, Bus Load=%" PRIu32 "%%, TX_Err=%" PRIu32 ", RX_Err=%" PRIu32
                               ", Retrans=%" PRIu32 ", Collisions=%" PRIu32 ", Latency(Max)=%" PRId64 " us",
                               current_time_ms, diag.bus_load, diag.can_diag.tx_error_counter,
                               diag.can_diag.rx_error_counter, diag.retransmission_count,
                               diag.collision_count, diag.latency.max_latency);
                logger_module_async_write(diag_summary);
                last_diag_persist_time_ms = current_time_ms;
            }
        }
        else
        {
            ESP_LOGW(TAG, "Falha ao atualizar dados diagnósticos.");
        }
        vTaskDelay(pdMS_TO_TICKS(g_monitor_diag_acq_interval_ms));
    }
}

/**
 * @brief Task OTA para gerenciamento do fluxo de atualização remota.
 *
 * Executa o fluxo OTA com retry (download, segmentação, distribuição e aplicação)
 * e aciona rollback em caso de falhas irreversíveis.
 *
 * @param pvParameters Parâmetro da task (não utilizado).
 */
static void ota_task(void *pvParameters)
{
    (void)pvParameters;
    const TickType_t check_interval = pdMS_TO_TICKS(60000U);
    for (;;)
    {
        if (ota_module_check_update())
        {
            uint32_t retry_count = 0U;
            bool result = false;
            do
            {
                result = ota_module_download_firmware("monitor_ecu");
                if (!result)
                {
                    ESP_LOGW(TAG, "Falha no download, tentativa %u", retry_count + 1U);
                    retry_count++;
                    vTaskDelay(pdMS_TO_TICKS(g_monitor_retry_delay_ms));
                }
            } while (!result && (retry_count < g_monitor_max_retry_count));
            if (!result)
            {
                ESP_LOGE(TAG, "Download falhou após %u tentativas.", retry_count);
                vTaskDelay(check_interval);
                continue;
            }
            uint32_t firmware_size = 0U;
            uint8_t *firmware_data = sd_storage_module_read_file("firmware.bin", &firmware_size);
            if ((firmware_data == NULL) || (firmware_size == 0U))
            {
                ESP_LOGE(TAG, "Falha na leitura do firmware do SD Card.");
                vTaskDelay(check_interval);
                continue;
            }
            retry_count = 0U;
            do
            {
                result = ota_module_segment_firmware(firmware_data, firmware_size);
                if (!result)
                {
                    ESP_LOGW(TAG, "Falha na segmentação, tentativa %u", retry_count + 1U);
                    retry_count++;
                    vTaskDelay(pdMS_TO_TICKS(g_monitor_retry_delay_ms));
                }
            } while (!result && (retry_count < g_monitor_max_retry_count));
            if (!result)
            {
                ESP_LOGE(TAG, "Segmentação falhou após %u tentativas.", retry_count);
                (void)perform_rollback("monitor_ecu");
                sd_storage_module_free_buffer(firmware_data);
                vTaskDelay(check_interval);
                continue;
            }
            retry_count = 0U;
            do
            {
                result = ota_module_distribute_firmware("monitor_ecu");
                if (!result)
                {
                    ESP_LOGW(TAG, "Falha na distribuição, tentativa %u", retry_count + 1U);
                    retry_count++;
                    vTaskDelay(pdMS_TO_TICKS(g_monitor_retry_delay_ms));
                }
            } while (!result && (retry_count < g_monitor_max_retry_count));
            if (!result)
            {
                ESP_LOGE(TAG, "Distribuição falhou após %u tentativas.", retry_count);
                (void)perform_rollback("monitor_ecu");
                sd_storage_module_free_buffer(firmware_data);
                vTaskDelay(check_interval);
                continue;
            }
            retry_count = 0U;
            do
            {
                result = ota_module_apply_update("monitor_ecu");
                if (!result)
                {
                    ESP_LOGW(TAG, "Falha na aplicação, tentativa %u", retry_count + 1U);
                    retry_count++;
                    vTaskDelay(pdMS_TO_TICKS(g_monitor_retry_delay_ms));
                }
            } while (!result && (retry_count < g_monitor_max_retry_count));
            if (!result)
            {
                ESP_LOGE(TAG, "Aplicação falhou após %u tentativas.", retry_count);
                (void)perform_rollback("monitor_ecu");
                sd_storage_module_free_buffer(firmware_data);
                vTaskDelay(check_interval);
                continue;
            }
            ESP_LOGI(TAG, "Processo OTA concluído com sucesso para ECU: monitor_ecu");
            sd_storage_module_free_buffer(firmware_data);
        }
        vTaskDelay(check_interval);
    }
}

/**
 * @brief Task de comunicação para manutenção da rede CAN-ESP.
 *
 * Atualiza periodicamente as tabelas de roteamento.
 *
 * @param pvParameters Parâmetro da task (não utilizado).
 */
static void communication_task(void *pvParameters)
{
    (void)pvParameters;
    for (;;)
    {
        (void)routing_module_recalculate_routes();
        vTaskDelay(pdMS_TO_TICKS(g_monitor_comm_interval_ms));
    }
}

/**
 * @brief Task para atualização dinâmica dos parâmetros da ECU.
 *
 * Invoca periodicamente ota_module_refresh_config() para atualizar os parâmetros internos.
 *
 * @param pvParameters Parâmetro da task (não utilizado).
 */
static void config_update_task(void *pvParameters)
{
    (void)pvParameters;
    const TickType_t config_check_interval = pdMS_TO_TICKS(g_monitor_config_check_interval_ms);
    for (;;)
    {
        if (ota_module_refresh_config())
        {
            ESP_LOGI(TAG, "Configuração OTA atualizada com sucesso.");
        }
        else
        {
            ESP_LOGW(TAG, "Falha ao atualizar a configuração OTA.");
        }
        vTaskDelay(config_check_interval);
    }
}

/**
 * @brief Inicializa a ECU de Monitoramento e Diagnóstico.
 *
 * Inicializa os módulos de conectividade (Wi-Fi, MQTT, ESP-MESH), Routing, OTA, SD Storage,
 * Diagnosis, Logger e Alert, e inicia as tasks responsáveis pelo fluxo:
 * - OTA (com retry/rollback)
 * - Comunicação (atualização de rotas)
 * - Atualização dinâmica dos parâmetros
 * - Aquisição de mensagens CAN (com decodificação do identificador estendido)
 * - Aquisição e persistência dos dados diagnósticos (conforme critérios definidos)
 *
 * @return true se a inicialização for bem-sucedida; false caso contrário.
 */
bool monitor_ecu_init(void)
{
    ESP_LOGI(TAG, "Initializing Monitor ECU...");

    if (!wifi_connection_module_init())
    {
        ESP_LOGE(TAG, "Wi-Fi initialization failed.");
        return false;
    }
    ESP_LOGI(TAG, "Wi-Fi initialized successfully.");

    if (!mqtt_connection_module_init())
    {
        ESP_LOGE(TAG, "MQTT initialization failed.");
        return false;
    }
    ESP_LOGI(TAG, "MQTT initialized successfully.");

    if (!esp_mesh_connection_module_init())
    {
        ESP_LOGE(TAG, "ESP-MESH initialization failed.");
        return false;
    }
    ESP_LOGI(TAG, "ESP-MESH initialized successfully.");

    if (!routing_module_init())
    {
        ESP_LOGE(TAG, "Routing module initialization failed.");
        return false;
    }
    if (!routing_module_start())
    {
        ESP_LOGE(TAG, "Routing module task start failed.");
        return false;
    }
    ESP_LOGI(TAG, "Routing module initialized and started successfully.");

    if (!ota_module_init())
    {
        ESP_LOGE(TAG, "OTA module initialization failed.");
        return false;
    }
    ESP_LOGI(TAG, "OTA module initialized successfully.");

    if (!ota_module_register_callback(ota_event_handler))
    {
        ESP_LOGE(TAG, "Failed to register OTA callback.");
        return false;
    }
    ESP_LOGI(TAG, "OTA callback registered successfully.");

    /* Carrega parâmetros MONITOR_ do arquivo "config.ini" */
    load_retry_config();
    load_monitor_parameters();

    if (xTaskCreate(ota_task, "OTA_Task", OTA_TASK_STACK_SIZE, NULL, OTA_TASK_PRIORITY, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create OTA task.");
        return false;
    }
    ESP_LOGI(TAG, "OTA task created successfully.");

    if (xTaskCreate(communication_task, "Comm_Task", COMM_TASK_STACK_SIZE, NULL, COMM_TASK_PRIORITY, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create Communication task.");
        return false;
    }
    ESP_LOGI(TAG, "Communication task created successfully.");

    if (xTaskCreate(config_update_task, "Config_Task", CONFIG_TASK_STACK_SIZE, NULL, CONFIG_TASK_PRIORITY, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create Configuration Update task.");
        return false;
    }
    ESP_LOGI(TAG, "Configuration Update task created successfully.");

    if (xTaskCreate(can_acquisition_task, "CAN_Acq_Task", CAN_ACQ_TASK_STACK_SIZE, NULL, CAN_ACQ_TASK_PRIORITY, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create CAN Acquisition task.");
        return false;
    }
    ESP_LOGI(TAG, "CAN Acquisition task created successfully.");

    if (xTaskCreate(diagnosis_acquisition_task, "Diag_Acq_Task", DIAG_ACQ_TASK_STACK_SIZE, NULL, DIAG_ACQ_TASK_PRIORITY, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create Diagnosis Acquisition task.");
        return false;
    }
    ESP_LOGI(TAG, "Diagnosis Acquisition task created successfully.");

    ESP_LOGI(TAG, "Monitor ECU initialized successfully.");
    return true;
}
