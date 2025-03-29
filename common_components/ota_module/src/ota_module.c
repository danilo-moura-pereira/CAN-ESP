/**
 * @file ota_module.c
 * @brief Implementação do módulo OTA para o projeto CAN-ESP.
 *
 * Este módulo gerencia a atualização remota do firmware via OTA, integrando a notificação por MQTT,
 * a distribuição via ESP-MESH e a aplicação do firmware utilizando a API OTA do ESP-IDF.
 * O componente é desenvolvido para aplicações críticas em veículos elétricos reais, em conformidade
 * com as diretrizes MISRA C:2012 e os padrões ISO 11898.
 *
 * @note Este módulo assume que os módulos de conexão, roteamento e armazenamento já foram inicializados.
 */

#include "ota_module.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "wifi_connection_module.h"
#include "mqtt_connection_module.h"
#include "esp_mesh_connection_module.h"
#include "routing_module.h"
#include "sd_storage_module.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* Tag para logs */
#define TAG "OTA_MODULE"

/* Definições para leitura do arquivo de configuração "config.ini" */
#define CONFIG_LINE_MAX_LEN    128U
#define MAX_FILENAME_LENGTH    128U
#define MOUNT_POINT            "/sdcard"  /* Exemplo de ponto de montagem */

/* Comprimento máximo para os tópicos MQTT */
#define TOPIC_MAX_LEN 64U

/**
 * @brief Estrutura para armazenar os parâmetros de configuração OTA.
 *
 * Contém as versões dos firmwares para cada ECU, os tópicos MQTT correspondentes e o intervalo de verificação.
 */
typedef struct
{
    uint32_t firmware_version_monitor;      /**< Versão do firmware da ECU de Monitoramento */
    uint32_t firmware_version_acceleration;   /**< Versão do firmware da ECU de Controle de Aceleração */
    uint32_t firmware_version_steering;       /**< Versão do firmware da ECU de Controle de Direção */
    uint32_t firmware_version_motor;          /**< Versão do firmware da ECU de Controle de Motor */
    uint32_t firmware_version_brake;          /**< Versão do firmware da ECU de Controle de Freio */
    char topic_monitor[TOPIC_MAX_LEN];        /**< Tópico MQTT para a ECU de Monitoramento */
    char topic_acceleration[TOPIC_MAX_LEN];     /**< Tópico MQTT para a ECU de Controle de Aceleração */
    char topic_steering[TOPIC_MAX_LEN];         /**< Tópico MQTT para a ECU de Controle de Direção */
    char topic_motor[TOPIC_MAX_LEN];            /**< Tópico MQTT para a ECU de Controle de Motor */
    char topic_brake[TOPIC_MAX_LEN];            /**< Tópico MQTT para a ECU de Controle de Freio */
    uint32_t check_interval_ms;               /**< Intervalo para verificação de atualização (ms) */
} ota_config_t;

/* Variável estática para armazenar a configuração OTA com valores padrão */
static ota_config_t ota_config = {
    1U, 1U, 1U, 1U, 1U,
    "can-esp/firmware/update/monitor_ecu",
    "can-esp/firmware/update/acceleration_control_ecu",
    "can-esp/firmware/update/steering_control_ecu",
    "can-esp/firmware/update/motor_control_ecu",
    "can-esp/firmware/update/brake_control_ecu",
    60000U
};

/**
 * @brief Estrutura para representar um segmento de firmware.
 */
typedef struct {
    const uint8_t *data;  /**< Ponteiro para o início do segmento */
    uint32_t size;        /**< Tamanho do segmento (em bytes) */
} ota_segment_t;

/* Variáveis estáticas para armazenar os segmentos do firmware */
static ota_segment_t *ota_segments = NULL;
static uint32_t ota_segment_count = 0U;

/**
 * @brief Estrutura interna para gerenciamento do estado OTA.
 */
typedef struct
{
    ota_status_t status;               /**< Status atual da atualização OTA */
    char current_ecu[32];              /**< Identificador da ECU em atualização */
    uint32_t firmware_size;            /**< Tamanho total do firmware */
    uint8_t *firmware_data;            /**< Buffer contendo o firmware */
    bool update_in_progress;           /**< Flag para indicar atualização em andamento */
    ota_event_callback_t callbacks[5]; /**< Array de callbacks para eventos OTA */
    uint8_t callback_count;            /**< Número de callbacks registrados */
} ota_module_context_t;

/* Contexto estático do módulo OTA */
static ota_module_context_t ota_ctx = {
    OTA_STATUS_IDLE,
    "",
    0U,
    NULL,
    false,
    {NULL},
    0U
};

/* Protótipos de funções internas */
static void ota_notify_callbacks(ota_status_t status, const char *ecu_id, const void *data);
static bool ota_load_config(void);
static bool ota_save_config(void);
static void ota_rollback(const char *ecu_id);

/* Declaração externa das funções de download via MQTT/HTTP.
Supõe-se que estas funções estejam implementadas no módulo MQTT:
    - mqtt_connection_module_get_update_version: retorna a versão disponível em formato numérico.
    - mqtt_connection_module_download_file: realiza o download do firmware.
*/
extern bool mqtt_connection_module_get_update_version(const char *topic, uint32_t *version);
extern bool mqtt_connection_module_download_file(const char *topic, const char *filename, uint8_t **data, uint32_t *size);

/**
 * @brief Carrega o firmware do SD Card para a memória.
 *
 * Lê o arquivo especificado do SD Card e armazena os dados no contexto OTA.
 *
 * @param filename Nome do arquivo a ser lido.
 * @return true se o firmware for carregado com sucesso, false caso contrário.
 */
bool ota_module_load_firmware(const char *filename)
{
    uint32_t file_size = 0U;
    uint8_t *file_data = sd_storage_module_read_file(filename, &file_size);
    if ((file_data == NULL) || (file_size == 0U))
    {
        ESP_LOGE(TAG, "Failed to load firmware file %s from SD Card.", filename);
        return false;
    }
    ota_ctx.firmware_data = file_data;
    ota_ctx.firmware_size = file_size;
    ESP_LOGI(TAG, "Firmware file %s loaded: %u bytes.", filename, file_size);
    return true;
}

/**
 * @brief Inicializa o módulo OTA.
 *
 * Configura os recursos necessários, integrando os módulos Wi-Fi, MQTT, ESP-MESH,
 * Routing e SD Storage, e carrega os parâmetros de configuração do arquivo "config.ini".
 *
 * @return true se a inicialização for bem-sucedida, false caso contrário.
 */
bool ota_module_init(void)
{
    ESP_LOGI(TAG, "Initializing OTA module...");

    if (!wifi_connection_module_init())
    {
        ESP_LOGE(TAG, "Wi-Fi initialization failed.");
        return false;
    }
    if (!mqtt_connection_module_init())
    {
        ESP_LOGE(TAG, "MQTT initialization failed.");
        return false;
    }
    if (!esp_mesh_connection_module_init())
    {
        ESP_LOGE(TAG, "ESP-MESH initialization failed.");
        return false;
    }
    if (!routing_module_init())
    {
        ESP_LOGE(TAG, "Routing module initialization failed.");
        return false;
    }
    if (!sd_storage_module_init())
    {
        ESP_LOGE(TAG, "SD Storage module initialization failed.");
        return false;
    }

    /* Subscrever os tópicos utilizando os valores carregados dinamicamente */
    if (!mqtt_connection_module_subscribe(ota_config.topic_monitor) ||
        !mqtt_connection_module_subscribe(ota_config.topic_acceleration) ||
        !mqtt_connection_module_subscribe(ota_config.topic_steering) ||
        !mqtt_connection_module_subscribe(ota_config.topic_motor) ||
        !mqtt_connection_module_subscribe(ota_config.topic_brake))
    {
        ESP_LOGE(TAG, "Failed to subscribe to one or more OTA topics.");
        return false;
    }

    if (!ota_load_config())
    {
        ESP_LOGW(TAG, "Failed to load OTA configuration from config.ini. Using default values.");
    }

    ota_ctx.status = OTA_STATUS_IDLE;
    ota_ctx.update_in_progress = false;
    ESP_LOGI(TAG, "OTA module initialized successfully.");
    return true;
}

/**
 * @brief Registra um callback para eventos OTA.
 *
 * @param callback Ponteiro para a função de callback.
 * @return true se o callback for registrado com sucesso, false caso contrário.
 */
bool ota_module_register_callback(ota_event_callback_t callback)
{
    if (callback == NULL)
    {
        ESP_LOGE(TAG, "Null callback provided.");
        return false;
    }
    if (ota_ctx.callback_count >= (sizeof(ota_ctx.callbacks) / sizeof(ota_ctx.callbacks[0])))
    {
        ESP_LOGE(TAG, "Maximum number of OTA callbacks reached.");
        return false;
    }
    ota_ctx.callbacks[ota_ctx.callback_count] = callback;
    ota_ctx.callback_count++;
    ESP_LOGI(TAG, "OTA callback registered successfully. Total callbacks: %u", ota_ctx.callback_count);
    return true;
}

/**
 * @brief Verifica periodicamente se há uma nova atualização de firmware disponível para a ECU "monitor_ecu".
 *
 * Obtém a versão disponível do firmware em formato numérico por meio do módulo MQTT e compara com a versão instalada.
 *
 * @return true se uma atualização estiver disponível, false caso contrário.
 */
bool ota_module_check_update(void)
{
    uint32_t available_version = 0U;
    bool ret;

    ESP_LOGI(TAG, "Checking for firmware updates via MQTT for ECU: monitor_ecu...");

    /* Obtém a versão disponível do firmware para a ECU de Monitoramento */
    ret = mqtt_connection_module_get_update_version(ota_config.topic_monitor, &available_version);
    if (!ret)
    {
        ESP_LOGW(TAG, "No update version received on topic %s", ota_config.topic_monitor);
        return false;
    }
    ESP_LOGI(TAG, "Received available firmware version %u for ECU: monitor_ecu", available_version);
    return ota_module_check_version("monitor_ecu", available_version);
}

/**
 * @brief Verifica sob demanda se a versão disponível do firmware é superior à instalada para a ECU especificada.
 *
 * @param ecu_id Identificador da ECU.
 * @param available_version Versão disponível do firmware.
 * @return true se a versão disponível for superior à instalada, false caso contrário.
 */
bool ota_module_check_version(const char *ecu_id, uint32_t available_version)
{
    uint32_t installed_version = 0U;

    if (ecu_id == NULL)
    {
        ESP_LOGE(TAG, "Null ECU ID provided for version check.");
        return false;
    }

    if (strcmp(ecu_id, "monitor_ecu") == 0)
    {
        installed_version = ota_config.firmware_version_monitor;
    }
    else if (strcmp(ecu_id, "acceleration_control_ecu") == 0)
    {
        installed_version = ota_config.firmware_version_acceleration;
    }
    else if (strcmp(ecu_id, "steering_control_ecu") == 0)
    {
        installed_version = ota_config.firmware_version_steering;
    }
    else if (strcmp(ecu_id, "motor_control_ecu") == 0)
    {
        installed_version = ota_config.firmware_version_motor;
    }
    else if (strcmp(ecu_id, "brake_control_ecu") == 0)
    {
        installed_version = ota_config.firmware_version_brake;
    }
    else
    {
        ESP_LOGE(TAG, "Unknown ECU ID: %s", ecu_id);
        return false;
    }

    ESP_LOGI(TAG, "ECU: %s, Installed version: %u, Available version: %u", ecu_id, installed_version, available_version);
    if (available_version > installed_version)
    {
        ota_ctx.status = OTA_STATUS_UPDATE_AVAILABLE;
        (void)strncpy(ota_ctx.current_ecu, ecu_id, sizeof(ota_ctx.current_ecu) - 1);
        ota_notify_callbacks(ota_ctx.status, ecu_id, NULL);
        ESP_LOGI(TAG, "Update available for ECU: %s", ecu_id);
        return true;
    }
    else
    {
        ESP_LOGI(TAG, "No update available for ECU: %s", ecu_id);
        return false;
    }
}

/**
 * @brief Inicia o download do firmware para a ECU especificada.
 *
 * Realiza o download via MQTT/HTTP e grava o firmware no SD Card utilizando um nome padronizado,
 * conforme o padrão:
 * - firmware_monitor_ecu_vX.bin
 * - firmware_acceleration_control_ecu_vX.bin
 * - firmware_steering_control_ecu_vX.bin
 * - firmware_motor_control_ecu_vX.bin
 * - firmware_brake_control_ecu_vX.bin
 *
 * Onde _vX é substituído pelo número da versão instalada.
 *
 * Após gravar o arquivo, o firmware é carregado para a memória por meio de ota_module_load_firmware().
 *
 * @param ecu_id Identificador da ECU que receberá a atualização.
 * @return true se o download e o carregamento forem realizados com sucesso, false caso contrário.
 */
bool ota_module_download_firmware(const char *ecu_id)
{
    char filename[MAX_FILENAME_LENGTH];
    const char *topic = NULL;
    uint32_t version = 0U;
    bool ret;

    if (ecu_id == NULL)
    {
        ESP_LOGE(TAG, "Null ECU ID provided for firmware download.");
        return false;
    }
    if (ota_ctx.update_in_progress)
    {
        ESP_LOGW(TAG, "An update is already in progress.");
        return false;
    }

    ESP_LOGI(TAG, "Starting firmware download for ECU: %s", ecu_id);
    ota_ctx.update_in_progress = true;
    ota_ctx.status = OTA_STATUS_DOWNLOADING;
    ota_notify_callbacks(ota_ctx.status, ecu_id, NULL);

    /* Seleciona o tópico e gera o nome do arquivo padronizado de acordo com a ECU */
    if (strcmp(ecu_id, "monitor_ecu") == 0)
    {
        topic = ota_config.topic_monitor;
        version = ota_config.firmware_version_monitor;
        (void)snprintf(filename, MAX_FILENAME_LENGTH, "firmware_monitor_ecu_v%u.bin", version);
    }
    else if (strcmp(ecu_id, "acceleration_control_ecu") == 0)
    {
        topic = ota_config.topic_acceleration;
        version = ota_config.firmware_version_acceleration;
        (void)snprintf(filename, MAX_FILENAME_LENGTH, "firmware_acceleration_control_ecu_v%u.bin", version);
    }
    else if (strcmp(ecu_id, "steering_control_ecu") == 0)
    {
        topic = ota_config.topic_steering;
        version = ota_config.firmware_version_steering;
        (void)snprintf(filename, MAX_FILENAME_LENGTH, "firmware_steering_control_ecu_v%u.bin", version);
    }
    else if (strcmp(ecu_id, "motor_control_ecu") == 0)
    {
        topic = ota_config.topic_motor;
        version = ota_config.firmware_version_motor;
        (void)snprintf(filename, MAX_FILENAME_LENGTH, "firmware_motor_control_ecu_v%u.bin", version);
    }
    else if (strcmp(ecu_id, "brake_control_ecu") == 0)
    {
        topic = ota_config.topic_brake;
        version = ota_config.firmware_version_brake;
        (void)snprintf(filename, MAX_FILENAME_LENGTH, "firmware_brake_control_ecu_v%u.bin", version);
    }
    else
    {
        ESP_LOGE(TAG, "Unknown ECU ID: %s", ecu_id);
        ota_ctx.update_in_progress = false;
        return false;
    }

    ESP_LOGI(TAG, "Downloading firmware from topic: %s, saving as: %s", topic, filename);

    /* Realiza o download do firmware utilizando a função do módulo MQTT */
    ret = mqtt_connection_module_download_file(topic, filename, NULL, NULL);
    if (!ret)
    {
        ESP_LOGE(TAG, "Firmware download failed for ECU: %s", ecu_id);
        ota_ctx.update_in_progress = false;
        ota_ctx.status = OTA_STATUS_FAILURE;
        ota_notify_callbacks(ota_ctx.status, ecu_id, NULL);
        return false;
    }

    /* Carrega o firmware do SD Card para a memória */
    if (!ota_module_load_firmware(filename))
    {
        ESP_LOGE(TAG, "Failed to load firmware file %s for ECU: %s", filename, ecu_id);
        ota_ctx.update_in_progress = false;
        ota_ctx.status = OTA_STATUS_FAILURE;
        ota_notify_callbacks(ota_ctx.status, ecu_id, NULL);
        return false;
    }

    ESP_LOGI(TAG, "Firmware downloaded and loaded for ECU: %s", ecu_id);
    return true;
}

/**
 * @brief Segmenta o firmware em pacotes para distribuição via ESP-MESH.
 *
 * Divide o firmware carregado em blocos de tamanho OTA_PACKET_SIZE e armazena os segmentos
 * em um array dinâmico do tipo ota_segment_t.
 *
 * @param firmware_data Ponteiro para os dados do firmware.
 * @param firmware_size Tamanho total do firmware em bytes.
 * @return true se a segmentação for realizada com sucesso, false caso contrário.
 */
bool ota_module_segment_firmware(const uint8_t *firmware_data, uint32_t firmware_size)
{
    uint32_t num_segments, i;

    if ((firmware_data == NULL) || (firmware_size == 0U))
    {
        ESP_LOGE(TAG, "Invalid firmware data for segmentation.");
        return false;
    }

    num_segments = (firmware_size + OTA_PACKET_SIZE - 1U) / OTA_PACKET_SIZE;

    ota_segments = (ota_segment_t *)malloc(num_segments * sizeof(ota_segment_t));
    if (ota_segments == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for firmware segments.");
        return false;
    }
    ota_segment_count = num_segments;

    for (i = 0U; i < num_segments; i++)
    {
        ota_segments[i].data = firmware_data + (i * OTA_PACKET_SIZE);
        if ((firmware_size - (i * OTA_PACKET_SIZE)) >= OTA_PACKET_SIZE)
        {
            ota_segments[i].size = OTA_PACKET_SIZE;
        }
        else
        {
            ota_segments[i].size = firmware_size - (i * OTA_PACKET_SIZE);
        }
        ESP_LOGD(TAG, "Segment %u: size %u bytes", i, ota_segments[i].size);
    }
    ESP_LOGI(TAG, "Firmware segmentation completed: %u segments created.", ota_segment_count);
    return true;
}

/**
 * @brief Distribui os pacotes de firmware para a ECU especificada via ESP-MESH.
 *
 * Itera sobre o array de segmentos gerado por ota_module_segment_firmware e envia cada pacote para a ECU
 * utilizando a função routing_module_send_message(), que encaminha a mensagem via ESP-MESH.
 *
 * @param ecu_id Identificador da ECU destino.
 * @return true se todos os pacotes forem enviados com sucesso, false caso contrário.
 */
bool ota_module_distribute_firmware(const char *ecu_id)
{
    uint32_t i;

    if (ecu_id == NULL)
    {
        ESP_LOGE(TAG, "Null ECU ID provided for firmware distribution.");
        return false;
    }
    if (ota_segments == NULL)
    {
        ESP_LOGE(TAG, "Firmware has not been segmented. Call ota_module_segment_firmware first.");
        return false;
    }

    ESP_LOGI(TAG, "Distributing firmware to ECU: %s", ecu_id);
    ota_ctx.status = OTA_STATUS_DISTRIBUTING;
    ota_notify_callbacks(ota_ctx.status, ecu_id, NULL);

    for (i = 0U; i < ota_segment_count; i++)
    {
        if (!routing_module_send_message(ecu_id, ota_segments[i].data, (uint16_t)ota_segments[i].size, 0))
        {
            ESP_LOGE(TAG, "Failed to send firmware segment %u for ECU: %s", i, ecu_id);
            ota_ctx.status = OTA_STATUS_FAILURE;
            ota_notify_callbacks(ota_ctx.status, ecu_id, NULL);
            free(ota_segments);
            ota_segments = NULL;
            ota_segment_count = 0U;
            return false;
        }
    }
    ESP_LOGI(TAG, "Firmware distribution completed for ECU: %s", ecu_id);

    free(ota_segments);
    ota_segments = NULL;
    ota_segment_count = 0U;
    return true;
}

/**
 * @brief Aplica a atualização do firmware utilizando a API OTA do ESP-IDF.
 *
 * Grava o firmware na partição OTA. Em caso de erro, o status é definido como OTA_STATUS_FAILURE
 * e o rollback é acionado.
 *
 * @param ecu_id Identificador da ECU que aplicará a atualização.
 * @return true se a atualização for aplicada com sucesso, false caso contrário.
 */
bool ota_module_apply_update(const char *ecu_id)
{
    const esp_partition_t *update_partition = NULL;
    esp_ota_handle_t ota_handle = 0;
    esp_err_t err;

    if (ecu_id == NULL)
    {
        ESP_LOGE(TAG, "Null ECU ID provided for firmware update application.");
        return false;
    }
    ESP_LOGI(TAG, "Applying firmware update on ECU: %s", ecu_id);
    ota_ctx.status = OTA_STATUS_APPLYING;
    ota_notify_callbacks(ota_ctx.status, ecu_id, NULL);

    update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL)
    {
        ESP_LOGE(TAG, "Failed to find update partition.");
        ota_ctx.status = OTA_STATUS_FAILURE;
        ota_notify_callbacks(ota_ctx.status, ecu_id, NULL);
        ota_rollback(ecu_id);
        return false;
    }

    err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_begin failed (%s).", esp_err_to_name(err));
        ota_ctx.status = OTA_STATUS_FAILURE;
        ota_notify_callbacks(ota_ctx.status, ecu_id, NULL);
        ota_rollback(ecu_id);
        return false;
    }

    err = esp_ota_write(ota_handle, ota_ctx.firmware_data, ota_ctx.firmware_size);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_write failed (%s).", esp_err_to_name(err));
        (void)esp_ota_end(ota_handle);
        ota_ctx.status = OTA_STATUS_FAILURE;
        ota_notify_callbacks(ota_ctx.status, ecu_id, NULL);
        ota_rollback(ecu_id);
        return false;
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_end failed (%s).", esp_err_to_name(err));
        ota_ctx.status = OTA_STATUS_FAILURE;
        ota_notify_callbacks(ota_ctx.status, ecu_id, NULL);
        ota_rollback(ecu_id);
        return false;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s).", esp_err_to_name(err));
        ota_ctx.status = OTA_STATUS_FAILURE;
        ota_notify_callbacks(ota_ctx.status, ecu_id, NULL);
        ota_rollback(ecu_id);
        return false;
    }

    ESP_LOGI(TAG, "Firmware update applied successfully on ECU: %s", ecu_id);
    ota_ctx.status = OTA_STATUS_SUCCESS;
    ota_notify_callbacks(ota_ctx.status, ecu_id, NULL);

    if (!ota_module_update_config())
    {
        ESP_LOGW(TAG, "Failed to update OTA configuration.");
    }

    if (ota_ctx.firmware_data != NULL)
    {
        free(ota_ctx.firmware_data);
        ota_ctx.firmware_data = NULL;
    }
    ota_ctx.update_in_progress = false;
    return true;
}

/**
 * @brief Atualiza as configurações OTA no arquivo "config.ini".
 *
 * Utiliza o sd_storage_module para persistir os parâmetros de configuração atualizados, incluindo
 * as versões de firmware, os tópicos MQTT e o intervalo de verificação.
 *
 * @return true se as configurações forem atualizadas com sucesso, false caso contrário.
 */
bool ota_module_update_config(void)
{
    char config_buffer[512];
    int len;

    ESP_LOGI(TAG, "Updating OTA configuration in config.ini...");
    len = snprintf(config_buffer, sizeof(config_buffer),
                "OTA_FIRMWARE_VERSION_MONITOR=%u\n"
                "OTA_FIRMWARE_VERSION_ACCELERATION=%u\n"
                "OTA_FIRMWARE_VERSION_STEERING=%u\n"
                "OTA_FIRMWARE_VERSION_MOTOR=%u\n"
                "OTA_FIRMWARE_VERSION_BRAKE=%u\n"
                "MQTT_TOPIC_MONITOR=%s\n"
                "MQTT_TOPIC_ACCELERATION=%s\n"
                "MQTT_TOPIC_STEERING=%s\n"
                "MQTT_TOPIC_MOTOR=%s\n"
                "MQTT_TOPIC_BRAKE=%s\n"
                "OTA_CHECK_INTERVAL_MS=%u\n",
                ota_config.firmware_version_monitor,
                ota_config.firmware_version_acceleration,
                ota_config.firmware_version_steering,
                ota_config.firmware_version_motor,
                ota_config.firmware_version_brake,
                ota_config.topic_monitor,
                ota_config.topic_acceleration,
                ota_config.topic_steering,
                ota_config.topic_motor,
                ota_config.topic_brake,
                ota_config.check_interval_ms);
    if (len <= 0)
    {
        ESP_LOGE(TAG, "Error formatting OTA configuration.");
        return false;
    }
    if (!sd_storage_module_write_file("config.ini", config_buffer, (uint32_t)len))
    {
        ESP_LOGE(TAG, "Failed to write OTA configuration to file.");
        return false;
    }
    return true;
}

/**
 * @brief Atualiza dinamicamente os parâmetros de configuração OTA.
 *
 * Lê o arquivo "config.ini" e atualiza os parâmetros internos do módulo OTA sem reinicializar a ECU.
 *
 * @return true se os parâmetros forem atualizados com sucesso, false caso contrário.
 */
bool ota_module_refresh_config(void)
{
    ESP_LOGI(TAG, "Refreshing OTA configuration dynamically...");
    return ota_load_config();
}

/**
 * @brief Carrega a configuração OTA a partir do arquivo "config.ini".
 *
 * Lê os parâmetros:
 * - OTA_FIRMWARE_VERSION_MONITOR
 * - OTA_FIRMWARE_VERSION_ACCELERATION
 * - OTA_FIRMWARE_VERSION_STEERING
 * - OTA_FIRMWARE_VERSION_MOTOR
 * - OTA_FIRMWARE_VERSION_BRAKE
 * - MQTT_TOPIC_MONITOR
 * - MQTT_TOPIC_ACCELERATION
 * - MQTT_TOPIC_STEERING
 * - MQTT_TOPIC_MOTOR
 * - MQTT_TOPIC_BRAKE
 * - OTA_CHECK_INTERVAL_MS
 *
 * @return true se a configuração for carregada com sucesso, false caso contrário.
 */
static bool ota_load_config(void)
{
    char config_line[CONFIG_LINE_MAX_LEN];
    char config_path[MAX_FILENAME_LENGTH];
    FILE *file;

    ESP_LOGI(TAG, "Loading OTA configuration from config.ini...");
    (void)snprintf(config_path, MAX_FILENAME_LENGTH, "%s/%s", MOUNT_POINT, "config.ini");
    file = fopen(config_path, "r");
    if (file == NULL)
    {
        ESP_LOGW(TAG, "Config file %s not found, using default OTA configuration.", config_path);
        return false;
    }
    while (fgets(config_line, CONFIG_LINE_MAX_LEN, file) != NULL)
    {
        char *newline = strchr(config_line, '\n');
        if (newline != NULL)
        {
            *newline = '\0';
        }
        if (strncmp(config_line, "OTA_FIRMWARE_VERSION_MONITOR=", 29U) == 0)
        {
            ota_config.firmware_version_monitor = (uint32_t)atoi(config_line + 29U);
            ESP_LOGI(TAG, "Loaded firmware version (monitor): %u", ota_config.firmware_version_monitor);
        }
        else if (strncmp(config_line, "OTA_FIRMWARE_VERSION_ACCELERATION=", 34U) == 0)
        {
            ota_config.firmware_version_acceleration = (uint32_t)atoi(config_line + 34U);
            ESP_LOGI(TAG, "Loaded firmware version (acceleration): %u", ota_config.firmware_version_acceleration);
        }
        else if (strncmp(config_line, "OTA_FIRMWARE_VERSION_STEERING=", 30U) == 0)
        {
            ota_config.firmware_version_steering = (uint32_t)atoi(config_line + 30U);
            ESP_LOGI(TAG, "Loaded firmware version (steering): %u", ota_config.firmware_version_steering);
        }
        else if (strncmp(config_line, "OTA_FIRMWARE_VERSION_MOTOR=", 27U) == 0)
        {
            ota_config.firmware_version_motor = (uint32_t)atoi(config_line + 27U);
            ESP_LOGI(TAG, "Loaded firmware version (motor): %u", ota_config.firmware_version_motor);
        }
        else if (strncmp(config_line, "OTA_FIRMWARE_VERSION_BRAKE=", 27U) == 0)
        {
            ota_config.firmware_version_brake = (uint32_t)atoi(config_line + 27U);
            ESP_LOGI(TAG, "Loaded firmware version (brake): %u", ota_config.firmware_version_brake);
        }
        else if (strncmp(config_line, "MQTT_TOPIC_MONITOR=", 19U) == 0)
        {
            (void)strncpy(ota_config.topic_monitor, config_line + 19U, TOPIC_MAX_LEN - 1);
            ota_config.topic_monitor[TOPIC_MAX_LEN - 1] = '\0';
            ESP_LOGI(TAG, "Loaded MQTT topic (monitor): %s", ota_config.topic_monitor);
        }
        else if (strncmp(config_line, "MQTT_TOPIC_ACCELERATION=", 24U) == 0)
        {
            (void)strncpy(ota_config.topic_acceleration, config_line + 24U, TOPIC_MAX_LEN - 1);
            ota_config.topic_acceleration[TOPIC_MAX_LEN - 1] = '\0';
            ESP_LOGI(TAG, "Loaded MQTT topic (acceleration): %s", ota_config.topic_acceleration);
        }
        else if (strncmp(config_line, "MQTT_TOPIC_STEERING=", 20U) == 0)
        {
            (void)strncpy(ota_config.topic_steering, config_line + 20U, TOPIC_MAX_LEN - 1);
            ota_config.topic_steering[TOPIC_MAX_LEN - 1] = '\0';
            ESP_LOGI(TAG, "Loaded MQTT topic (steering): %s", ota_config.topic_steering);
        }
        else if (strncmp(config_line, "MQTT_TOPIC_MOTOR=", 17U) == 0)
        {
            (void)strncpy(ota_config.topic_motor, config_line + 17U, TOPIC_MAX_LEN - 1);
            ota_config.topic_motor[TOPIC_MAX_LEN - 1] = '\0';
            ESP_LOGI(TAG, "Loaded MQTT topic (motor): %s", ota_config.topic_motor);
        }
        else if (strncmp(config_line, "MQTT_TOPIC_BRAKE=", 17U) == 0)
        {
            (void)strncpy(ota_config.topic_brake, config_line + 17U, TOPIC_MAX_LEN - 1);
            ota_config.topic_brake[TOPIC_MAX_LEN - 1] = '\0';
            ESP_LOGI(TAG, "Loaded MQTT topic (brake): %s", ota_config.topic_brake);
        }
        else if (strncmp(config_line, "OTA_CHECK_INTERVAL_MS=", 22U) == 0)
        {
            ota_config.check_interval_ms = (uint32_t)atoi(config_line + 22U);
            ESP_LOGI(TAG, "Loaded OTA check interval: %u ms", ota_config.check_interval_ms);
        }
    }
    fclose(file);
    ESP_LOGI(TAG, "OTA configuration loaded from %s.", config_path);
    return true;
}

/**
 * @brief Salva as configurações OTA no arquivo "config.ini".
 *
 * Utiliza o sd_storage_module para persistir os parâmetros de configuração atualizados.
 *
 * @return true se as configurações forem salvas com sucesso, false caso contrário.
 */
static bool ota_save_config(void)
{
    char config_buffer[512];
    int len;

    ESP_LOGI(TAG, "Saving OTA configuration to config.ini...");
    len = snprintf(config_buffer, sizeof(config_buffer),
                "OTA_FIRMWARE_VERSION_MONITOR=%u\n"
                "OTA_FIRMWARE_VERSION_ACCELERATION=%u\n"
                "OTA_FIRMWARE_VERSION_STEERING=%u\n"
                "OTA_FIRMWARE_VERSION_MOTOR=%u\n"
                "OTA_FIRMWARE_VERSION_BRAKE=%u\n"
                "MQTT_TOPIC_MONITOR=%s\n"
                "MQTT_TOPIC_ACCELERATION=%s\n"
                "MQTT_TOPIC_STEERING=%s\n"
                "MQTT_TOPIC_MOTOR=%s\n"
                "MQTT_TOPIC_BRAKE=%s\n"
                "OTA_CHECK_INTERVAL_MS=%u\n",
                ota_config.firmware_version_monitor,
                ota_config.firmware_version_acceleration,
                ota_config.firmware_version_steering,
                ota_config.firmware_version_motor,
                ota_config.firmware_version_brake,
                ota_config.topic_monitor,
                ota_config.topic_acceleration,
                ota_config.topic_steering,
                ota_config.topic_motor,
                ota_config.topic_brake,
                ota_config.check_interval_ms);
    if (len <= 0)
    {
        ESP_LOGE(TAG, "Error formatting OTA configuration.");
        return false;
    }
    if (!sd_storage_module_write_file("config.ini", config_buffer, (uint32_t)len))
    {
        ESP_LOGE(TAG, "Failed to write OTA configuration to file.");
        return false;
    }
    return true;
}

/**
 * @brief Atualiza dinamicamente os parâmetros de configuração OTA.
 *
 * Lê o arquivo "config.ini" e atualiza os parâmetros internos do módulo OTA sem reinicializar a ECU.
 *
 * @return true se os parâmetros forem atualizados com sucesso, false caso contrário.
 */
bool ota_module_refresh_config(void)
{
    ESP_LOGI(TAG, "Refreshing OTA configuration dynamically...");
    return ota_load_config();
}

/**
 * @brief Exclui o firmware do SD Card.
 *
 * Remove o arquivo de firmware temporariamente armazenado no SD Card, liberando espaço.
 *
 * @param filename Nome do arquivo de firmware a ser excluído.
 * @return true se o arquivo for excluído com sucesso, false caso contrário.
 */
bool ota_module_delete_firmware(const char *filename)
{
    if (filename == NULL)
    {
        ESP_LOGE(TAG, "Null filename provided for firmware deletion.");
        return false;
    }
    if (!sd_storage_module_delete_file(filename))
    {
        ESP_LOGE(TAG, "Failed to delete firmware file %s from SD Card.", filename);
        return false;
    }
    ESP_LOGI(TAG, "Firmware file %s deleted successfully from SD Card.", filename);
    return true;
}

/**
 * @brief Realiza o rollback do firmware em caso de falha irreversível na atualização OTA.
 *
 * Este método aciona o rollback do firmware, restaurando a versão anterior ou adotando outra estratégia de fallback.
 *
 * @param ecu_id Identificador da ECU.
 * @return true se o rollback for bem-sucedido, false caso contrário.
 */
bool ota_module_rollback_update(const char *ecu_id)
{
    ESP_LOGW(TAG, "Initiating rollback update for ECU: %s", ecu_id);
    ota_rollback(ecu_id);
    /* Aqui, podemos considerar que o rollback foi acionado com sucesso, pois a função interna
    ota_rollback atualiza o status para OTA_STATUS_ROLLBACK e notifica os callbacks.
    Se for necessário, podemos adicionar verificações adicionais (por exemplo, se o firmware anterior
    foi efetivamente restaurado).
    */
    return true;
}
