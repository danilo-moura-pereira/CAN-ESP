#include "can_esp_lib.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>

static const char *TAG = "SELF_TEST";

/* Tarefa de recepção: processa mensagens continuamente */
static void receive_task(void *arg)
{
    while (1) {
        CAN_ESP_ProcessReceivedMessages();
        vTaskDelay(pdMS_TO_TICKS(10)); // Polling a cada 10 ms
    }
}

/* Tarefa de transmissão: envia várias mensagens com códigos distintos */
static void transmit_task(void *arg)
{
    /* Tabela de comandos conforme a definição do projeto */
    static const uint16_t message_commands[] = {
        0x001, 0x002, 0x003, 0x004,  // Controle do Motor Elétrico
        0x101, 0x102,                // Controle da Aceleração
        0x201, 0x202,                // Controle do Freio
        0x301, 0x302, 0x303,         // Controle da Direção
        0x401, 0x402, 0x403,         // Monitoramento da Bateria
        0x501, 0x502,                // Controle da Velocidade do Veículo
        0x601, 0x602, 0x603          // Diagnóstico (via OBD-II)
    };
    size_t num_msgs = sizeof(message_commands) / sizeof(message_commands[0]);

    for (size_t i = 0; i < num_msgs; i++) {
        /* Para este teste, usamos prioridade 1, módulo 1 e o comando conforme tabela */
        uint32_t id = CAN_ESP_EncodeID(1, 1, message_commands[i]);
        uint8_t data[CAN_MAX_DATA_LENGTH] = { (uint8_t)i, 0xAA, 0xBB, 0xCC, 0, 0, 0, 0 };

        if (CAN_ESP_SendMessage(id, data, 4) != CAN_ESP_OK) {
            ESP_LOGE(TAG, "Erro ao enviar mensagem para comando 0x%03X", message_commands[i]);
        } else {
            ESP_LOGI(TAG, "Mensagem enviada para comando 0x%03X", message_commands[i]);
        }
        vTaskDelay(pdMS_TO_TICKS(200));  // Aguarda 200 ms entre envios
    }
    vTaskDelete(NULL);
}

/* Callback para mensagens recebidas */
static void can_rx_callback(const CanEspMessage_t *msg)
{
    ESP_LOGI(TAG, "Callback: Mensagem recebida com ID: 0x%" PRIx32 ", Length: %u", msg->id, msg->length);
}

void app_main(void)
{
    /* Configuração para self test:
       - Bitrate: 25 Kbps (usando temporização customizada)
       - TX e RX conforme macros
       - Timeouts: valores padrão (1000 ms)
       - Filtro: aceita todas as mensagens
       - Modo: TWAI_MODE_NO_ACK (como no exemplo oficial do ESP-IDF)
       - self_rx: true (habilita auto-recepção)
       - Debug: nível 2 (detalhado)
    */
    CanEspConfig_t config = {
        .bitrate = 25000U,
        .tx_gpio = CAN_TX_GPIO,
        .rx_gpio = CAN_RX_GPIO,
        .transmit_timeout_ms = CAN_DEFAULT_TRANSMIT_TIMEOUT_MS,
        .receive_timeout_ms  = CAN_DEFAULT_RECEIVE_TIMEOUT_MS,
        .filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(),
        .mode = TWAI_MODE_NO_ACK,
        .use_custom_timing = true,
        .custom_timing_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_25KBITS(),
        .auto_retransmit = true,
        .debug_level = 2U,
        .self_rx = true
    };

    if (CAN_ESP_InitWithConfig(&config) != CAN_ESP_OK) {
        ESP_LOGE(TAG, "Erro ao inicializar o barramento CAN em modo NO_ACK.");
        return;
    }
    ESP_LOGI(TAG, "Barramento CAN iniciado para self test.");

    if (CAN_ESP_RegisterReceiveCallback(can_rx_callback) != CAN_ESP_OK) {
        ESP_LOGE(TAG, "Erro ao registrar callback de recepção.");
    }

    /* Cria as tarefas de transmissão e recepção */
    xTaskCreate(receive_task, "receive_task", 4096, NULL, 5, NULL);
    xTaskCreate(transmit_task, "transmit_task", 4096, NULL, 5, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
