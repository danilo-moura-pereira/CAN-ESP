/*
 * can_esp_lib.c
 * Implementação da Biblioteca CAN para ESP32 (ESP-IDF)
 * Projeto acadêmico para Mestrado Profissional em Engenharia Elétrica - UnB
 * Adaptado para conformidade com MISRA C:2012.
 * Nome da biblioteca: can_esp_lib
 */

#include "can_esp_lib.h"

#include "esp_log.h"
#include "driver/twai.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"  /* Para mutexes */

#include <string.h>
#include <inttypes.h>
#include <limits.h>
#include <stdlib.h>

#define TAG    "CAN_ESP_LIB"

/* Mutexes para proteção da configuração e das métricas de latência */
static SemaphoreHandle_t configMutex = NULL;
static SemaphoreHandle_t latencyMutex = NULL;

/* Fila de transmissão */
static QueueHandle_t txQueue = NULL;

/* Handle da tarefa de transmissão (para ajuste dinâmico de prioridade) */
static TaskHandle_t canTxTaskHandle = NULL;

/* Variáveis para medição de retransmissões, colisões e tentativas */
static uint32_t totalRetransmissions = 0;
static uint32_t totalCollisions = 0;
static uint32_t totalTransmissionAttempts = 0;

/* Callbacks */
static can_esp_receive_callback_t receive_callback = NULL;
static can_esp_transmit_callback_t transmit_callback = NULL;

/* Configuração padrão; self_rx e use_checksum desabilitados */
static CanEspConfig_t currentConfig = {
    .bitrate = 1000000U,
    .tx_gpio = CAN_TX_GPIO,
    .rx_gpio = CAN_RX_GPIO,
    .transmit_timeout_ms = CAN_DEFAULT_TRANSMIT_TIMEOUT_MS,
    .receive_timeout_ms = CAN_DEFAULT_RECEIVE_TIMEOUT_MS,
    .filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(),
    .mode = TWAI_MODE_NO_ACK,
    .use_custom_timing = false,
    .custom_timing_config = {0},
    .auto_retransmit = true,
    .debug_level = 2U,
    .self_rx = false,
    .use_checksum = false
};

static bool configInitialized = false;

/* Estrutura para armazenar métricas de latência */
static CanEspLatencyMetrics_t latencyMetrics = {0, 0, INT64_MAX, 0};

/* Variáveis para medição do bus load */
static int64_t busLoadTotalTime = 0;
static int64_t busLoadStartTime = 0;

/* Protótipo para função auxiliar de temporização */
static twai_timing_config_t GetTimingConfig(uint32_t bitrate);

/* Função auxiliar para obter configuração de temporização baseada no bitrate */
static twai_timing_config_t GetTimingConfig(uint32_t bitrate)
{
    if (bitrate == 1000000U) {
        static const twai_timing_config_t timing_config_1mbits = TWAI_TIMING_CONFIG_1MBITS();
        return timing_config_1mbits;
    } else if (bitrate == 500000U) {
        static const twai_timing_config_t timing_config_500kbits = TWAI_TIMING_CONFIG_500KBITS();
        return timing_config_500kbits;
    } else {
        static const twai_timing_config_t timing_config_1mbits = TWAI_TIMING_CONFIG_1MBITS();
        return timing_config_1mbits;
    }
}

/* Função auxiliar para converter CanEspMessage_t para twai_message_t */
static void convert_canesp_to_twai(const CanEspMessage_t *src, twai_message_t *dst)
{
    if (src == NULL || dst == NULL) {
        return;
    }
    dst->identifier = src->id;
    if (currentConfig.use_checksum && src->length < CAN_MAX_DATA_LENGTH) {
        dst->data_length_code = src->length + 1;
    } else {
        dst->data_length_code = src->length;
    }
    memcpy(dst->data, src->data, src->length);
    dst->extd = 1U;
    dst->rtr = 0;
    dst->ss = 0;
    dst->self = currentConfig.self_rx ? 1U : 0U;
}

/* Calcula um checksum simples (XOR de todos os bytes) */
uint8_t CAN_ESP_CalculateChecksum(const uint8_t *data, uint8_t length)
{
    uint8_t cs = 0;
    for (uint8_t i = 0U; i < length; i++) {
        cs ^= data[i];
    }
    return cs;
}

/*==============================================================================
                         FUNÇÕES DE CONFIGURAÇÃO DINÂMICA
 ==============================================================================*/

can_esp_status_t CAN_ESP_InitWithConfig(const CanEspConfig_t *config)
{
    twai_general_config_t generalConfig;
    twai_timing_config_t timingConfig;
    twai_filter_config_t filterConfig;

    if (config == NULL) {
        ESP_LOGE(TAG, "Ponteiro de configuração nulo.");
        return CAN_ESP_ERR_NULL_POINTER;
    }
    if (configMutex == NULL) {
        configMutex = xSemaphoreCreateMutex();
        if (configMutex == NULL) {
            ESP_LOGE(TAG, "Falha ao criar mutex de configuração.");
            return CAN_ESP_ERR_UNKNOWN;
        }
    }
    if (latencyMutex == NULL) {
        latencyMutex = xSemaphoreCreateMutex();
        if (latencyMutex == NULL) {
            ESP_LOGE(TAG, "Falha ao criar mutex de latência.");
            return CAN_ESP_ERR_UNKNOWN;
        }
    }
    xSemaphoreTake(configMutex, portMAX_DELAY);
    currentConfig = *config;
    configInitialized = true;
    xSemaphoreGive(configMutex);

    /* Inicializa a medição do bus load */
    busLoadStartTime = esp_timer_get_time();
    busLoadTotalTime = 0;

    generalConfig = (twai_general_config_t)TWAI_GENERAL_CONFIG_DEFAULT(currentConfig.tx_gpio, currentConfig.rx_gpio, currentConfig.mode);
    if (currentConfig.use_custom_timing) {
        timingConfig = currentConfig.custom_timing_config;
    } else {
        timingConfig = GetTimingConfig(currentConfig.bitrate);
    }
    filterConfig = currentConfig.filter_config;

    if (twai_driver_install(&generalConfig, &timingConfig, &filterConfig) != ESP_OK) {
        ESP_LOGE(TAG, "Falha na instalação do driver TWAI.");
        return CAN_ESP_ERR_DRIVER_INSTALL;
    }
    if (twai_start() != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao iniciar o barramento CAN.");
        return CAN_ESP_ERR_DRIVER_START;
    }
    ESP_LOGI(TAG, "Barramento CAN iniciado com configuração dinâmica.");

    if (txQueue == NULL) {
        txQueue = xQueueCreate(TX_QUEUE_LENGTH, sizeof(CanEspMessage_t));
        if (txQueue == NULL) {
            ESP_LOGE(TAG, "Falha ao criar a fila de transmissão.");
            return CAN_ESP_ERR_UNKNOWN;
        }
    }
    return CAN_ESP_OK;
}

can_esp_status_t CAN_ESP_Init(void)
{
    if (!configInitialized) {
        currentConfig.bitrate = 1000000U;
        currentConfig.tx_gpio = CAN_TX_GPIO;
        currentConfig.rx_gpio = CAN_RX_GPIO;
        currentConfig.transmit_timeout_ms = CAN_DEFAULT_TRANSMIT_TIMEOUT_MS;
        currentConfig.receive_timeout_ms = CAN_DEFAULT_RECEIVE_TIMEOUT_MS;
        currentConfig.use_checksum = false;  /* Padrão: checksum desabilitado */
        configInitialized = true;
    }
    return CAN_ESP_InitWithConfig(&currentConfig);
}

can_esp_status_t CAN_ESP_UpdateConfig(const CanEspConfig_t *config)
{
    can_esp_status_t status;
    if (config == NULL) {
        ESP_LOGE(TAG, "Ponteiro de configuração nulo na atualização.");
        return CAN_ESP_ERR_NULL_POINTER;
    }
    status = CAN_ESP_Deinit();
    if (status != CAN_ESP_OK) {
        return status;
    }
    xSemaphoreTake(configMutex, portMAX_DELAY);
    currentConfig = *config;
    xSemaphoreGive(configMutex);
    return CAN_ESP_InitWithConfig(config);
}

can_esp_status_t CAN_ESP_Deinit(void)
{
    if (twai_stop() != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao parar o barramento CAN.");
        return CAN_ESP_ERR_DRIVER_STOP;
    }
    if (twai_driver_uninstall() != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao desinstalar o driver TWAI.");
        return CAN_ESP_ERR_DRIVER_UNINSTALL;
    }
    ESP_LOGI(TAG, "Barramento CAN desinicializado com sucesso.");
    return CAN_ESP_OK;
}

/*==============================================================================
                   FUNÇÕES DE ATUALIZAÇÃO PARCIAL
 ==============================================================================*/

can_esp_status_t CAN_ESP_SetFilterConfig(const twai_filter_config_t *new_filter_config)
{
    if (new_filter_config == NULL) {
        ESP_LOGE(TAG, "Ponteiro de nova configuração de filtro nulo.");
        return CAN_ESP_ERR_NULL_POINTER;
    }
    xSemaphoreTake(configMutex, portMAX_DELAY);
    currentConfig.filter_config = *new_filter_config;
    xSemaphoreGive(configMutex);
    ESP_LOGI(TAG, "Nova configuração de filtro atualizada. Reinicializando driver...");
    return CAN_ESP_UpdateConfig(&currentConfig);
}

can_esp_status_t CAN_ESP_SetTimeouts(uint32_t tx_timeout_ms, uint32_t rx_timeout_ms)
{
    xSemaphoreTake(configMutex, portMAX_DELAY);
    currentConfig.transmit_timeout_ms = tx_timeout_ms;
    currentConfig.receive_timeout_ms  = rx_timeout_ms;
    xSemaphoreGive(configMutex);
    ESP_LOGI(TAG, "Timeouts atualizados: Tx = %" PRIu32 " ms, Rx = %" PRIu32 " ms", tx_timeout_ms, rx_timeout_ms);
    return CAN_ESP_OK;
}

/*==============================================================================
                        FUNÇÕES DE COMUNICAÇÃO SÍNCRONA
 ==============================================================================*/

can_esp_status_t CAN_ESP_SendMessage(uint32_t id, const uint8_t *data, uint8_t length)
{
    twai_message_t message;
    if (data == NULL) {
        ESP_LOGE(TAG, "Ponteiro de dados nulo.");
        return CAN_ESP_ERR_NULL_POINTER;
    }
    if (length > CAN_MAX_DATA_LENGTH) {
        ESP_LOGE(TAG, "Tamanho inválido dos dados. Máximo de %u bytes permitido.", (unsigned int)CAN_MAX_DATA_LENGTH);
        return CAN_ESP_ERR_INVALID_LENGTH;
    }
    message.identifier = id;
    message.data_length_code = length;
    message.extd = 1U;
    message.rtr = 0;
    message.ss = 0;
    message.self = currentConfig.self_rx ? 1U : 0U;
    memcpy(message.data, data, length);
    if (currentConfig.use_checksum) {
        if (length < CAN_MAX_DATA_LENGTH) {
            uint8_t cs = CAN_ESP_CalculateChecksum(data, length);
            message.data[length] = cs;
            message.data_length_code = length + 1;
        } else {
            ESP_LOGE(TAG, "Não há espaço para checksum nos dados.");
            return CAN_ESP_ERR_INVALID_LENGTH;
        }
    }
    if (twai_transmit(&message, pdMS_TO_TICKS(currentConfig.transmit_timeout_ms)) != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao transmitir mensagem CAN (ID: 0x%08X).", (unsigned int)id);
        if (transmit_callback != NULL) {
            transmit_callback(id, data, length, CAN_ESP_ERR_TRANSMIT);
        }
        return CAN_ESP_ERR_TRANSMIT;
    }
    if (transmit_callback != NULL) {
        transmit_callback(id, data, length, CAN_ESP_OK);
    }
    return CAN_ESP_OK;
}

can_esp_status_t CAN_ESP_ReceiveMessage(CanEspMessage_t *message, uint32_t timeout_ms)
{
    twai_message_t rx_message;
    if (message == NULL) {
        ESP_LOGE(TAG, "Ponteiro para mensagem nulo.");
        return CAN_ESP_ERR_NULL_POINTER;
    }
    if (twai_receive(&rx_message, pdMS_TO_TICKS(timeout_ms)) == ESP_OK) {
        message->id = rx_message.identifier;
        message->length = rx_message.data_length_code;
        memcpy(message->data, rx_message.data, rx_message.data_length_code);
        if (currentConfig.use_checksum) {
            if (message->length < 1U) {
                ESP_LOGE(TAG, "Mensagem recebida sem dados para checksum.");
                return CAN_ESP_ERR_RECEIVE;
            }
            uint8_t calc_cs = CAN_ESP_CalculateChecksum(message->data, message->length - 1);
            if (calc_cs != message->data[message->length - 1]) {
                ESP_LOGE(TAG, "Falha na verificação de checksum para a mensagem (ID: 0x%08X).", (unsigned int)message->id);
                return CAN_ESP_ERR_RECEIVE;
            }
            message->length -= 1;
        }
        return CAN_ESP_OK;
    }
    ESP_LOGE(TAG, "Timeout ou erro ao receber mensagem CAN.");
    return CAN_ESP_ERR_TIMEOUT;
}

/* Função para registrar callback de recepção */
can_esp_status_t CAN_ESP_RegisterReceiveCallback(can_esp_receive_callback_t callback)
{
    if (callback == NULL) {
        ESP_LOGE(TAG, "Tentativa de registrar callback de recepção nulo.");
        return CAN_ESP_ERR_NULL_POINTER;
    }
    receive_callback = callback;
    ESP_LOGI(TAG, "Callback de recepção registrado com sucesso.");
    return CAN_ESP_OK;
}

/* Processa mensagens recebidas chamando o callback registrado */
void CAN_ESP_ProcessReceivedMessages(void)
{
    CanEspMessage_t received_msg;
    if (CAN_ESP_ReceiveMessage(&received_msg, CAN_PROCESS_TIMEOUT_MS) == CAN_ESP_OK) {
        if (currentConfig.debug_level >= 2) {
            ESP_LOGI(TAG, "Mensagem recebida - ID: 0x%08X, Length: %u",
                     (unsigned int)received_msg.id, (unsigned int)received_msg.length);
        }
        if (receive_callback != NULL) {
            receive_callback(&received_msg);
        }
    }
}

/*==============================================================================
                    FUNÇÕES DE TRANSMISSÃO ASSÍNCRONA
 ==============================================================================*/

can_esp_status_t CAN_ESP_EnqueueMessage(const CanEspMessage_t *msg, bool high_priority)
{
    CanEspMessage_t local_msg;
    BaseType_t ret;
    if (msg == NULL) {
        ESP_LOGE(TAG, "Ponteiro de mensagem nulo ao enfileirar.");
        return CAN_ESP_ERR_NULL_POINTER;
    }
    if (txQueue == NULL) {
        ESP_LOGE(TAG, "Fila de transmissão não inicializada.");
        return CAN_ESP_ERR_UNKNOWN;
    }
    memcpy(&local_msg, msg, sizeof(CanEspMessage_t));
    local_msg.retry_count = 0U;
    if (high_priority) {
        ret = xQueueSendToFront(txQueue, &local_msg, portMAX_DELAY);
    } else {
        ret = xQueueSend(txQueue, &local_msg, portMAX_DELAY);
    }
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Falha ao enfileirar mensagem para transmissão.");
        return CAN_ESP_ERR_TRANSMIT;
    }
    return CAN_ESP_OK;
}

/* Ajusta dinamicamente a prioridade da tarefa de transmissão com base na saturação da fila */
can_esp_status_t CAN_ESP_AdjustTransmitTaskPriority(void)
{
    UBaseType_t count;
    UBaseType_t threshold;
    const UBaseType_t baselinePriority = 10U;
    const UBaseType_t highPriority = 15U;
    UBaseType_t currentPriority;

    if (txQueue == NULL || canTxTaskHandle == NULL) {
        ESP_LOGE(TAG, "Fila de transmissão ou handle da tarefa nula.");
        return CAN_ESP_ERR_UNKNOWN;
    }
    count = uxQueueMessagesWaiting(txQueue);
    threshold = (TX_QUEUE_LENGTH * 80U) / 100U;
    currentPriority = uxTaskPriorityGet(canTxTaskHandle);
    if (count >= threshold && currentPriority < highPriority) {
        ESP_LOGI(TAG, "Alta saturação da fila (%u mensagens). Aumentando prioridade para %u.",
                 (unsigned int)count, (unsigned int)highPriority);
        vTaskPrioritySet(canTxTaskHandle, highPriority);
    } else if (count < threshold && currentPriority > baselinePriority) {
        ESP_LOGI(TAG, "Fila abaixo do limiar (%u mensagens). Restaurando prioridade para %u.",
                 (unsigned int)count, (unsigned int)baselinePriority);
        vTaskPrioritySet(canTxTaskHandle, baselinePriority);
    }
    return CAN_ESP_OK;
}

/* Tarefa de transmissão assíncrona */
static void CAN_ESP_TransmitTask(void *arg)
{
    CanEspMessage_t msg;
    twai_message_t tx_msg;
    int64_t tx_start, tx_end, latency;
    for (;;) {
        if (xQueueReceive(txQueue, &msg, portMAX_DELAY) == pdPASS) {
            convert_canesp_to_twai(&msg, &tx_msg);
            totalTransmissionAttempts++;
            tx_start = esp_timer_get_time();
            if (twai_transmit(&tx_msg, pdMS_TO_TICKS(currentConfig.transmit_timeout_ms)) != ESP_OK) {
                ESP_LOGE(TAG, "Falha ao transmitir mensagem (ID: 0x%08X).", (unsigned int)msg.id);
                if (msg.retry_count < CAN_ESP_MAX_RETRANSMISSIONS) {
                    msg.retry_count++;
                    totalRetransmissions++;
                    totalCollisions++;
                    vTaskDelay(pdMS_TO_TICKS(CAN_ESP_BACKOFF_MS));
                    (void)CAN_ESP_EnqueueMessage(&msg, true);
                } else {
                    if (transmit_callback != NULL) {
                        transmit_callback(msg.id, msg.data, msg.length, CAN_ESP_ERR_TRANSMIT);
                    }
                }
            } else {
                tx_end = esp_timer_get_time();
                latency = tx_end - tx_start;
                xSemaphoreTake(latencyMutex, portMAX_DELAY);
                latencyMetrics.num_samples++;
                latencyMetrics.total_latency += latency;
                if (latency < latencyMetrics.min_latency) {
                    latencyMetrics.min_latency = latency;
                }
                if (latency > latencyMetrics.max_latency) {
                    latencyMetrics.max_latency = latency;
                }
                xSemaphoreGive(latencyMutex);
                busLoadTotalTime += latency;
                if (currentConfig.debug_level >= 2) {
                    ESP_LOGI(TAG, "Mensagem (ID: 0x%08X) transmitida em %" PRId64 " ms",
                             (unsigned int)msg.id, (latency / 1000U));
                }
                if (transmit_callback != NULL) {
                    transmit_callback(msg.id, msg.data, msg.length, CAN_ESP_OK);
                }
            }
            (void)CAN_ESP_AdjustTransmitTaskPriority();
        }
    }
}

void CAN_ESP_StartTransmitTask(void)
{
    if (txQueue == NULL) {
        txQueue = xQueueCreate(TX_QUEUE_LENGTH, sizeof(CanEspMessage_t));
        if (txQueue == NULL) {
            ESP_LOGE(TAG, "Falha ao criar a fila de transmissão.");
            return;
        }
    }
    xTaskCreate(CAN_ESP_TransmitTask, "CAN_TX_Task", 4096, NULL, 10, &canTxTaskHandle);
}

/*==============================================================================
               FUNÇÕES AUXILIARES PARA CODIFICAÇÃO DO ID CAN
 ==============================================================================*/

uint32_t CAN_ESP_EncodeID(uint8_t priority, uint16_t module, uint16_t command)
{
    return (((uint32_t)(priority & 0x07U) << 26) |
            ((uint32_t)(module & 0x03FFU) << 16) |
            ((uint32_t)(command & 0xFFFFU)));
}

void CAN_ESP_DecodeID(uint32_t id, uint8_t *priority, uint16_t *module, uint16_t *command)
{
    if (priority != NULL) {
        *priority = (uint8_t)((id >> 26) & 0x07U);
    }
    if (module != NULL) {
        *module = (uint16_t)((id >> 16) & 0x03FFU);
    }
    if (command != NULL) {
        *command = (uint16_t)(id & 0xFFFFU);
    }
}

/*==============================================================================
            FUNÇÃO DE CALLBACK PARA TRANSMISSÃO (OPCIONAL)
 ==============================================================================*/

can_esp_status_t CAN_ESP_RegisterTransmitCallback(can_esp_transmit_callback_t callback)
{
    transmit_callback = callback;
    ESP_LOGI(TAG, "Callback de transmissão registrado com sucesso (opcional).");
    return CAN_ESP_OK;
}

/*==============================================================================
                 FUNÇÃO DE DIAGNÓSTICO / STATUS TWAI
 ==============================================================================*/

can_esp_status_t CAN_ESP_GetDiagnostics(CanEspDiagnostics_t *diag)
{
    twai_status_info_t status_info;
    if (diag == NULL) {
        ESP_LOGE(TAG, "Ponteiro de diagnóstico nulo.");
        return CAN_ESP_ERR_NULL_POINTER;
    }
    if (twai_get_status_info(&status_info) != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao obter status TWAI.");
        return CAN_ESP_ERR_UNKNOWN;
    }
    diag->tx_error_counter = status_info.tx_error_counter;
    diag->rx_error_counter = status_info.rx_error_counter;
    diag->bus_off = (status_info.state == TWAI_STATE_BUS_OFF) ? true : false;
    return CAN_ESP_OK;
}

/*==============================================================================
         FUNÇÃO PARA MONITORAMENTO DE LATÊNCIA
 ==============================================================================*/

can_esp_status_t CAN_ESP_GetLatencyMetrics(CanEspLatencyMetrics_t *metrics)
{
    if (metrics == NULL) {
        ESP_LOGE(TAG, "Ponteiro de métricas nulo.");
        return CAN_ESP_ERR_NULL_POINTER;
    }
    xSemaphoreTake(latencyMutex, portMAX_DELAY);
    *metrics = latencyMetrics;
    xSemaphoreGive(latencyMutex);
    return CAN_ESP_OK;
}

/*==============================================================================
         FUNÇÃO PARA CONSULTA DO STATUS DA FILA DE TRANSMISSÃO
 ==============================================================================*/

can_esp_status_t CAN_ESP_GetQueueStatus(CanEspQueueStatus_t *status)
{
    if (status == NULL) {
        ESP_LOGE(TAG, "Ponteiro de status nulo.");
        return CAN_ESP_ERR_NULL_POINTER;
    }
    if (txQueue == NULL) {
        ESP_LOGE(TAG, "Fila de transmissão não inicializada.");
        return CAN_ESP_ERR_UNKNOWN;
    }
    status->messages_waiting = uxQueueMessagesWaiting(txQueue);
    status->queue_capacity = TX_QUEUE_LENGTH;
    return CAN_ESP_OK;
}

/*==============================================================================
          FUNÇÃO PARA CALCULAR BUS LOAD
 ==============================================================================*/
/**
 * @brief Retorna a carga do barramento (bus load) em porcentagem.
 *
 * Calcula a razão entre o tempo total de transmissão acumulado (busLoadTotalTime)
 * e o tempo decorrido desde o início da medição (busLoadStartTime).
 *
 * @return uint32_t Porcentagem de bus load.
 */
uint32_t CAN_ESP_GetBusLoad(void)
{
    int64_t now = esp_timer_get_time();
    int64_t elapsed = now - busLoadStartTime;
    if (elapsed <= 0) {
        return 0U;
    }
    return (uint32_t)((busLoadTotalTime * 100LL) / elapsed);
}

/*==============================================================================
          FUNÇÃO PARA RETORNAR O TOTAL DE RETRANSMISSÕES OCORRIDAS
 ==============================================================================*/
/**
 * @brief Retorna o total de retransmissões ocorridas.
 *
 *  Essa métrica inclui as retransmissões devido a falhas.
 * 
 * @return uint32_t Total de retransmissões ocorridas.
 */
uint32_t CAN_ESP_GetRetransmissionCount(void)
{
    return totalRetransmissions;
}

/*==============================================================================
          FUNÇÃO PARA RETORNAR O TOTAL DE TRANSMISSÕES OCORRIDAS
 ==============================================================================*/
/**
 * @brief Retorna o número total de tentativas de transmissão CAN.
 *
 * Essa métrica inclui tanto as transmissões iniciais quanto as retransmissões devido a falhas.
 *
 * @return uint32_t Total de tentativas de transmissão.
 */
uint32_t CAN_ESP_GetTransmissionAttempts(void)
{
    return totalTransmissionAttempts;
}

/*==============================================================================
          FUNÇÃO PARA O TOTAL DE COLISÕES
 ==============================================================================*/
/**
 * @brief Retorna o número total de colisões (aproximação via retransmissões).
 *
 * Como o driver TWAI não fornece um contador específico para colisões, este valor é
 * derivado do número de mensagens que precisaram ser retransmitidas.
 *
 * @return uint32_t Número total de colisões.
 */
uint32_t CAN_ESP_GetCollisionCount(void)
{
    return totalCollisions;
}
 
/**
 * @brief Calcula a taxa de colisões (em porcentagem).
 *
 * A taxa de colisões é definida como:
 *     (número total de colisões / total de transmissões realizadas) * 100
 *
 * @return uint32_t Taxa de colisões em porcentagem.
 */
uint32_t CAN_ESP_GetCollisionRate(void)
{
    if (totalTransmissionAttempts == 0) {
        return 0U;
    }
    return (uint32_t)((totalCollisions * 100ULL) / totalTransmissionAttempts);
}

/*==============================================================================
          IMPLEMENTAÇÃO DA TAREFA DE RECEPÇÃO (EVENTOS)
 ==============================================================================*/

static void CAN_ESP_ReceiveTask(void *arg)
{
    CanEspMessage_t msg;
    for (;;) {
        if (CAN_ESP_ReceiveMessage(&msg, portMAX_DELAY) == CAN_ESP_OK) {
            if (receive_callback != NULL) {
                receive_callback(&msg);
            }
        }
    }
}

void CAN_ESP_StartReceiveTask(void)
{
    xTaskCreate(CAN_ESP_ReceiveTask, "CAN_RX_Task", 4096, NULL, 10, NULL);
}

/*==============================================================================
       IMPLEMENTAÇÃO DO TESTE DE LOOPBACK (ROUND-TRIP TIME)
 ==============================================================================*/

/**
 * @brief Realiza um teste de loopback para medir o tempo de resposta total (round-trip time).
 *
 * Envia uma mensagem com o timestamp atual e, utilizando self_rx, aguarda o retorno para
 * calcular o tempo de round-trip.
 */
can_esp_status_t CAN_ESP_MeasureRoundTripTime(int64_t *round_trip_time, uint32_t timeout_ms)
{
    if (round_trip_time == NULL) {
        ESP_LOGE(TAG, "Ponteiro de round_trip_time nulo.");
        return CAN_ESP_ERR_NULL_POINTER;
    }

    int64_t send_timestamp = esp_timer_get_time();
    uint8_t payload[8];
    memcpy(payload, &send_timestamp, sizeof(send_timestamp));

    /* Ativa self_rx temporariamente */
    bool original_self_rx = currentConfig.self_rx;
    xSemaphoreTake(configMutex, portMAX_DELAY);
    currentConfig.self_rx = true;
    xSemaphoreGive(configMutex);

    can_esp_status_t status = CAN_ESP_SendMessage(CAN_ESP_SELF_TEST_ID, payload, sizeof(payload));
    if (status != CAN_ESP_OK) {
        ESP_LOGE(TAG, "Falha ao enviar mensagem de self-test.");
        xSemaphoreTake(configMutex, portMAX_DELAY);
        currentConfig.self_rx = original_self_rx;
        xSemaphoreGive(configMutex);
        return status;
    }

    CanEspMessage_t rx_msg;
    status = CAN_ESP_ReceiveMessage(&rx_msg, timeout_ms);
    if (status != CAN_ESP_OK) {
        ESP_LOGE(TAG, "Falha ou timeout na recepção da mensagem de self-test.");
        xSemaphoreTake(configMutex, portMAX_DELAY);
        currentConfig.self_rx = original_self_rx;
        xSemaphoreGive(configMutex);
        return status;
    }

    if (rx_msg.length < sizeof(send_timestamp)) {
        ESP_LOGE(TAG, "Mensagem de self-test com tamanho inválido.");
        xSemaphoreTake(configMutex, portMAX_DELAY);
        currentConfig.self_rx = original_self_rx;
        xSemaphoreGive(configMutex);
        return CAN_ESP_ERR_RECEIVE;
    }
    int64_t received_timestamp = 0;
    memcpy(&received_timestamp, rx_msg.data, sizeof(received_timestamp));

    *round_trip_time = esp_timer_get_time() - send_timestamp;
    ESP_LOGI(TAG, "Self-test round-trip time: %" PRId64 " ms", (*round_trip_time / 1000U));

    xSemaphoreTake(configMutex, portMAX_DELAY);
    currentConfig.self_rx = original_self_rx;
    xSemaphoreGive(configMutex);

    return CAN_ESP_OK;
}
