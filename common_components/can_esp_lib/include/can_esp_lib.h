/*
 * can_esp_lib.h
 * Cabeçalho da Biblioteca CAN para ESP32 (ESP-IDF)
 * Projeto acadêmico para Mestrado Profissional em Engenharia Elétrica - UnB
 * Adaptado para conformidade com MISRA C:2012.
 * Nome da biblioteca: can_esp_lib
 */

#ifndef CAN_ESP_LIB_H
#define CAN_ESP_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "driver/twai.h"  /* Para os tipos twai_filter_config_t, twai_mode_t, twai_timing_config_t */

/* Número máximo de bytes de dados em uma mensagem CAN */
#define CAN_MAX_DATA_LENGTH    (8U)

/* Timeouts padrão (em milissegundos) */
#define CAN_DEFAULT_TRANSMIT_TIMEOUT_MS    (1000U)
#define CAN_DEFAULT_RECEIVE_TIMEOUT_MS     (1000U)

/* Pinos CAN padrão */
#ifndef CAN_TX_GPIO
#define CAN_TX_GPIO    (5U)
#endif

#ifndef CAN_RX_GPIO
#define CAN_RX_GPIO    (4U)
#endif

/* Controle de retransmissões */
#define CAN_ESP_MAX_RETRANSMISSIONS  (3U)
#define CAN_ESP_BACKOFF_MS           (50U)

#define CAN_PROCESS_TIMEOUT_MS    (10U)
#define TX_QUEUE_LENGTH 32

/**
 * @brief Estrutura para configuração dinâmica da camada CAN.
 */
typedef struct
{
    uint32_t bitrate;
    uint8_t  tx_gpio;
    uint8_t  rx_gpio;
    uint32_t transmit_timeout_ms;
    uint32_t receive_timeout_ms;
    twai_filter_config_t filter_config;
    twai_mode_t mode;
    bool use_custom_timing;
    twai_timing_config_t custom_timing_config;
    bool auto_retransmit;
    uint8_t debug_level;
    bool self_rx;
    bool use_checksum;  /**< Se verdadeiro, calcula e verifica checksum */
} CanEspConfig_t;

/**
 * @brief Estrutura para mensagens CAN.
 *
 * O campo retry_count armazena o número de tentativas de retransmissão.
 */
typedef struct
{
    uint32_t id;
    uint8_t  length;
    uint8_t  data[CAN_MAX_DATA_LENGTH];
    uint8_t  retry_count;
} CanEspMessage_t;

/**
 * @brief Estrutura para diagnósticos da interface TWAI.
 */
typedef struct {
    uint32_t tx_error_counter;
    uint32_t rx_error_counter;
    bool     bus_off;
} CanEspDiagnostics_t;

/**
 * @brief Estrutura para status da fila de transmissão.
 */
typedef struct {
    UBaseType_t messages_waiting;
    UBaseType_t queue_capacity;
} CanEspQueueStatus_t;

/**
 * @brief Estrutura para métricas de latência de transmissão.
 */
typedef struct {
    uint32_t num_samples;
    int64_t total_latency;
    int64_t min_latency;
    int64_t max_latency;
} CanEspLatencyMetrics_t;

/**
 * @brief Enumeração dos códigos de status da biblioteca.
 */
typedef enum
{
    CAN_ESP_OK = 0,
    CAN_ESP_ERR_NULL_POINTER,
    CAN_ESP_ERR_INVALID_LENGTH,
    CAN_ESP_ERR_TRANSMIT,
    CAN_ESP_ERR_RECEIVE,
    CAN_ESP_ERR_DRIVER_INSTALL,
    CAN_ESP_ERR_DRIVER_START,
    CAN_ESP_ERR_DRIVER_STOP,
    CAN_ESP_ERR_DRIVER_UNINSTALL,
    CAN_ESP_ERR_TIMEOUT,
    CAN_ESP_ERR_UNKNOWN
} can_esp_status_t;

/* Protótipos de funções de configuração dinâmica */
can_esp_status_t CAN_ESP_InitWithConfig(const CanEspConfig_t *config);
can_esp_status_t CAN_ESP_Init(void);
can_esp_status_t CAN_ESP_UpdateConfig(const CanEspConfig_t *config);
can_esp_status_t CAN_ESP_Deinit(void);

/* Protótipos de funções de atualização parcial */
can_esp_status_t CAN_ESP_SetFilterConfig(const twai_filter_config_t *new_filter_config);
can_esp_status_t CAN_ESP_SetTimeouts(uint32_t tx_timeout_ms, uint32_t rx_timeout_ms);

/* Protótipos de funções de comunicação síncrona */
can_esp_status_t CAN_ESP_SendMessage(uint32_t id, const uint8_t *data, uint8_t length);
can_esp_status_t CAN_ESP_ReceiveMessage(CanEspMessage_t *message, uint32_t timeout_ms);

/* Protótipos de funções de callback e processamento de mensagens recebidas */
typedef void (*can_esp_receive_callback_t)(const CanEspMessage_t *msg);
can_esp_status_t CAN_ESP_RegisterReceiveCallback(can_esp_receive_callback_t callback);
void CAN_ESP_ProcessReceivedMessages(void);

/* Protótipos de funções para transmissão assíncrona */
can_esp_status_t CAN_ESP_EnqueueMessage(const CanEspMessage_t *msg, bool high_priority);
void CAN_ESP_StartTransmitTask(void);

/* Função para iniciar a tarefa de recepção baseada em eventos */
void CAN_ESP_StartReceiveTask(void);

/* Protótipos de funções auxiliares para codificação do ID CAN */
uint32_t CAN_ESP_EncodeID(uint8_t priority, uint16_t module, uint16_t command);
void CAN_ESP_DecodeID(uint32_t id, uint8_t *priority, uint16_t *module, uint16_t *command);

/* Protótipos de funções de callback para transmissão (opcional) */
typedef void (*can_esp_transmit_callback_t)(uint32_t id, const uint8_t *data, uint8_t length, can_esp_status_t status);
can_esp_status_t CAN_ESP_RegisterTransmitCallback(can_esp_transmit_callback_t callback);

/* Protótipos de funções de diagnóstico e monitoramento */
can_esp_status_t CAN_ESP_GetDiagnostics(CanEspDiagnostics_t *diag);
can_esp_status_t CAN_ESP_GetLatencyMetrics(CanEspLatencyMetrics_t *metrics);
can_esp_status_t CAN_ESP_GetQueueStatus(CanEspQueueStatus_t *status);
uint32_t CAN_ESP_GetBusLoad(void);

/* Protótipo da função para calcular checksum */
uint8_t CAN_ESP_CalculateChecksum(const uint8_t *data, uint8_t length);

/* Protótipo para ajuste dinâmico da prioridade da tarefa de transmissão */
can_esp_status_t CAN_ESP_AdjustTransmitTaskPriority(void);

/* Função para obter o total de transmissões ocorridas */
uint32_t CAN_ESP_GetTransmissionAttempts(void);

/* Função para obter o total de retransmissões ocorridas */
uint32_t CAN_ESP_GetRetransmissionCount(void);

/* Nova funcionalidade: Métricas de colisões */
uint32_t CAN_ESP_GetCollisionCount(void);
uint32_t CAN_ESP_GetCollisionRate(void);

/* Nova funcionalidade: Medição do tempo de resposta total (round-trip time) via loopback */
/**
 * @brief Identificador reservado para o teste de loopback self.
 */
#define CAN_ESP_SELF_TEST_ID    (0x0F000001U)

/**
 * @brief Realiza um teste de loopback para medir o tempo de resposta total (round-trip time).
 *
 * Esta função envia uma mensagem contendo o timestamp atual (em microsegundos) como payload e,
 * em modo self‑rx, aguarda a recepção da mesma mensagem. O tempo de resposta total é calculado
 * como a diferença entre o timestamp da recepção e o timestamp originalmente enviado.
 *
 * @param[out] round_trip_time Ponteiro para onde será armazenado o tempo de round-trip (em microsegundos).
 * @param[in] timeout_ms Tempo máximo de espera (em milissegundos) para a recepção da mensagem.
 * @return can_esp_status_t CAN_ESP_OK se o teste for bem-sucedido, ou um código de erro apropriado.
 */
can_esp_status_t CAN_ESP_MeasureRoundTripTime(int64_t *round_trip_time, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* CAN_ESP_LIB_H */
