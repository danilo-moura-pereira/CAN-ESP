/*
 * diagnosis_module.c
 * Implementação do Módulo de Diagnóstico para a ECU de Monitoramento e Diagnóstico.
 * Projeto acadêmico para Mestrado Profissional em Engenharia Elétrica - UnB
 * Adaptado para conformidade com MISRA C:2012 e padrões automotivos.
 *
 * Este módulo coleta dados críticos da camada CAN (can_esp_lib), processa e analisa
 * esses dados comparando-os com limiares configuráveis, além de armazenar as leituras
 * em um buffer circular para análise histórica. Um callback de alerta pode ser registrado
 * para notificar imediatamente condições anormais.
 */

#include "diagnosis_module.h"

#include "esp_log.h"
#include "esp_timer.h"

#include <stdio.h>
#include <math.h>

static const char *TAG = "DIAGNOSIS_MODULE";

/* Buffer circular para armazenamento histórico e índice corrente */
static DiagnosisData_t diag_history[DIAG_HISTORY_SIZE];
static uint32_t diag_history_index = 0U;

/* Limiar críticos padrão (inicialmente) */
static uint32_t threshold_tx_errors   = 10U;
static uint32_t threshold_rx_errors   = 10U;
static uint32_t threshold_bus_load    = 80U;      /* porcentagem */
static int64_t  threshold_max_latency = 5000;     /* microsegundos */
static uint32_t threshold_retrans     = 5U;
static uint32_t threshold_collisions  = 5U;

/* Callback de alerta para notificação em tempo real */
static diagnosis_alert_callback_t alert_callback = NULL;

/* Função interna para analisar os dados e comparar com os limiares */
static void analyze_diagnosis_data(DiagnosisData_t *data)
{
    data->abnormal = false;

    if (data->can_diag.tx_error_counter > threshold_tx_errors) {
        ESP_LOGW(TAG, "Alerta: TX Erros elevados (%" PRIu32 ").", data->can_diag.tx_error_counter);
        data->abnormal = true;
    }
    if (data->can_diag.rx_error_counter > threshold_rx_errors) {
        ESP_LOGW(TAG, "Alerta: RX Erros elevados (%" PRIu32 ").", data->can_diag.rx_error_counter);
        data->abnormal = true;
    }
    if (data->bus_load > threshold_bus_load) {
        ESP_LOGW(TAG, "Alerta: Bus Load elevado (%" PRIu32 "%%).", data->bus_load);
        data->abnormal = true;
    }
    if (data->latency.max_latency > threshold_max_latency) {
        ESP_LOGW(TAG, "Alerta: Latência máxima elevada (%" PRId64 " ms).", (data->latency.max_latency / 1000U));
        data->abnormal = true;
    }
    if (data->retransmission_count > threshold_retrans) {
        ESP_LOGW(TAG, "Alerta: Retransmissões elevadas (%" PRIu32 ").", data->retransmission_count);
        data->abnormal = true;
    }
    if (data->collision_count > threshold_collisions) {
        ESP_LOGW(TAG, "Alerta: Colisões elevadas (%" PRIu32 ").", data->collision_count);
        data->abnormal = true;
    }

    /* Se um callback de alerta estiver registrado e alguma condição anormal foi detectada, notifica */
    if (data->abnormal && alert_callback != NULL) {
        alert_callback(data);
    }
}

/*------------------------------------------------------------------------------
 * Funções de configuração e atualização de diagnóstico
 *----------------------------------------------------------------------------*/

bool diagnosis_module_init(void)
{
    if (CAN_ESP_Init() != CAN_ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao inicializar a camada CAN.");
        return false;
    }
    ESP_LOGI(TAG, "Módulo de diagnóstico inicializado com sucesso.");
    return true;
}

bool diagnosis_module_update(DiagnosisData_t *data)
{
    if (data == NULL)
    {
        ESP_LOGE(TAG, "Ponteiro de dados de diagnóstico nulo.");
        return false;
    }

    /* Coleta dados da camada CAN */
    if (CAN_ESP_GetDiagnostics(&data->can_diag) != CAN_ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao obter diagnóstico CAN.");
        return false;
    }
    if (CAN_ESP_GetLatencyMetrics(&data->latency) != CAN_ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao obter métricas de latência.");
        return false;
    }
    if (CAN_ESP_GetQueueStatus(&data->queue_status) != CAN_ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao obter status da fila de transmissão.");
        return false;
    }
    data->bus_load = CAN_ESP_GetBusLoad();
    data->retransmission_count = CAN_ESP_GetRetransmissionCount();
    data->collision_count = CAN_ESP_GetCollisionCount();
    data->transmission_attempts = CAN_ESP_GetTransmissionAttempts();

    /* Define o timestamp atual para o registro desta medição */
    data->timestamp = esp_timer_get_time();

    /* Processa e analisa os dados em relação aos limiares */
    analyze_diagnosis_data(data);

    /* Armazena a leitura atual no buffer histórico */
    diag_history[diag_history_index] = *data;
    diag_history_index = (diag_history_index + 1U) % DIAG_HISTORY_SIZE;

    return true;
}

void diagnosis_module_print(const DiagnosisData_t *data)
{
    if (data == NULL)
    {
        ESP_LOGE(TAG, "Dados de diagnóstico nulos para impressão.");
        return;
    }
    
    ESP_LOGI(TAG, "Diagnóstico CAN: TX Erros = %" PRIu32 ", RX Erros = %" PRIu32 ", Bus-Off = %s",
             data->can_diag.tx_error_counter,
             data->can_diag.rx_error_counter,
             data->can_diag.bus_off ? "Sim" : "Não");
    
    ESP_LOGI(TAG, "Métricas de Latência: Amostras = %" PRIu32 ", Total = %" PRId64 " ms, Mínima = %" PRId64 " ms, Máxima = %" PRId64 " ms",
             (data->latency.num_samples / 1000U),
             (data->latency.total_latency / 1000U),
             (data->latency.min_latency / 1000U),
             (data->latency.max_latency / 1000U));
    
    ESP_LOGI(TAG, "Status da Fila: %u mensagens esperando de %u",
             (unsigned int)data->queue_status.messages_waiting,
             (unsigned int)data->queue_status.queue_capacity);
    
    ESP_LOGI(TAG, "Bus Load: %" PRIu32 "%%", data->bus_load);
    
    ESP_LOGI(TAG, "Retransmissões Totais: %" PRIu32 "", data->retransmission_count);
    ESP_LOGI(TAG, "Colisões Totais: %" PRIu32 "", data->collision_count);
    ESP_LOGI(TAG, "Tentativas de Transmissão: %" PRIu32 "", data->transmission_attempts);
    
    ESP_LOGI(TAG, "Timestamp da medição: %" PRId64 " ms", (data->timestamp / 1000U));
    
    if (data->abnormal) {
        ESP_LOGW(TAG, "Condição anormal detectada nos parâmetros de diagnóstico.");
    }
}

/* Retorna o histórico de diagnósticos */
bool diagnosis_module_get_history(DiagnosisData_t *history, uint32_t max_entries, uint32_t *num_entries)
{
    uint32_t i;
    if (history == NULL || num_entries == NULL) {
        ESP_LOGE(TAG, "Parâmetros nulos na obtenção do histórico.");
        return false;
    }
    for (i = 0U; i < DIAG_HISTORY_SIZE && i < max_entries; i++) {
        history[i] = diag_history[i];
    }
    *num_entries = i;
    return true;
}

/*------------------------------------------------------------------------------
 * Funções de gestão dinâmica de limiares e notificação
 *----------------------------------------------------------------------------*/

/**
 * @brief Atualiza os limiares críticos utilizados para análise.
 *
 * Permite ajustar dinamicamente os limiares de erros, bus load, latência, retransmissões e colisões.
 *
 * @return true se os limiares forem atualizados com sucesso, false caso contrário.
 */
bool diagnosis_module_set_thresholds(uint32_t tx_errors, uint32_t rx_errors, uint32_t bus_load,
                                       int64_t max_latency, uint32_t retransmissions, uint32_t collisions)
{
    threshold_tx_errors = tx_errors;
    threshold_rx_errors = rx_errors;
    threshold_bus_load = bus_load;
    threshold_max_latency = max_latency;
    threshold_retrans = retransmissions;
    threshold_collisions = collisions;
    ESP_LOGI(TAG, "Limiar críticos atualizados.");
    return true;
}

/**
 * @brief Registra um callback para alertas em tempo real.
 *
 * Este callback será invocado sempre que a análise dos dados indicar uma condição anormal.
 *
 * @return true se o callback for registrado com sucesso, false caso contrário.
 */
bool diagnosis_module_register_alert_callback(diagnosis_alert_callback_t callback)
{
    if (callback == NULL) {
        ESP_LOGE(TAG, "Tentativa de registrar callback de alerta nulo.");
        return false;
    }
    alert_callback = callback;
    ESP_LOGI(TAG, "Callback de alerta registrado com sucesso.");
    return true;
}

/*------------------------------------------------------------------------------
 * Funções para estatísticas avançadas da latência
 *------------------------------------------------------------------------------*/

/**
 * @brief Calcula a média e o desvio padrão da latência (max_latency) utilizando o histórico.
 *
 * Itera sobre o buffer histórico e calcula a média e o desvio padrão do campo max_latency.
 *
 * @param[out] average Ponteiro para a média da latência (em microsegundos).
 * @param[out] stddev Ponteiro para o desvio padrão da latência (em microsegundos).
 * @return true se os cálculos forem realizados com sucesso, false caso contrário.
 */
bool diagnosis_module_get_latency_statistics(int64_t *average, int64_t *stddev)
{
    uint32_t count = DIAG_HISTORY_SIZE;
    uint32_t valid_samples = 0U;
    int64_t sum = 0;
    int64_t sum_sq = 0;
    uint32_t i;

    if (average == NULL || stddev == NULL) {
        ESP_LOGE(TAG, "Ponteiro nulo nos parâmetros de estatísticas.");
        return false;
    }
    /* Percorre o buffer histórico para acumular os valores de latência máxima */
    for (i = 0U; i < count; i++) {
        if (diag_history[i].timestamp != 0) { /* Considera apenas entradas válidas */
            int64_t lat = diag_history[i].latency.max_latency;
            sum += lat;
            sum_sq += (lat * lat);
            valid_samples++;
        }
    }
    if (valid_samples == 0U) {
        ESP_LOGW(TAG, "Nenhuma amostra válida para estatísticas de latência.");
        *average = 0;
        *stddev = 0;
        return true;
    }
    *average = sum / valid_samples;
    /* Calcula o desvio padrão (populacional) */
    double mean = (double)*average;
    double variance = ((double)sum_sq / valid_samples) - (mean * mean);
    *stddev = (int64_t)sqrt(variance);
    return true;
}
