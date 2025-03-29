/*
 * diagnosis_module.h
 * Cabeçalho do Módulo de Diagnóstico para a ECU de Monitoramento e Diagnóstico.
 * Projeto acadêmico para Mestrado Profissional em Engenharia Elétrica - UnB
 * Adaptado para conformidade com MISRA C:2012 e padrões automotivos (ISO 11898).
 *
 * Este módulo coleta, processa, analisa e armazena as métricas críticas da rede CAN.
 * Ele fornece uma interface para consulta dos dados atuais e históricos, possibilitando
 * o monitoramento da integridade do sistema e a geração de alertas em tempo real.
 */

#ifndef DIAGNOSIS_MODULE_H
#define DIAGNOSIS_MODULE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "can_esp_lib.h"

/* Tamanho do buffer histórico */
#define DIAG_HISTORY_SIZE 50U

/**
 * @brief Estrutura que agrega os dados de diagnóstico do sistema.
 *
 * Essa estrutura consolida os dados coletados da camada CAN e os resultados da análise.
 */
typedef struct {
    CanEspDiagnostics_t can_diag;           /**< Diagnóstico do barramento CAN (TX/RX erros, Bus-Off). */
    CanEspLatencyMetrics_t latency;         /**< Métricas de latência da transmissão CAN. */
    CanEspQueueStatus_t queue_status;       /**< Status da fila de transmissão. */
    uint32_t bus_load;                      /**< Carga do barramento CAN (em porcentagem). */
    uint32_t retransmission_count;          /**< Número total de retransmissões ocorridas. */
    uint32_t collision_count;               /**< Número total de colisões (proxy). */
    uint32_t transmission_attempts;         /**< Número total de tentativas de transmissão. */
    bool abnormal;                          /**< Indicador de condição anormal (baseado em limiares). */
    int64_t timestamp;                      /**< Timestamp da medição (em microsegundos). */
} DiagnosisData_t;

/**
 * @brief Callback para notificação de alerta em tempo real.
 *
 * Esse callback é chamado quando os dados processados indicam uma condição anormal.
 *
 * @param data Ponteiro para a estrutura DiagnosisData_t com os dados atuais.
 */
typedef void (*diagnosis_alert_callback_t)(const DiagnosisData_t *data);

/**
 * @brief Inicializa o módulo de diagnóstico.
 *
 * Verifica se a camada can_esp_lib está operacional e inicializa estruturas internas,
 * inclusive o buffer histórico.
 *
 * @return true se a inicialização for bem-sucedida, false caso contrário.
 */
bool diagnosis_module_init(void);

/**
 * @brief Atualiza os dados de diagnóstico.
 *
 * Coleta os dados da camada can_esp_lib, processa e analisa os valores comparando-os
 * com os limiares críticos definidos. Armazena a medição atual com timestamp em um buffer
 * circular para histórico.
 *
 * @param[out] data Ponteiro para a estrutura DiagnosisData_t que receberá os dados atualizados.
 * @return true se a atualização ocorrer com sucesso, false caso contrário.
 */
bool diagnosis_module_update(DiagnosisData_t *data);

/**
 * @brief Exibe os dados de diagnóstico.
 *
 * Imprime os dados coletados e processados no log, facilitando a análise dos parâmetros críticos.
 *
 * @param data Ponteiro para a estrutura DiagnosisData_t contendo os dados.
 */
void diagnosis_module_print(const DiagnosisData_t *data);

/**
 * @brief Retorna o histórico de dados de diagnóstico.
 *
 * Copia até max_entries registros do buffer histórico para o array history.
 *
 * @param[out] history Array que receberá os registros históricos.
 * @param[in] max_entries Tamanho máximo do array history.
 * @param[out] num_entries Número real de registros copiados.
 * @return true se os registros forem copiados com sucesso, false caso contrário.
 */
bool diagnosis_module_get_history(DiagnosisData_t *history, uint32_t max_entries, uint32_t *num_entries);

/**
 * @brief Atualiza dinamicamente os limiares críticos usados para análise.
 *
 * Permite configurar os limiares de: erros de transmissão, erros de recepção, bus load,
 * latência máxima, retransmissões e colisões.
 *
 * @param tx_errors Limiar para erros de transmissão.
 * @param rx_errors Limiar para erros de recepção.
 * @param bus_load Limiar para a carga do barramento (porcentagem).
 * @param max_latency Limiar para a latência máxima (em microsegundos).
 * @param retransmissions Limiar para número de retransmissões.
 * @param collisions Limiar para número de colisões.
 * @return true se os limiares forem atualizados com sucesso, false caso contrário.
 */
bool diagnosis_module_set_thresholds(uint32_t tx_errors, uint32_t rx_errors, uint32_t bus_load,
                                       int64_t max_latency, uint32_t retransmissions, uint32_t collisions);

/**
 * @brief Registra um callback para notificação de alertas em tempo real.
 *
 * Sempre que os dados analisados indicarem uma condição anormal, o callback será invocado.
 *
 * @param callback Ponteiro para a função de alerta.
 * @return true se o callback for registrado com sucesso, false caso contrário.
 */
bool diagnosis_module_register_alert_callback(diagnosis_alert_callback_t callback);

/**
 * @brief Calcula estatísticas avançadas da latência utilizando o histórico.
 *
 * Calcula a média e o desvio padrão da latência (campo max_latency de cada medição)
 * a partir dos registros históricos.
 *
 * @param[out] average Ponteiro para armazenar a média da latência (em microsegundos).
 * @param[out] stddev Ponteiro para armazenar o desvio padrão da latência (em microsegundos).
 * @return true se os cálculos forem realizados com sucesso, false caso contrário.
 */
bool diagnosis_module_get_latency_statistics(int64_t *average, int64_t *stddev);

#ifdef __cplusplus
}
#endif

#endif /* DIAGNOSIS_MODULE_H */
