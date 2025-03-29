/*
 * alert_module.h
 * Cabeçalho do Módulo de Alertas da ECU de Monitoramento e Diagnóstico.
 * Projeto acadêmico para Mestrado Profissional em Engenharia Elétrica - UnB.
 * Adaptado para conformidade com MISRA C:2012 e ISO 11898.
 */

#ifndef ALERT_MODULE_H
#define ALERT_MODULE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "diagnosis_module.h"  /* Para acesso à estrutura DiagnosisData_t */

/* Número máximo de entradas no histórico de alertas */
#define ALERT_HISTORY_SIZE 100U

/**
 * @brief Enumeração dos níveis de alerta.
 */
typedef enum {
    ALERT_LEVEL_INFO = 0,     /**< Alerta informativo */
    ALERT_LEVEL_WARNING = 1,  /**< Alerta de aviso */
    ALERT_LEVEL_CRITICAL = 2  /**< Alerta crítico */
} AlertLevel_t;

/**
 * @brief Estrutura que armazena uma entrada de alerta.
 */
typedef struct {
    uint32_t timestamp;             /**< Timestamp em milissegundos */
    AlertLevel_t level;             /**< Nível do alerta */
    char message[128];              /**< Mensagem descritiva do alerta */
} AlertData_t;

/**
 * @brief Estrutura para configuração dos limiares de alerta.
 *
 * Permite definir os valores críticos para erros de transmissão e recepção,
 * carga do barramento e retransmissões.
 */
typedef struct {
    uint32_t max_tx_error_threshold;         /**< Limiar para erros de transmissão */
    uint32_t max_rx_error_threshold;         /**< Limiar para erros de recepção */
    uint32_t max_bus_load_threshold;           /**< Limiar para carga do barramento (em porcentagem) */
    uint32_t max_retransmission_threshold;     /**< Limiar para retransmissões */
} AlertThresholds_t;

/**
 * @brief Callback para notificação em tempo real de alertas.
 *
 * Esta função será chamada sempre que um novo alerta for registrado.
 *
 * @param alert Ponteiro para a estrutura AlertData_t contendo os detalhes do alerta.
 */
typedef void (*alert_notification_callback_t)(const AlertData_t *alert);

/**
 * @brief Inicializa o módulo de alertas.
 *
 * Prepara o histórico interno e realiza a configuração inicial.
 *
 * @return true se a inicialização for bem-sucedida, false caso contrário.
 */
bool alert_module_init(void);

/**
 * @brief Verifica as condições de diagnóstico e gera alertas conforme os limiares configurados.
 *
 * Essa função avalia os dados presentes em diag e, se algum limiar crítico for ultrapassado,
 * registra um alerta e notifica via callback.
 *
 * @param diag Ponteiro para os dados de diagnóstico (DiagnosisData_t).
 */
void alert_module_check_conditions(const DiagnosisData_t *diag);

/**
 * @brief Obtém o histórico de alertas armazenados.
 *
 * Copia as entradas do histórico para o buffer fornecido.
 *
 * @param[out] buffer Ponteiro para o buffer de AlertData_t.
 * @param[in] max_entries Número máximo de entradas que o buffer pode armazenar.
 * @return O número de entradas copiadas.
 */
uint32_t alert_module_get_history(AlertData_t *buffer, uint32_t max_entries);

/**
 * @brief Imprime o histórico de alertas via ESP_LOG.
 */
void alert_module_print_history(void);

/**
 * @brief Define os limiares de alerta dinamicamente.
 *
 * Permite atualizar os valores críticos para erros, carga do barramento e retransmissões.
 *
 * @param thresholds Ponteiro para a estrutura AlertThresholds_t com os novos limiares.
 */
void alert_module_set_thresholds(const AlertThresholds_t *thresholds);

/**
 * @brief Registra um callback para notificação em tempo real de alertas.
 *
 * Quando um novo alerta for registrado, a função callback é invocada para que outros módulos
 * possam ser notificados imediatamente.
 *
 * @param callback Função callback para notificação.
 */
void alert_module_register_notification_callback(alert_notification_callback_t callback);

#ifdef __cplusplus
}
#endif

#endif /* ALERT_MODULE_H */
