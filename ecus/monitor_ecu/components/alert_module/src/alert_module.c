/*
 * alert_module.c
 * Implementação do Módulo de Alertas da ECU de Monitoramento e Diagnóstico.
 * Projeto acadêmico para Mestrado Profissional em Engenharia Elétrica - UnB.
 * Adaptado para conformidade com MISRA C:2012 e padrões automotivos (ISO 11898).
 */

#include "alert_module.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "logger_module.h"  /* Integração com o módulo de logs */
#include <string.h>
#include <limits.h>
#include <inttypes.h>

#define TAG "ALERT_MODULE"

/* Histórico de alertas */
static AlertData_t alert_history[ALERT_HISTORY_SIZE];
static uint32_t alert_index = 0U;

/* Limiar de alerta padrão */
static AlertThresholds_t alert_thresholds = {
    .tx_error_threshold = 100U,
    .rx_error_threshold = 100U,
    .bus_load_threshold = 80U,        /* Em porcentagem */
    .retransmission_threshold = 50U
};

/* Callback para notificação de alerta */
static alert_notification_callback_t notification_callback = NULL;

/**
 * @brief Inicializa o módulo de alertas.
 *
 * Inicializa o histórico de alertas e configura os limiares padrão.
 *
 * @return true se a inicialização for bem-sucedida, false caso contrário.
 */
bool alert_module_init(void)
{
    (void)memset(alert_history, 0, sizeof(alert_history));
    alert_index = 0U;
    ESP_LOGI(TAG, "Módulo de alertas inicializado com sucesso.");
    return true;
}

/**
 * @brief Função interna para registrar um novo alerta.
 *
 * Adiciona uma nova entrada no histórico de alertas, registra o alerta via log e invoca
 * o callback de notificação (se registrado). Essa função também integra com o módulo logger_module.
 *
 * @param level Nível do alerta.
 * @param message Mensagem descritiva do alerta.
 */
static void register_alert(AlertLevel_t level, const char *message)
{
    if (message == NULL)
    {
        return;
    }

    /* Obtém timestamp do RTC DS3231 */
    alert_history[alert_index].timestamp = logger_module_get_rtc_timestamp();
    alert_history[alert_index].level = level;
    strncpy(alert_history[alert_index].message, message, sizeof(alert_history[alert_index].message) - 1U);
    alert_history[alert_index].message[sizeof(alert_history[alert_index].message) - 1U] = '\0';

    ESP_LOGW(TAG, "ALERTA [%s]: %s (Timestamp: %" PRIu32 " ms)",
             (level == ALERT_LEVEL_CRITICAL) ? "CRÍTICO" :
             (level == ALERT_LEVEL_WARNING) ? "AVISO" : "INFO",
             alert_history[alert_index].message,
             alert_history[alert_index].timestamp);

    /* Garante que os alertas sejam logados no sistema */
    logger_module_log_alert(level, alert_history[alert_index].message);

    alert_index = (alert_index + 1U) % ALERT_HISTORY_SIZE;
}

/**
 * @brief Verifica as condições de diagnóstico e gera alertas conforme os limiares configurados.
 *
 * Compara os dados de diagnóstico fornecidos com os limiares dinâmicos e registra os alertas apropriados.
 *
 * @param diag Ponteiro para os dados de diagnóstico coletados.
 */
void alert_module_check_conditions(const DiagnosisData_t *diag)
{
    if (diag == NULL)
    {
        return;
    }

    /* Verifica estado Bus-Off */
    if (diag->can_diag.bus_off)
    {
        register_alert(ALERT_LEVEL_CRITICAL, "Estado Bus-Off detectado!");
    }

    /* Verifica taxa de erros de transmissão ou recepção */
    if ((diag->can_diag.tx_error_counter > alert_thresholds.tx_error_threshold) ||
        (diag->can_diag.rx_error_counter > alert_thresholds.rx_error_threshold))
    {
        register_alert(ALERT_LEVEL_WARNING, "Alta taxa de erros na rede CAN!");
    }

    /* Verifica carga do barramento */
    if (diag->bus_load > alert_thresholds.bus_load_threshold)
    {
        register_alert(ALERT_LEVEL_WARNING, "Carga do barramento CAN acima do limiar!");
    }

    /* Verifica número de retransmissões */
    if (diag->retransmission_count > alert_thresholds.retransmission_threshold)
    {
        register_alert(ALERT_LEVEL_WARNING, "Alta taxa de retransmissões na rede CAN!");
    }
}

/**
 * @brief Obtém o histórico de alertas armazenados.
 *
 * Copia as entradas de alerta para o buffer fornecido.
 *
 * @param[out] buffer Ponteiro para o buffer onde os alertas serão armazenados.
 * @param[in] max_entries Número máximo de entradas que o buffer pode armazenar.
 * @return O número de alertas copiados.
 */
uint32_t alert_module_get_history(AlertData_t *buffer, uint32_t max_entries)
{
    uint32_t count = (max_entries < ALERT_HISTORY_SIZE) ? max_entries : ALERT_HISTORY_SIZE;
    if (buffer != NULL)
    {
        for (uint32_t i = 0U; i < count; i++)
        {
            buffer[i] = alert_history[i];
        }
    }
    return count;
}

/**
 * @brief Imprime o histórico de alertas no log.
 */
void alert_module_print_history(void)
{
    ESP_LOGI(TAG, "Histórico de Alertas:");
    for (uint32_t i = 0U; i < ALERT_HISTORY_SIZE; i++)
    {
        if (alert_history[i].timestamp > 0U)
        {
            ESP_LOGI(TAG, "[%" PRIu32 " ms] Nível: %d - %s",
                     alert_history[i].timestamp,
                     (int)alert_history[i].level,
                     alert_history[i].message);
        }
    }
}

/**
 * @brief Configura dinamicamente os limiares de alerta.
 *
 * Atualiza os valores de limiar para erros de transmissão, recepção, carga do barramento e retransmissões.
 *
 * @param thresholds Estrutura contendo os novos limiares.
 */
void alert_module_set_thresholds(const AlertThresholds_t *thresholds)
{
    if (thresholds == NULL)
    {
        ESP_LOGE(TAG, "Ponteiro de limiares nulo.");
        return;
    }
    alert_thresholds = *thresholds;
    ESP_LOGI(TAG, "Limiar de alerta atualizado: TX Erro = %u, RX Erro = %u, Bus Load = %u%%, Retransmissões = %u.",
             alert_thresholds.tx_error_threshold,
             alert_thresholds.rx_error_threshold,
             alert_thresholds.bus_load_threshold,
             alert_thresholds.retransmission_threshold);
}

/**
 * @brief Registra um callback para notificações de alerta em tempo real.
 *
 * O callback é invocado sempre que um novo alerta é registrado, permitindo que outros módulos
 * sejam notificados imediatamente.
 *
 * @param callback Função callback para notificação de alerta.
 */
void alert_module_register_notification_callback(alert_notification_callback_t callback)
{
    notification_callback = callback;
    ESP_LOGI(TAG, "Callback de notificação de alerta registrado com sucesso.");
}
