/**
 * @file monitor_ecu.h
 * @brief Declarações públicas para a ECU de Monitoramento e Diagnóstico do projeto CAN-ESP.
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

#ifndef MONITOR_ECU_H
#define MONITOR_ECU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * @brief Inicializa a ECU de Monitoramento e Diagnóstico (nó raiz) do projeto CAN-ESP.
 *
 * Inicializa os módulos de conectividade (Wi-Fi, MQTT, ESP-MESH), Routing, OTA, SD Storage,
 * Diagnosis, Logger e Alert, e inicia as tasks responsáveis pelo fluxo:
 * - OTA (com retry/rollback)
 * - Comunicação (atualização de rotas)
 * - Atualização dinâmica dos parâmetros
 * - Aquisição de mensagens CAN
 * - Aquisição e persistência dos dados diagnósticos (conforme critérios definidos)
 *
 * @return true se a inicialização for bem-sucedida; false, caso contrário.
 */
bool monitor_ecu_init(void);

#ifdef __cplusplus
}
#endif

#endif /* MONITOR_ECU_H */
