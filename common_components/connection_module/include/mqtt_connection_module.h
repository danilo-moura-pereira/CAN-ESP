/**
 * @file mqtt_connection_module.h
 * @brief Cabeçalho do Módulo de Conectividade MQTT para a rede CAN‑ESP.
 *
 * Este módulo gerencia a conexão MQTT para o projeto CAN‑ESP, permitindo a publicação e
 * recepção de mensagens via broker MQTT. A configuração (broker URI, client ID, tópico, QoS,
 * keepalive, etc.) pode ser atualizada dinamicamente. Desenvolvido para aplicações em redes CAN
 * de veículos elétricos reais, em conformidade com MISRA C:2012 e ISO 11898.
 *
 * @note Este módulo depende do componente wifi_connection_module para a conectividade Wi‑Fi.
 *
 * @author 
 * @date 
 */

#ifndef MQTT_CONNECTION_MODULE_H
#define MQTT_CONNECTION_MODULE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Parâmetros de configuração MQTT.
 */
typedef struct {
    char broker_uri[128];   /**< URI do broker MQTT (ex.: "mqtt://broker.hivemq.com") */
    char client_id[64];     /**< Identificador do cliente MQTT */
    char topic[64];         /**< Tópico MQTT para publicação e assinatura */
    uint32_t qos;           /**< Nível de QoS (0, 1 ou 2) */
    uint32_t keepalive;     /**< Intervalo keepalive em segundos */
} mqtt_config_params_t;

/**
 * @brief Inicializa o módulo MQTT.
 *
 * Configura e inicia o cliente MQTT utilizando os parâmetros atuais. Se necessário, a
 * configuração pode ser carregada do arquivo de configuração (via sd_storage_module).
 *
 * @return true se a inicialização for bem-sucedida, false caso contrário.
 */
bool mqtt_connection_module_init(void);

/**
 * @brief Finaliza a conexão MQTT e libera os recursos alocados.
 *
 * @return true se a operação for bem-sucedida, false caso contrário.
 */
bool mqtt_connection_module_deinit(void);

/**
 * @brief Publica uma mensagem MQTT.
 *
 * Publica a mensagem no tópico configurado com o nível de QoS definido.
 *
 * @param message Mensagem a ser publicada.
 * @return true se a mensagem for publicada com sucesso, false caso contrário.
 */
bool mqtt_connection_module_publish(const char *message);

/**
 * @brief Registra um callback para a recepção de mensagens MQTT.
 *
 * O callback será invocado sempre que uma mensagem for recebida no tópico assinado.
 *
 * @param callback Ponteiro para a função callback que recebe o tópico e os dados.
 */
void mqtt_connection_module_register_message_callback(void (*callback)(const char *topic, const char *data));

/**
 * @brief Atualiza a configuração MQTT dinamicamente.
 *
 * Permite alterar os parâmetros de configuração MQTT em tempo real.
 *
 * @param config Ponteiro para a estrutura contendo a nova configuração.
 * @return true se a atualização for bem-sucedida, false caso contrário.
 */
bool mqtt_connection_module_update_config(const mqtt_config_params_t *config);

/**
 * @brief Retorna a configuração MQTT atualmente em uso.
 *
 * Copia os parâmetros atuais para a estrutura fornecida.
 *
 * @param[out] config Ponteiro para a estrutura que receberá a configuração.
 * @return true se a operação for bem-sucedida, false caso contrário.
 */
bool mqtt_connection_module_get_config(mqtt_config_params_t *config);

#ifdef __cplusplus
}
#endif

#endif /* MQTT_CONNECTION_MODULE_H */
