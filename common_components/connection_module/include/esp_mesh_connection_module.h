/**
 * @file esp_mesh_connection_module.h
 * @brief Módulo de Conectividade Mesh para a rede CAN-ESP.
 *
 * Este arquivo contém as declarações das funções e estruturas utilizadas para o gerenciamento
 * da conectividade mesh, incluindo organização da rede, registro de callbacks para eventos críticos,
 * fallback para configuração de parâmetros e interfaces para consulta de topologia.
 *
 * @note Este módulo está em conformidade com MISRA C:2012 e ISO 11898, sendo adequado para veículos elétricos reais.
 */

#ifndef ESP_MESH_CONNECTION_MODULE_H
#define ESP_MESH_CONNECTION_MODULE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Valores padrão para a rede mesh */
#define MESH_DEFAULT_CHANNEL               1U
#define MESH_DEFAULT_MAX_RETRY             5U
#define MESH_DEFAULT_RECONNECT_DELAY_MS    1000U

/* Valores padrão para o mesh AP (softAP) */
#define MESH_AP_DEFAULT_MAX_CONNECTION         8U
#define MESH_AP_DEFAULT_NONMESH_MAX_CONNECTION 4U
#define MESH_AP_DEFAULT_PASSWORD               "mesh_default"
#define MESH_AP_DEFAULT_AUTHMODE               3U  /* Exemplo: WPA2-PSK */

#define MAX_FILENAME_LENGTH               256U

/* Fallback para configuração do roteador (softAP) */
#define FALLBACK_ROUTER_SSID              "Fallback_SSID"
#define FALLBACK_ROUTER_PASSWORD          "Fallback_Password"
#define FALLBACK_ROUTER_CHANNEL           1U
#define FALLBACK_ROUTER_AUTHMODE          3U  /* Exemplo: WPA2-PSK */

/**
 * @brief Identificadores de eventos críticos da rede mesh.
 */
typedef enum
{
    MESH_EVENT_STARTED = 0,         /**< Rede mesh iniciada */
    MESH_EVENT_STOPPED,             /**< Rede mesh parada */
    MESH_EVENT_DISCONNECTED,        /**< Rede mesh desconectada */
    MESH_EVENT_PARENT_CONNECTED,    /**< Conexão estabelecida com o nó pai */
    MESH_EVENT_ROOT_SWITCHED,       /**< Troca do nó root da rede mesh */
    MESH_EVENT_NEIGHBOR_CHANGE,     /**< Alteração na vizinhança mesh */
    MESH_EVENT_MAX                  /**< Número total de eventos */
} mesh_event_id_t;

/**
 * @brief Estrutura contendo os parâmetros de configuração da rede mesh.
 */
typedef struct
{
    char mesh_id[32];                /**< Identificador da rede mesh */
    uint8_t channel;                 /**< Canal de operação da rede mesh */
    uint8_t max_retry;               /**< Número máximo de tentativas de reconexão */
    uint32_t reconnection_delay_ms;  /**< Delay inicial para reconexão (em ms) */
    bool auto_reconnect;             /**< Habilita reconexão automática */
    bool self_organized;             /**< Define se a rede é auto-organizada */
    /* Parâmetros do mesh AP (softAP) */
    char router_ssid[32];            /**< SSID do roteador */
    char router_password[64];        /**< Senha do roteador */
    uint8_t router_channel;          /**< Canal do roteador */
    uint8_t router_authmode;         /**< Modo de autenticação do roteador */
    /* Parâmetros adicionais do mesh AP */
    uint8_t mesh_ap_max_connection;         /**< Máximo de conexões mesh */
    uint8_t mesh_ap_nonmesh_max_connection;   /**< Máximo de conexões não mesh */
    char mesh_ap_password[64];                /**< Senha do mesh AP */
    uint8_t mesh_ap_authmode;                 /**< Modo de autenticação do mesh AP */
} esp_mesh_config_params_t;

/**
 * @brief Estrutura com informações de topologia da rede mesh.
 */
typedef struct
{
    char parent_id[32];                /**< Identificador do nó pai atual */
    uint8_t neighbor_count;            /**< Número de nós vizinhos conectados */
    char neighbor_ids[8][32];          /**< Lista de identificadores dos nós vizinhos (máximo 8) */
} esp_mesh_topology_info_t;

/**
 * @brief Tipo de callback para notificação de eventos críticos da rede mesh.
 *
 * @param event Código do evento (conforme ::mesh_event_id_t).
 * @param event_data Ponteiro para dados associados ao evento.
 */
typedef void (*esp_mesh_event_callback_t)(int event, void *event_data);

/**
 * @brief Inicializa o módulo de conectividade mesh.
 *
 * Lê os parâmetros de configuração, define o mesh_id (usando o MAC se necessário),
 * inicializa o ESP‑MESH, registra o handler de eventos e cria o event group.
 *
 * @return true se a inicialização for bem-sucedida, false caso contrário.
 */
bool esp_mesh_connection_module_init(void);

/**
 * @brief Inicia o serviço de conectividade mesh.
 *
 * Inicia a rede mesh e cria a tarefa de monitoramento para verificar a saúde da conexão.
 *
 * @return true se o serviço for iniciado com sucesso, false caso contrário.
 */
bool esp_mesh_connection_module_start(void);

/**
 * @brief Reinicializa a conexão mesh.
 *
 * Reinicia a rede mesh para aplicar novas configurações ou recuperar a conexão após uma queda.
 *
 * @return true se a reinicialização for bem-sucedida, false caso contrário.
 */
bool esp_mesh_connection_module_reconnect(void);

/**
 * @brief Define os parâmetros de configuração da rede mesh.
 *
 * Atualiza a configuração, persiste as novas configurações na NVS, notifica módulos interessados
 * e reinicializa a rede mesh para aplicar as mudanças.
 *
 * @param params Ponteiro para a estrutura com os novos parâmetros.
 * @return true se a configuração for atualizada com sucesso, false caso contrário.
 */
bool esp_mesh_connection_module_set_config(const esp_mesh_config_params_t *params);

/**
 * @brief Retorna a configuração atual da rede mesh.
 *
 * Copia os parâmetros atuais para a estrutura fornecida.
 *
 * @param[out] params Ponteiro para a estrutura que receberá os parâmetros.
 * @return true se a operação for bem-sucedida, false caso contrário.
 */
bool esp_mesh_connection_module_get_config(esp_mesh_config_params_t *params);

/**
 * @brief Atualiza a configuração do roteador (softAP) utilizando os parâmetros do módulo Wi‑Fi.
 *
 * Se a obtenção da configuração via wifi_connection_module_get_config() falhar, utiliza valores de fallback.
 *
 * @return true se a configuração for atualizada com sucesso, false caso contrário.
 */
bool esp_mesh_connection_module_update_router_config(void);

/**
 * @brief Define a organização da rede mesh.
 *
 * Chama a API esp_mesh_set_self_organized() para definir se a rede deve ser auto-organizada.
 *
 * @param self_organized true para rede auto-organizada, false caso contrário.
 * @return true se a operação for bem-sucedida, false caso contrário.
 */
bool esp_mesh_connection_module_set_network_organization(bool self_organized);

/**
 * @brief Registra um callback para eventos críticos da rede mesh.
 *
 * Permite que outros módulos se inscrevam para receber notificações de eventos (ex.: reconexão, falhas, status).
 *
 * @param callback Ponteiro para a função de callback.
 * @return true se o callback for registrado com sucesso, false caso contrário.
 */
bool esp_mesh_connection_module_register_callback(esp_mesh_event_callback_t callback);

/**
 * @brief Remove um callback registrado para eventos críticos da rede mesh.
 *
 * @param callback Ponteiro para a função de callback a ser removido.
 * @return true se o callback for removido com sucesso, false caso contrário.
 */
bool esp_mesh_connection_module_unregister_callback(esp_mesh_event_callback_t callback);

/**
 * @brief Obtém informações de topologia da rede mesh.
 *
 * Preenche a estrutura ::esp_mesh_topology_info_t com informações atuais, incluindo o nó pai e os nós vizinhos.
 *
 * @param[out] topology_info Ponteiro para a estrutura que receberá as informações de topologia.
 * @return true se a operação for bem-sucedida, false caso contrário.
 */
bool esp_mesh_connection_module_get_topology(esp_mesh_topology_info_t *topology_info);

/**
 * @brief Manipulador de eventos da rede mesh.
 *
 * Trata os eventos críticos do ESP‑MESH, implementando ações reais para cada evento,
 * tais como atualização do estado interno, gerenciamento de reconexão e notificação aos módulos.
 *
 * @param arg Parâmetro não utilizado.
 * @param event_base Base do evento.
 * @param event_id Identificador do evento (conforme ::mesh_event_id_t).
 * @param event_data Dados associados ao evento.
 * @return int Código de status (0 para sucesso).
 */
int esp_mesh_connection_module_event_handler(void *arg, int event_base, int event_id, void *event_data);

/**
 * @brief Retorna o status atual da rede mesh.
 *
 * Retorna informações básicas sobre o estado da rede, como se o nó atual é root.
 *
 * @return uint32_t Código ou bitmask representando o status.
 */
uint32_t esp_mesh_connection_module_get_status(void);

#ifdef __cplusplus
}
#endif

#endif /* ESP_MESH_CONNECTION_MODULE_H */
