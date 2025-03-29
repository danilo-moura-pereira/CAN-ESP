/**
 * @file routing_module.h
 * @brief Módulo de Roteamento para o projeto CAN-ESP.
 *
 * Este módulo gerencia as tabelas de roteamento e de vizinhança, permitindo a inserção, atualização e remoção
 * de entradas na tabela de roteamento, o encaminhamento de mensagens entre as ECUs (suportando unicast, multicast
 * e broadcast), o recebimento seguro de mensagens e a atualização automática das tabelas com base em eventos críticos
 * recebidos do esp_mesh_connection_module, tais como MESH_EVENT_PARENT_CONNECTED, MESH_EVENT_NEIGHBOR_CHANGE e
 * MESH_EVENT_ROOT_SWITCHED.
 *
 * O módulo implementa estratégias de fallback e repetição em caso de falha no roteamento e disponibiliza APIs para
 * atualização dinâmica das configurações, que são persistidas no arquivo "config.ini" utilizando o sd_storage_module.
 *
 * Mecanismos de sincronização (mutexes, event groups e filas de eventos) e tasks dedicadas para o envio e recepção
 * de mensagens são empregados para garantir acesso seguro e processamento assíncrono dos dados em ambiente multitarefa.
 *
 * @note Este módulo está em conformidade com MISRA C:2012 e ISO 11898, sendo adequado para veículos elétricos reais.
 */

#ifndef ROUTING_MODULE_H
#define ROUTING_MODULE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAX_FILENAME_LENGTH
#define MAX_FILENAME_LENGTH 256U
#endif

/** @defgroup ROUTING_MODULE Eventos do Roteamento
 *  @{
 */
#define ROUTING_EVENT_TABLE_UPDATED         0U   /**< Tabela de roteamento atualizada */
#define ROUTING_EVENT_NEIGHBOR_TABLE_UPDATED  1U   /**< Tabela de vizinhança atualizada */
#define ROUTING_EVENT_ROUTE_FAILURE           2U   /**< Falha no encaminhamento de mensagem */
#define ROUTING_EVENT_MESSAGE_RECEIVED        3U   /**< Mensagem recebida */
/** @} */

/** @defgroup MESSAGE_MODES Modos de Envio de Mensagens
 *  @{
 */
#define ROUTING_MODE_UNICAST   0U /**< Envio para um único destino */
#define ROUTING_MODE_MULTICAST 1U /**< Envio para um grupo de destinos */
#define ROUTING_MODE_BROADCAST 2U /**< Envio para todos os nós na rede */
/** @} */

/** Máximo de entradas na tabela de roteamento */
#define MAX_ROUTING_TABLE_ENTRIES  16U
/** Máximo de entradas na tabela de vizinhança */
#define MAX_NEIGHBOR_TABLE_ENTRIES 8U

/**
 * @brief Estrutura de uma entrada na tabela de roteamento.
 */
typedef struct
{
    char dest_id[32];    /**< Identificador do nó destino */
    char next_hop[32];   /**< Identificador do próximo salto */
    uint8_t cost;        /**< Métrica de custo para a rota */
    uint32_t timestamp;  /**< Timestamp da última atualização */
} routing_table_entry_t;

/**
 * @brief Estrutura da tabela de roteamento.
 */
typedef struct
{
    routing_table_entry_t entries[MAX_ROUTING_TABLE_ENTRIES]; /**< Array de entradas de roteamento */
    uint8_t count;                                            /**< Número atual de entradas */
} routing_table_t;

/**
 * @brief Estrutura de uma entrada na tabela de vizinhança.
 */
typedef struct
{
    char neighbor_id[32];  /**< Identificador do nó vizinho */
    int8_t rssi;           /**< Força do sinal (RSSI) */
    uint8_t link_quality;  /**< Qualidade do link */
} neighbor_table_entry_t;

/**
 * @brief Estrutura da tabela de vizinhança.
 */
typedef struct
{
    neighbor_table_entry_t entries[MAX_NEIGHBOR_TABLE_ENTRIES]; /**< Array de entradas de vizinhança */
    uint8_t count;                                             /**< Número atual de vizinhos */
} neighbor_table_t;

/**
 * @brief Estrutura de configuração para o módulo de roteamento.
 */
typedef struct
{
    uint8_t default_cost;     /**< Valor de custo padrão para rotas criadas automaticamente */
    uint8_t retry_count;      /**< Número de tentativas de repetição em caso de falha no roteamento */
    uint32_t retry_delay_ms;  /**< Intervalo entre tentativas de repetição (em ms) */
} routing_config_t;

/**
 * @brief Estrutura que contém uma mensagem recebida.
 */
typedef struct {
    char src_id[32];      /**< Identificador do nó de origem */
    uint16_t length;      /**< Comprimento da mensagem */
    uint8_t data[256];    /**< Dados da mensagem (tamanho máximo de 256 bytes) */
} routing_received_message_t;

/**
 * @brief Estrutura que contém uma mensagem a ser enviada.
 */
typedef struct {
    char dest_id[32];     /**< Identificador do nó destino (para unicast/multicast; vazio para broadcast) */
    uint8_t data[256];    /**< Dados da mensagem */
    uint16_t length;      /**< Comprimento dos dados (em bytes) */
    uint8_t mode;         /**< Modo de envio (veja @ref MESSAGE_MODES) */
} routing_send_message_item_t;

/**
 * @brief Tipo de callback para eventos do módulo de roteamento.
 *
 * @param event Código do evento (veja as definições ROUTING_EVENT_*).
 * @param data Ponteiro para dados específicos do evento. No caso de uma mensagem recebida, o callback
 *             receberá um ponteiro para ::routing_received_message_t.
 */
typedef void (*routing_event_callback_t)(uint8_t event, void *data);

/**
 * @brief Constantes para eventos mesh recebidos do esp_mesh_connection_module.
 */
#define MESH_EVENT_NEIGHBOR_CHANGE   5U  /**< Evento de alteração na vizinhança */
#define MESH_EVENT_PARENT_CONNECTED  6U  /**< Evento de conexão com o nó pai */
#define MESH_EVENT_ROOT_SWITCHED     7U  /**< Evento de troca de nó root */

/**
 * @brief Inicializa o módulo de roteamento.
 *
 * Cria os mutexes, o event group e as filas de eventos para envio, recepção e processamento,
 * inicializa as tabelas e carrega as configurações persistidas do arquivo "config.ini".
 *
 * @return true se a inicialização for bem-sucedida, false caso contrário.
 */
bool routing_module_init(void);

/**
 * @brief Inicia o módulo de roteamento.
 *
 * Cria tasks dedicadas para o processamento de eventos mesh, envio e recepção de mensagens.
 *
 * @return true se o módulo iniciar com sucesso, false caso contrário.
 */
bool routing_module_start(void);

/**
 * @brief Atualiza a tabela de vizinhança com base nas informações de topologia.
 *
 * Atualiza a tabela de vizinhança com os dados fornecidos (por exemplo, via evento MESH_EVENT_NEIGHBOR_CHANGE)
 * e dispara o recálculo das rotas.
 *
 * @param topology_info Ponteiro para a estrutura ::neighbor_table_t contendo as informações de topologia.
 * @return true se a atualização for bem-sucedida, false caso contrário.
 */
bool routing_module_update_topology(const neighbor_table_t *topology_info);

/**
 * @brief Recalcula a tabela de roteamento com base na tabela de vizinhança atual.
 *
 * Executa o algoritmo de roteamento, atualizando a tabela de roteamento.
 *
 * @return true se o recálculo for bem-sucedido, false caso contrário.
 */
bool routing_module_recalculate_routes(void);

/**
 * @brief Insere uma nova entrada na tabela de roteamento.
 *
 * Adiciona uma entrada na tabela, desde que não exista uma para o mesmo destino.
 *
 * @param entry Ponteiro para a nova entrada.
 * @return true se a inserção for bem-sucedida, false caso contrário.
 */
bool routing_module_insert_route(const routing_table_entry_t *entry);

/**
 * @brief Atualiza uma entrada existente na tabela de roteamento.
 *
 * Atualiza os dados da entrada identificada pelo destino.
 *
 * @param entry Ponteiro para a entrada com os dados atualizados.
 * @return true se a atualização for bem-sucedida, false caso contrário.
 */
bool routing_module_update_route(const routing_table_entry_t *entry);

/**
 * @brief Remove uma entrada da tabela de roteamento.
 *
 * Remove a entrada correspondente ao destino informado.
 *
 * @param dest_id Identificador do nó destino.
 * @return true se a remoção for bem-sucedida, false caso contrário.
 */
bool routing_module_remove_route(const char *dest_id);

/**
 * @brief Consulta a tabela de roteamento atual.
 *
 * Copia a tabela interna para a estrutura fornecida.
 *
 * @param table Ponteiro para a estrutura ::routing_table_t que receberá os dados.
 * @return true se a operação for bem-sucedida, false caso contrário.
 */
bool routing_module_get_routing_table(routing_table_t *table);

/**
 * @brief Consulta a tabela de vizinhança atual.
 *
 * Copia a tabela interna para a estrutura fornecida.
 *
 * @param table Ponteiro para a estrutura ::neighbor_table_t que receberá os dados.
 * @return true se a operação for bem-sucedida, false caso contrário.
 */
bool routing_module_get_neighbor_table(neighbor_table_t *table);

/**
 * @brief Envia uma mensagem utilizando o módulo de roteamento.
 *
 * Encaminha uma mensagem para o destino especificado utilizando o modo indicado.
 * Para unicast e multicast, o parâmetro dest_id é utilizado; para broadcast, ele é ignorado.
 * Em caso de falha, tenta recalcular as rotas e repetir o envio conforme os parâmetros configurados.
 *
 * @param dest_id Identificador do nó destino.
 * @param data Ponteiro para os dados da mensagem.
 * @param length Comprimento dos dados (em bytes).
 * @param mode Modo de envio (ROUTING_MODE_UNICAST, ROUTING_MODE_MULTICAST ou ROUTING_MODE_BROADCAST).
 * @return true se a mensagem for enfileirada com sucesso, false caso contrário.
 */
bool routing_module_send_message(const char *dest_id, const uint8_t *data, uint16_t length, uint8_t mode);

/**
 * @brief Recebe uma mensagem utilizando o módulo de roteamento.
 *
 * Enfileira a mensagem recebida para processamento pela task dedicada de recepção.
 *
 * @param src_id Identificador do nó de origem.
 * @param data Ponteiro para os dados da mensagem.
 * @param length Comprimento dos dados (em bytes). O tamanho máximo permitido é 256 bytes.
 * @return true se a mensagem for enfileirada com sucesso, false caso contrário.
 */
bool routing_module_receive_message(const char *src_id, const uint8_t *data, uint16_t length);

/**
 * @brief Enfileira um evento mesh recebido do esp_mesh_connection_module.
 *
 * Cria um item de evento e o envia para a fila, sinalizando através do event group.
 *
 * @param event_id Código do evento mesh.
 * @param event_data Ponteiro para dados específicos do evento.
 * @return true se o evento for enfileirado com sucesso, false caso contrário.
 */
bool routing_module_queue_mesh_event(uint8_t event_id, const void *event_data);

/**
 * @brief Processa um evento mesh recebido do esp_mesh_connection_module.
 *
 * Este método é chamado pela task de eventos para processar itens da fila.
 *
 * @param event_id Código do evento mesh.
 * @param event_data Ponteiro para dados específicos do evento.
 * @return true se o evento for processado com sucesso, false caso contrário.
 */
bool routing_module_process_mesh_event(uint8_t event_id, const void *event_data);

/**
 * @brief Atualiza dinamicamente as configurações do módulo de roteamento.
 *
 * Permite ajustar parâmetros como o custo padrão, número de tentativas de fallback e intervalo entre tentativas,
 * salvando-os no arquivo "config.ini" para persistência.
 *
 * @param config Ponteiro para a estrutura ::routing_config_t com as novas configurações.
 * @return true se as configurações forem atualizadas e salvas com sucesso, false caso contrário.
 */
bool routing_module_set_config(const routing_config_t *config);

/**
 * @brief Obtém as configurações atuais do módulo de roteamento.
 *
 * Copia as configurações atuais para a estrutura fornecida.
 *
 * @param config Ponteiro para a estrutura ::routing_config_t que receberá as configurações.
 * @return true se a operação for bem-sucedida, false caso contrário.
 */
bool routing_module_get_config(routing_config_t *config);

/**
 * @brief Registra um callback para eventos do módulo de roteamento.
 *
 * Permite que outros módulos se inscrevam para receber notificações sobre atualizações nas tabelas e sobre erros.
 *
 * @param callback Ponteiro para a função de callback.
 * @return true se o callback for registrado com sucesso, false caso contrário.
 */
bool routing_module_register_callback(routing_event_callback_t callback);

/**
 * @brief Remove um callback registrado para eventos do módulo de roteamento.
 *
 * @param callback Ponteiro para a função de callback a ser removido.
 * @return true se o callback for removido com sucesso, false caso contrário.
 */
bool routing_module_unregister_callback(routing_event_callback_t callback);

#ifdef __cplusplus
}
#endif

#endif /* ROUTING_MODULE_H */
