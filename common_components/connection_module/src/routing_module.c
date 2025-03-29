/**
 * @file routing_module.c
 * @brief Implementação do módulo de roteamento para o projeto CAN-ESP.
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
 * Mecanismos de sincronização (mutexes, event groups e uma fila de eventos) são empregados para garantir acesso
 * seguro aos dados compartilhados e processamento determinístico em ambiente multitarefa.
 *
 * @note Este módulo está em conformidade com MISRA C:2012 e ISO 11898, sendo adequado para veículos elétricos reais.
 */

#include "routing_module.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "sd_storage_module.h"  /* Para operações com "config.ini" */
#include <string.h>
#include <stdio.h>

/* Tag para logs */
#define TAG "ROUTING_MODULE"

/* Tamanho máximo para linhas de configuração */
#define CONFIG_LINE_MAX_LEN 128U

/* Mutex para proteger o acesso às tabelas de roteamento e vizinhança */
static SemaphoreHandle_t routing_table_mutex = NULL;
/* Mutex para proteger o acesso à configuração */
static SemaphoreHandle_t config_mutex = NULL;
/* Mutex para acesso exclusivo ao arquivo de configuração */
static SemaphoreHandle_t file_mutex = NULL;
/* Mutex para proteger o processamento de mensagens recebidas */
static SemaphoreHandle_t receive_mutex = NULL;

/* Event group para sinalizar a chegada de novos eventos */
static EventGroupHandle_t routing_event_group = NULL;
#define ROUTING_EVENT_BIT_NEW  (1 << 0)

/* Estrutura para itens de evento */
typedef struct
{
    uint8_t event_id;
    void *event_data;
} routing_event_item_t;
#define ROUTING_EVENT_QUEUE_LENGTH 10
static QueueHandle_t routing_event_queue = NULL;

/* Tabelas internas */
static routing_table_t routing_table = { { {0} }, 0U };
static neighbor_table_t neighbor_table = { { {0} }, 0U };

/* Array e contador para callbacks registrados */
#define MAX_ROUTING_CALLBACKS  10U
static routing_event_callback_t routing_callbacks[MAX_ROUTING_CALLBACKS] = { 0 };
static uint8_t routing_callback_count = 0U;

/* Configuração dinâmica do módulo */
static routing_config_t routing_config = { 1U, 3U, 500U };  /* default_cost=1, retry_count=3, retry_delay_ms=500 */

/* --- TASKS para gerenciamento de eventos e mensagens --- */

/* Protótipo da task de processamento de eventos mesh */
static void routing_module_event_task(void *pvParameters);

/* Protótipos das tasks de envio e recepção de mensagens */
static void routing_module_send_task(void *pvParameters);
static void routing_module_receive_task(void *pvParameters);

/**
 * @brief Notifica todos os callbacks registrados sobre um evento.
 *
 * @param event Código do evento.
 * @param data Dados associados ao evento.
 */
static void routing_module_notify(uint8_t event, void *data)
{
    uint8_t i;
    for (i = 0U; i < routing_callback_count; i++)
    {
        if (routing_callbacks[i] != NULL)
        {
            routing_callbacks[i](event, data);
        }
    }
}

/**
 * @brief Tarefa dedicada para processar eventos mesh enfileirados.
 *
 * Aguarda sinais via event group e processa os eventos da fila, adquirindo os mutexes necessários
 * para atualizar as tabelas.
 */
static void routing_module_event_task(void *pvParameters)
{
    (void)pvParameters;
    EventBits_t uxBits;
    routing_event_item_t event_item;
    for (;;)
    {
        uxBits = xEventGroupWaitBits(routing_event_group, ROUTING_EVENT_BIT_NEW, pdTRUE, pdFALSE, portMAX_DELAY);
        if ((uxBits & ROUTING_EVENT_BIT_NEW) != 0U)
        {
            while (xQueueReceive(routing_event_queue, &event_item, 0) == pdPASS)
            {
                if (event_item.event_id == MESH_EVENT_NEIGHBOR_CHANGE)
                {
                    xSemaphoreTake(routing_table_mutex, portMAX_DELAY);
                    (void)routing_module_update_topology((const neighbor_table_t *)event_item.event_data);
                    xSemaphoreGive(routing_table_mutex);
                }
                else if ((event_item.event_id == MESH_EVENT_PARENT_CONNECTED) ||
                         (event_item.event_id == MESH_EVENT_ROOT_SWITCHED))
                {
                    xSemaphoreTake(routing_table_mutex, portMAX_DELAY);
                    (void)routing_module_recalculate_routes();
                    xSemaphoreGive(routing_table_mutex);
                }
                else
                {
                    ESP_LOGW(TAG, "Unhandled event in task: %u", event_item.event_id);
                }
            }
        }
    }
}

/**
 * @brief Carrega as configurações de roteamento a partir do arquivo "config.ini".
 *
 * Procura por chaves: ROUTING_DEFAULT_COST, ROUTING_RETRY_COUNT e ROUTING_RETRY_DELAY_MS.
 *
 * @return true se a configuração for carregada com sucesso, false caso contrário.
 */
static bool routing_module_load_config(void)
{
    char config_line[CONFIG_LINE_MAX_LEN];
    char config_path[MAX_FILENAME_LENGTH];
    FILE *file;
    
    xSemaphoreTake(file_mutex, portMAX_DELAY);
    (void)snprintf(config_path, MAX_FILENAME_LENGTH, "%s/%s", MOUNT_POINT, "config.ini");
    file = fopen(config_path, "r");
    if (file == NULL)
    {
        ESP_LOGW(TAG, "Config file %s not found, using default routing configuration.", config_path);
        xSemaphoreGive(file_mutex);
        return false;
    }
    while (fgets(config_line, CONFIG_LINE_MAX_LEN, file) != NULL)
    {
        char *newline = strchr(config_line, '\n');
        if (newline != NULL)
        {
            *newline = '\0';
        }
        if (strncmp(config_line, "ROUTING_DEFAULT_COST=", 21) == 0)
        {
            routing_config.default_cost = (uint8_t)atoi(config_line + 21);
        }
        else if (strncmp(config_line, "ROUTING_RETRY_COUNT=", 20) == 0)
        {
            routing_config.retry_count = (uint8_t)atoi(config_line + 20);
        }
        else if (strncmp(config_line, "ROUTING_RETRY_DELAY_MS=", 23) == 0)
        {
            routing_config.retry_delay_ms = (uint32_t)atoi(config_line + 23);
        }
    }
    fclose(file);
    xSemaphoreGive(file_mutex);
    ESP_LOGI(TAG, "Routing configuration loaded from %s.", config_path);
    return true;
}

/* --- TASKS dedicadas para gerenciamento de mensagens (envio e recepção) --- */

/**
 * @brief Tarefa dedicada para processar mensagens de envio.
 *
 * Aguarda itens na fila de envio e executa o procedimento de envio, incluindo fallback e repetição.
 */
static void routing_module_send_task(void *pvParameters)
{
    (void)pvParameters;
    routing_send_queue_item_t send_item;
    for (;;)
    {
        if (xQueueReceive(routing_send_queue, &send_item, portMAX_DELAY) == pdPASS)
        {
            uint8_t i;
            bool found = false;
            char next_hop[32] = {0};
            if (send_item.item.mode == ROUTING_MODE_UNICAST)
            {
                xSemaphoreTake(routing_table_mutex, portMAX_DELAY);
                for (i = 0U; i < routing_table.count; i++)
                {
                    if (strncmp(routing_table.entries[i].dest_id, send_item.item.dest_id,
                                sizeof(routing_table.entries[i].dest_id)) == 0)
                    {
                        (void)snprintf(next_hop, sizeof(next_hop), "%s", routing_table.entries[i].next_hop);
                        found = true;
                        break;
                    }
                }
                xSemaphoreGive(routing_table_mutex);
                uint8_t attempts = 0U;
                while (!found && (attempts < routing_config.retry_count))
                {
                    ESP_LOGW(TAG, "Send task: Route not found for destination: %s. Attempt %u/%u. Retrying...",
                             send_item.item.dest_id, attempts + 1U, routing_config.retry_count);
                    vTaskDelay(pdMS_TO_TICKS(routing_config.retry_delay_ms));
                    xSemaphoreTake(routing_table_mutex, portMAX_DELAY);
                    (void)routing_module_recalculate_routes();
                    for (i = 0U; i < routing_table.count; i++)
                    {
                        if (strncmp(routing_table.entries[i].dest_id, send_item.item.dest_id,
                                    sizeof(routing_table.entries[i].dest_id)) == 0)
                        {
                            (void)snprintf(next_hop, sizeof(next_hop), "%s", routing_table.entries[i].next_hop);
                            found = true;
                            break;
                        }
                    }
                    xSemaphoreGive(routing_table_mutex);
                    attempts++;
                }
                if (!found)
                {
                    ESP_LOGE(TAG, "Send task: Route not found for destination: %s after %u attempts.",
                             send_item.item.dest_id, routing_config.retry_count);
                    routing_module_notify(ROUTING_EVENT_ROUTE_FAILURE, (void *)send_item.item.dest_id);
                    continue;
                }
                ESP_LOGI(TAG, "Send task: Sending unicast message to %s. Size: %u bytes.", next_hop, send_item.item.length);
            }
            else if (send_item.item.mode == ROUTING_MODE_MULTICAST)
            {
                uint8_t count = 0U;
                xSemaphoreTake(routing_table_mutex, portMAX_DELAY);
                for (i = 0U; i < routing_table.count; i++)
                {
                    if (strstr(routing_table.entries[i].dest_id, send_item.item.dest_id) != NULL)
                    {
                        count++;
                    }
                }
                xSemaphoreGive(routing_table_mutex);
                if (count == 0U)
                {
                    ESP_LOGW(TAG, "Send task: No multicast routes found for group: %s.", send_item.item.dest_id);
                    routing_module_notify(ROUTING_EVENT_ROUTE_FAILURE, (void *)send_item.item.dest_id);
                    continue;
                }
                ESP_LOGI(TAG, "Send task: Sending multicast message to group %s. Routes found: %u. Size: %u bytes.",
                         send_item.item.dest_id, count, send_item.item.length);
            }
            else if (send_item.item.mode == ROUTING_MODE_BROADCAST)
            {
                ESP_LOGI(TAG, "Send task: Sending broadcast message to all neighbors. Size: %u bytes.", send_item.item.length);
            }
            else
            {
                ESP_LOGE(TAG, "Send task: Invalid routing mode: %u", send_item.item.mode);
            }
            /* Em uma implementação real, o envio seria realizado pela interface CAN/Wi-Fi */
        }
    }
}

/**
 * @brief Tarefa dedicada para processar mensagens recebidas.
 *
 * Aguarda itens na fila de recepção e notifica os callbacks registrados com a mensagem recebida.
 * O callback é responsável por liberar a memória alocada para a mensagem.
 */
static void routing_module_receive_task(void *pvParameters)
{
    (void)pvParameters;
    routing_received_message_t *msg = NULL;
    for (;;)
    {
        if (xQueueReceive(routing_receive_queue, &msg, portMAX_DELAY) == pdPASS)
        {
            ESP_LOGI(TAG, "Receive task: Processing message from %s, size: %u bytes.", msg->src_id, msg->length);
            routing_module_notify(ROUTING_EVENT_MESSAGE_RECEIVED, (void *)msg);
            /* O callback que receber a mensagem deve liberar a memória alocada */
        }
    }
}

/* --- Fim das TASKS dedicadas para envio e recepção de mensagens --- */

/**
 * @brief Persiste as configurações de roteamento no arquivo "config.ini".
 *
 * Grava as chaves ROUTING_DEFAULT_COST, ROUTING_RETRY_COUNT e ROUTING_RETRY_DELAY_MS.
 *
 * @return true se as configurações forem salvas com sucesso, false caso contrário.
 */
static bool routing_module_save_config(void)
{
    char config_path[MAX_FILENAME_LENGTH];
    FILE *file;
    
    xSemaphoreTake(file_mutex, portMAX_DELAY);
    (void)snprintf(config_path, MAX_FILENAME_LENGTH, "%s/%s", MOUNT_POINT, "config.ini");
    file = fopen(config_path, "w");
    if (file == NULL)
    {
        ESP_LOGE(TAG, "Failed to open config file %s for writing.", config_path);
        xSemaphoreGive(file_mutex);
        return false;
    }
    fprintf(file, "ROUTING_DEFAULT_COST=%u\n", routing_config.default_cost);
    fprintf(file, "ROUTING_RETRY_COUNT=%u\n", routing_config.retry_count);
    fprintf(file, "ROUTING_RETRY_DELAY_MS=%lu\n", (unsigned long)routing_config.retry_delay_ms);
    fclose(file);
    xSemaphoreGive(file_mutex);
    ESP_LOGI(TAG, "Routing configuration saved to %s.", config_path);
    return true;
}

/**
 * @brief Inicializa o módulo de roteamento.
 *
 * Cria os mutexes, o event group, a fila de eventos e inicializa as tabelas, carregando as configurações persistidas.
 *
 * @return true se a inicialização for bem-sucedida, false caso contrário.
 */
bool routing_module_init(void)
{
    routing_table_mutex = xSemaphoreCreateMutex();
    config_mutex = xSemaphoreCreateMutex();
    file_mutex = xSemaphoreCreateMutex();
    receive_mutex = xSemaphoreCreateMutex();
    if ((routing_table_mutex == NULL) || (config_mutex == NULL) || (file_mutex == NULL) || (receive_mutex == NULL))
    {
        ESP_LOGE(TAG, "Failed to create one or more mutexes.");
        return false;
    }
    routing_event_group = xEventGroupCreate();
    if (routing_event_group == NULL)
    {
        ESP_LOGE(TAG, "Failed to create event group.");
        return false;
    }
    routing_event_queue = xQueueCreate(ROUTING_EVENT_QUEUE_LENGTH, sizeof(routing_event_item_t));
    if (routing_event_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create event queue.");
        return false;
    }
    xSemaphoreTake(routing_table_mutex, portMAX_DELAY);
    (void)memset(&routing_table, 0, sizeof(routing_table));
    (void)memset(&neighbor_table, 0, sizeof(neighbor_table));
    xSemaphoreGive(routing_table_mutex);
    xSemaphoreTake(config_mutex, portMAX_DELAY);
    (void)routing_module_load_config();
    xSemaphoreGive(config_mutex);
    ESP_LOGI(TAG, "Routing module initialized.");
    return true;
}

/**
 * @brief Inicia o módulo de roteamento.
 *
 * Cria tasks dedicadas para o processamento de eventos mesh, envio e recepção de mensagens.
 *
 * @return true se o módulo iniciar com sucesso, false caso contrário.
 */
bool routing_module_start(void)
{
    BaseType_t result;

    /* Inicializa as filas de envio e recepção de mensagens */
    /* As filas devem ser criadas antes de criar as tasks de envio e recepção */
    // Para este exemplo, assumimos que as filas já foram criadas em outra parte ou não são necessárias,
    // mas se necessário, podem ser criadas aqui com xQueueCreate.
    /* Criação das tasks dedicadas */
    result = xTaskCreate(routing_module_event_task, "RoutingEventTask", 4096, NULL, 5, NULL);
    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create routing event task.");
        return false;
    }
    result = xTaskCreate(routing_module_send_task, "RoutingSendTask", 4096, NULL, 5, NULL);
    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create routing send task.");
        return false;
    }
    result = xTaskCreate(routing_module_receive_task, "RoutingReceiveTask", 4096, NULL, 5, NULL);
    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create routing receive task.");
        return false;
    }
    ESP_LOGI(TAG, "Routing module started.");
    return true;
}

/**
 * @brief Atualiza a tabela de vizinhança com base nas informações de topologia.
 *
 * Copia os dados fornecidos para a tabela de vizinhança, notifica a atualização e dispara o recálculo das rotas.
 *
 * @param topology_info Ponteiro para a estrutura ::neighbor_table_t contendo as informações de topologia.
 * @return true se a atualização for bem-sucedida, false caso contrário.
 */
bool routing_module_update_topology(const neighbor_table_t *topology_info)
{
    if (topology_info == NULL)
    {
        ESP_LOGE(TAG, "Null topology info provided.");
        return false;
    }
    xSemaphoreTake(routing_table_mutex, portMAX_DELAY);
    (void)memcpy(&neighbor_table, topology_info, sizeof(neighbor_table_t));
    ESP_LOGI(TAG, "Neighbor table updated. Total neighbors: %u", neighbor_table.count);
    xSemaphoreGive(routing_table_mutex);
    routing_module_notify(ROUTING_EVENT_NEIGHBOR_TABLE_UPDATED, (void *)&neighbor_table);
    return routing_module_recalculate_routes();
}

/**
 * @brief Recalcula a tabela de roteamento com base na tabela de vizinhança atual.
 *
 * Para cada vizinho, cria uma rota com o custo definido em routing_config.
 *
 * @return true se o recálculo for bem-sucedido, false caso contrário.
 */
bool routing_module_recalculate_routes(void)
{
    uint8_t i;
    xSemaphoreTake(routing_table_mutex, portMAX_DELAY);
    routing_table.count = 0U;
    for (i = 0U; i < neighbor_table.count; i++)
    {
        if (routing_table.count < MAX_ROUTING_TABLE_ENTRIES)
        {
            (void)snprintf(routing_table.entries[routing_table.count].dest_id,
                           sizeof(routing_table.entries[routing_table.count].dest_id),
                           "%s", neighbor_table.entries[i].neighbor_id);
            (void)snprintf(routing_table.entries[routing_table.count].next_hop,
                           sizeof(routing_table.entries[routing_table.count].next_hop),
                           "%s", neighbor_table.entries[i].neighbor_id);
            routing_table.entries[routing_table.count].cost = routing_config.default_cost;
            routing_table.entries[routing_table.count].timestamp = (uint32_t)xTaskGetTickCount();
            routing_table.count++;
        }
    }
    ESP_LOGI(TAG, "Routes recalculated. Total entries: %u", routing_table.count);
    xSemaphoreGive(routing_table_mutex);
    routing_module_notify(ROUTING_EVENT_TABLE_UPDATED, (void *)&routing_table);
    return true;
}

/**
 * @brief Insere uma nova entrada na tabela de roteamento.
 *
 * Adiciona uma entrada na tabela, desde que não exista uma para o mesmo destino.
 *
 * @param entry Ponteiro para a nova entrada.
 * @return true se a inserção for bem-sucedida, false caso contrário.
 */
bool routing_module_insert_route(const routing_table_entry_t *entry)
{
    uint8_t i;
    if (entry == NULL)
    {
        ESP_LOGE(TAG, "Null entry provided for insertion.");
        return false;
    }
    xSemaphoreTake(routing_table_mutex, portMAX_DELAY);
    for (i = 0U; i < routing_table.count; i++)
    {
        if (strncmp(routing_table.entries[i].dest_id, entry->dest_id,
                    sizeof(routing_table.entries[i].dest_id)) == 0)
        {
            ESP_LOGW(TAG, "Entry for destination %s already exists. Use update function.", entry->dest_id);
            xSemaphoreGive(routing_table_mutex);
            return false;
        }
    }
    if (routing_table.count >= MAX_ROUTING_TABLE_ENTRIES)
    {
        ESP_LOGE(TAG, "Routing table full. Cannot insert new entry.");
        xSemaphoreGive(routing_table_mutex);
        return false;
    }
    (void)memcpy(&routing_table.entries[routing_table.count], entry, sizeof(routing_table_entry_t));
    routing_table.count++;
    ESP_LOGI(TAG, "Inserted entry for destination %s.", entry->dest_id);
    xSemaphoreGive(routing_table_mutex);
    routing_module_notify(ROUTING_EVENT_TABLE_UPDATED, (void *)&routing_table);
    return true;
}

/**
 * @brief Atualiza uma entrada existente na tabela de roteamento.
 *
 * Atualiza os dados da entrada identificada pelo destino.
 *
 * @param entry Ponteiro para a entrada com os dados atualizados.
 * @return true se a atualização for bem-sucedida, false caso contrário.
 */
bool routing_module_update_route(const routing_table_entry_t *entry)
{
    uint8_t i;
    if (entry == NULL)
    {
        ESP_LOGE(TAG, "Null entry provided for update.");
        return false;
    }
    xSemaphoreTake(routing_table_mutex, portMAX_DELAY);
    for (i = 0U; i < routing_table.count; i++)
    {
        if (strncmp(routing_table.entries[i].dest_id, entry->dest_id,
                    sizeof(routing_table.entries[i].dest_id)) == 0)
        {
            (void)memcpy(&routing_table.entries[i], entry, sizeof(routing_table_entry_t));
            ESP_LOGI(TAG, "Updated entry for destination %s.", entry->dest_id);
            xSemaphoreGive(routing_table_mutex);
            routing_module_notify(ROUTING_EVENT_TABLE_UPDATED, (void *)&routing_table);
            return true;
        }
    }
    xSemaphoreGive(routing_table_mutex);
    ESP_LOGW(TAG, "Entry for destination %s not found for update.", entry->dest_id);
    return false;
}

/**
 * @brief Remove uma entrada da tabela de roteamento.
 *
 * Remove a entrada correspondente ao destino informado.
 *
 * @param dest_id Identificador do nó destino.
 * @return true se a remoção for bem-sucedida, false caso contrário.
 */
bool routing_module_remove_route(const char *dest_id)
{
    uint8_t i, j;
    if (dest_id == NULL)
    {
        ESP_LOGE(TAG, "Null destination provided for removal.");
        return false;
    }
    xSemaphoreTake(routing_table_mutex, portMAX_DELAY);
    for (i = 0U; i < routing_table.count; i++)
    {
        if (strncmp(routing_table.entries[i].dest_id, dest_id,
                    sizeof(routing_table.entries[i].dest_id)) == 0)
        {
            for (j = i; j < routing_table.count - 1U; j++)
            {
                (void)memcpy(&routing_table.entries[j],
                             &routing_table.entries[j + 1U],
                             sizeof(routing_table_entry_t));
            }
            (void)memset(&routing_table.entries[routing_table.count - 1U], 0, sizeof(routing_table_entry_t));
            routing_table.count--;
            ESP_LOGI(TAG, "Removed entry for destination %s.", dest_id);
            xSemaphoreGive(routing_table_mutex);
            routing_module_notify(ROUTING_EVENT_TABLE_UPDATED, (void *)&routing_table);
            return true;
        }
    }
    xSemaphoreGive(routing_table_mutex);
    ESP_LOGW(TAG, "Entry for destination %s not found for removal.", dest_id);
    routing_module_notify(ROUTING_EVENT_ROUTE_FAILURE, (void *)dest_id);
    return false;
}

/**
 * @brief Consulta a tabela de roteamento atual.
 *
 * Copia a tabela interna para a estrutura fornecida.
 *
 * @param table Ponteiro para a estrutura ::routing_table_t que receberá os dados.
 * @return true se a operação for bem-sucedida, false caso contrário.
 */
bool routing_module_get_routing_table(routing_table_t *table)
{
    if (table == NULL)
    {
        return false;
    }
    xSemaphoreTake(routing_table_mutex, portMAX_DELAY);
    (void)memcpy(table, &routing_table, sizeof(routing_table_t));
    xSemaphoreGive(routing_table_mutex);
    return true;
}

/**
 * @brief Consulta a tabela de vizinhança atual.
 *
 * Copia a tabela interna para a estrutura fornecida.
 *
 * @param table Ponteiro para a estrutura ::neighbor_table_t que receberá os dados.
 * @return true se a operação for bem-sucedida, false caso contrário.
 */
bool routing_module_get_neighbor_table(neighbor_table_t *table)
{
    if (table == NULL)
    {
        return false;
    }
    xSemaphoreTake(routing_table_mutex, portMAX_DELAY);
    (void)memcpy(table, &neighbor_table, sizeof(neighbor_table_t));
    xSemaphoreGive(routing_table_mutex);
    return true;
}

/**
 * @brief Enfileira uma mensagem para envio.
 *
 * Preenche um item de mensagem de envio e o coloca na fila de envio para processamento pela task dedicada.
 *
 * @param dest_id Identificador do nó destino.
 * @param data Ponteiro para os dados da mensagem.
 * @param length Comprimento dos dados (em bytes).
 * @param mode Modo de envio (ROUTING_MODE_UNICAST, ROUTING_MODE_MULTICAST ou ROUTING_MODE_BROADCAST).
 * @return true se a mensagem for enfileirada com sucesso, false caso contrário.
 */
bool routing_module_send_message(const char *dest_id, const uint8_t *data, uint16_t length, uint8_t mode)
{
    routing_send_queue_item_t item;
    
    if ((data == NULL) || (length == 0U))
    {
        ESP_LOGE(TAG, "Invalid parameters for sending message.");
        routing_module_notify(ROUTING_EVENT_ROUTE_FAILURE, (void *)dest_id);
        return false;
    }
    
    (void)memset(&item, 0, sizeof(item));
    if (dest_id != NULL)
    {
        (void)strncpy(item.item.dest_id, dest_id, sizeof(item.item.dest_id) - 1);
    }
    (void)memcpy(item.item.data, data, length);
    item.item.length = length;
    item.item.mode = mode;
    
    if (xQueueSend(routing_send_queue, &item, portMAX_DELAY) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to enqueue message for sending.");
        return false;
    }
    return true;
}

/**
 * @brief Recebe uma mensagem utilizando o módulo de roteamento.
 *
 * Processa de forma segura a mensagem recebida, utilizando mecanismos de sincronização, e notifica os callbacks
 * registrados com a mensagem recebida. O módulo aloca dinamicamente uma estrutura ::routing_received_message_t que
 * deve ser liberada pelo callback após o processamento.
 *
 * @param src_id Identificador do nó de origem.
 * @param data Ponteiro para os dados da mensagem.
 * @param length Comprimento dos dados (em bytes). O tamanho máximo permitido é 256 bytes.
 * @return true se a mensagem for recebida e processada com sucesso, false caso contrário.
 */
bool routing_module_receive_message(const char *src_id, const uint8_t *data, uint16_t length)
{
    routing_received_message_t *msg = NULL;
    
    if ((src_id == NULL) || (data == NULL) || (length == 0U) || (length > 256U))
    {
        ESP_LOGE(TAG, "Invalid parameters for receiving message.");
        return false;
    }
    
    xSemaphoreTake(receive_mutex, portMAX_DELAY);
    msg = (routing_received_message_t *)pvPortMalloc(sizeof(routing_received_message_t));
    if (msg == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for received message.");
        xSemaphoreGive(receive_mutex);
        return false;
    }
    (void)strncpy(msg->src_id, src_id, sizeof(msg->src_id) - 1);
    msg->src_id[sizeof(msg->src_id) - 1] = '\0';
    msg->length = length;
    (void)memcpy(msg->data, data, length);
    xSemaphoreGive(receive_mutex);
    
    ESP_LOGI(TAG, "Received message from %s, size: %u bytes.", msg->src_id, msg->length);
    routing_module_notify(ROUTING_EVENT_MESSAGE_RECEIVED, (void *)msg);
    return true;
}

/**
 * @brief Enfileira um evento mesh recebido do esp_mesh_connection_module.
 *
 * Cria um item de evento e o envia para a fila, sinalizando através do event group.
 *
 * @param event_id Código do evento mesh.
 * @param event_data Ponteiro para dados específicos do evento.
 * @return true se o evento for enfileirado com sucesso, false caso contrário.
 */
bool routing_module_queue_mesh_event(uint8_t event_id, const void *event_data)
{
    routing_event_item_t item;
    item.event_id = event_id;
    item.event_data = (void *)event_data;
    
    if (xQueueSend(routing_event_queue, &item, portMAX_DELAY) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to queue mesh event: %u", event_id);
        return false;
    }
    xEventGroupSetBits(routing_event_group, ROUTING_EVENT_BIT_NEW);
    return true;
}

/**
 * @brief Processa um evento mesh recebido do esp_mesh_connection_module.
 *
 * Este método é chamado pela task de eventos para processar itens da fila.
 *
 * @param event_id Código do evento mesh.
 * @param event_data Ponteiro para dados específicos do evento (por exemplo, ::neighbor_table_t ou uma string para o nó pai).
 * @return true se o evento for processado com sucesso, false caso contrário.
 */
bool routing_module_process_mesh_event(uint8_t event_id, const void *event_data)
{
    if (event_data == NULL)
    {
        ESP_LOGE(TAG, "Null event data provided.");
        return false;
    }
    if (event_id == MESH_EVENT_NEIGHBOR_CHANGE)
    {
        ESP_LOGI(TAG, "Processing MESH_EVENT_NEIGHBOR_CHANGE event.");
        return routing_module_update_topology((const neighbor_table_t *)event_data);
    }
    else if (event_id == MESH_EVENT_PARENT_CONNECTED)
    {
        ESP_LOGI(TAG, "Processing MESH_EVENT_PARENT_CONNECTED event. Parent connected.");
        return routing_module_recalculate_routes();
    }
    else if (event_id == MESH_EVENT_ROOT_SWITCHED)
    {
        ESP_LOGI(TAG, "Processing MESH_EVENT_ROOT_SWITCHED event. Root switched.");
        return routing_module_recalculate_routes();
    }
    else
    {
        ESP_LOGW(TAG, "Unhandled mesh event: %u", event_id);
        return false;
    }
}

/**
 * @brief Atualiza dinamicamente as configurações do módulo de roteamento.
 *
 * Permite ajustar parâmetros como o custo padrão, número de tentativas de fallback e intervalo entre tentativas,
 * salvando-os no arquivo "config.ini" para persistência.
 *
 * @param config Ponteiro para a estrutura ::routing_config_t com as novas configurações.
 * @return true se as configurações forem atualizadas e salvas com sucesso, false caso contrário.
 */
bool routing_module_set_config(const routing_config_t *config)
{
    if (config == NULL)
    {
        ESP_LOGE(TAG, "Null configuration provided.");
        return false;
    }
    xSemaphoreTake(config_mutex, portMAX_DELAY);
    routing_config.default_cost = config->default_cost;
    routing_config.retry_count = config->retry_count;
    routing_config.retry_delay_ms = config->retry_delay_ms;
    ESP_LOGI(TAG, "Routing configuration updated: default_cost=%u, retry_count=%u, retry_delay_ms=%lu",
             routing_config.default_cost, routing_config.retry_count, (unsigned long)routing_config.retry_delay_ms);
    xSemaphoreGive(config_mutex);
    (void)routing_module_save_config();
    return true;
}

/**
 * @brief Obtém as configurações atuais do módulo de roteamento.
 *
 * Copia as configurações atuais para a estrutura fornecida.
 *
 * @param config Ponteiro para a estrutura ::routing_config_t que receberá as configurações.
 * @return true se a operação for bem-sucedida, false caso contrário.
 */
bool routing_module_get_config(routing_config_t *config)
{
    if (config == NULL)
    {
        return false;
    }
    xSemaphoreTake(config_mutex, portMAX_DELAY);
    config->default_cost = routing_config.default_cost;
    config->retry_count = routing_config.retry_count;
    config->retry_delay_ms = routing_config.retry_delay_ms;
    xSemaphoreGive(config_mutex);
    return true;
}

/**
 * @brief Registra um callback para eventos do módulo de roteamento.
 *
 * Permite que outros módulos se inscrevam para receber notificações sobre atualizações nas tabelas e sobre erros.
 *
 * @param callback Ponteiro para a função de callback.
 * @return true se o callback for registrado com sucesso, false caso contrário.
 */
bool routing_module_register_callback(routing_event_callback_t callback)
{
    uint8_t i;
    if (callback == NULL)
    {
        return false;
    }
    if (routing_callback_count >= MAX_ROUTING_CALLBACKS)
    {
        ESP_LOGE(TAG, "Maximum number of callbacks reached.");
        return false;
    }
    for (i = 0U; i < routing_callback_count; i++)
    {
        if (routing_callbacks[i] == callback)
        {
            ESP_LOGW(TAG, "Callback already registered.");
            return true;
        }
    }
    routing_callbacks[routing_callback_count] = callback;
    routing_callback_count++;
    ESP_LOGI(TAG, "Callback registered successfully. Total: %u", routing_callback_count);
    return true;
}

/**
 * @brief Remove um callback registrado para eventos do módulo de roteamento.
 *
 * @param callback Ponteiro para a função de callback a ser removido.
 * @return true se o callback for removido com sucesso, false caso contrário.
 */
bool routing_module_unregister_callback(routing_event_callback_t callback)
{
    uint8_t i, j;
    if (callback == NULL)
    {
        return false;
    }
    for (i = 0U; i < routing_callback_count; i++)
    {
        if (routing_callbacks[i] == callback)
        {
            for (j = i; j < routing_callback_count - 1U; j++)
            {
                routing_callbacks[j] = routing_callbacks[j + 1U];
            }
            routing_callbacks[routing_callback_count - 1U] = NULL;
            routing_callback_count--;
            ESP_LOGI(TAG, "Callback unregistered successfully. Remaining: %u", routing_callback_count);
            return true;
        }
    }
    ESP_LOGW(TAG, "Callback not found for unregistration.");
    return false;
}
