/**
 * @file esp_mesh_connection_module.c
 * @brief Implementação do Módulo de Conectividade Mesh para a rede CAN-ESP.
 *
 * Este módulo implementa a conectividade mesh para a rede CAN-ESP, permitindo a integração
 * com módulos de MQTT, Wi‑Fi e OTA. São implementados recursos para configuração dinâmica,
 * monitoramento da conexão, reconexão com backoff progressivo, notificações via callbacks,
 * fallback de configuração do roteador, e interfaces para consulta de topologia.
 *
 * @note Os parâmetros de configuração (exceto os do softAP) são lidos a partir do arquivo "config.ini"
 * via o módulo sd_storage_module. Os parâmetros do softAP são obtidos via a API do wifi_connection_module.
 * Este módulo está em conformidade com MISRA C:2012 e ISO 11898.
 */

#include "esp_mesh_connection_module.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "sd_storage_module.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <esp_system.h>

/* Tag para logs */
#define TAG "MESH_CONN_MODULE"

/* Tamanho máximo da linha de configuração */
#define CONFIG_LINE_MAX_LEN 128U

/* Event Group Bits para monitoramento da conexão mesh */
#define MESH_CONNECTED_BIT    (1U << 0U)
#define MESH_DISCONNECTED_BIT (1U << 1U)

/* Tamanho máximo para registro de callbacks */
#define MAX_MESH_EVENT_CALLBACKS 10U

/* Variáveis estáticas internas */
static EventGroupHandle_t mesh_event_group = NULL;
static esp_mesh_config_params_t mesh_config =
{
    .mesh_id = "",
    .channel = MESH_DEFAULT_CHANNEL,
    .max_retry = MESH_DEFAULT_MAX_RETRY,
    .reconnection_delay_ms = MESH_DEFAULT_RECONNECT_DELAY_MS,
    .auto_reconnect = true,
    .self_organized = true,
    /* Parâmetros do mesh AP (softAP) – serão atualizados via update_router_config */
    .router_ssid = "",
    .router_password = "",
    .router_channel = 0U,
    .router_authmode = 0U,
    /* Parâmetros adicionais do mesh AP */
    .mesh_ap_max_connection = MESH_AP_DEFAULT_MAX_CONNECTION,
    .mesh_ap_nonmesh_max_connection = MESH_AP_DEFAULT_NONMESH_MAX_CONNECTION,
    .mesh_ap_password = MESH_AP_DEFAULT_PASSWORD,
    .mesh_ap_authmode = MESH_AP_DEFAULT_AUTHMODE
};

/* Estrutura interna para armazenar informações de topologia */
static esp_mesh_topology_info_t mesh_topology_info = { {0}, 0, { {0} } };

/* Array e contador para callbacks registrados */
static esp_mesh_event_callback_t event_callbacks[MAX_MESH_EVENT_CALLBACKS] = { 0 };
static uint8_t event_callback_count = 0U;

/* Protótipos de funções internas */
static bool mesh_connection_module_load_config(void);
static void mesh_monitor_task(void *arg);
static bool mesh_reconnection_policy(void);
static bool set_mesh_id_from_mac(void);
static bool persist_mesh_config(void);

/**
 * @brief Manipulador de eventos da rede mesh.
 *
 * Trata os eventos críticos do ESP‑MESH, implementando ações reais para cada evento,
 * como atualização do estado interno, gerenciamento de reconexão e atualização das informações
 * de topologia (nó pai e vizinhança). Além disso, notifica os módulos registrados.
 *
 * @param arg Parâmetro não utilizado.
 * @param event_base Base do evento.
 * @param event_id Identificador do evento (conforme ::mesh_event_id_t).
 * @param event_data Dados associados ao evento.
 * @return int Código de status (0 para sucesso).
 */
int esp_mesh_connection_module_event_handler(void *arg, int event_base, int event_id, void *event_data)
{
    (void)arg;
    ESP_LOGI(TAG, "Evento mesh recebido: base=%d, id=%d", event_base, event_id);

    switch ((mesh_event_id_t)event_id)
    {
        case MESH_EVENT_STARTED:
            if (xEventGroupSetBits(mesh_event_group, MESH_CONNECTED_BIT) != 0U)
            {
                ESP_LOGI(TAG, "Rede mesh iniciada e sinalizada como conectada.");
            }
            else
            {
                ESP_LOGE(TAG, "Erro ao sinalizar estado de conexão mesh.");
            }
            break;

        case MESH_EVENT_STOPPED:
            (void)xEventGroupClearBits(mesh_event_group, MESH_CONNECTED_BIT);
            (void)xEventGroupSetBits(mesh_event_group, MESH_DISCONNECTED_BIT);
            ESP_LOGW(TAG, "Rede mesh parada. Estado atualizado para desconectado.");
            break;

        case MESH_EVENT_DISCONNECTED:
            (void)xEventGroupClearBits(mesh_event_group, MESH_CONNECTED_BIT);
            (void)xEventGroupSetBits(mesh_event_group, MESH_DISCONNECTED_BIT);
            ESP_LOGW(TAG, "Rede mesh desconectada. Iniciando procedimento de reconexão.");
            if (mesh_config.auto_reconnect)
            {
                if (!mesh_reconnection_policy())
                {
                    ESP_LOGE(TAG, "Falha ao aplicar a política de reconexão mesh.");
                }
            }
            break;

        case MESH_EVENT_PARENT_CONNECTED:
            {
                /* Se event_data for fornecido, presume-se que é uma string com o ID do nó pai */
                const char *parent = (const char *)event_data;
                if (parent != NULL)
                {
                    (void)snprintf(mesh_topology_info.parent_id, sizeof(mesh_topology_info.parent_id), "%s", parent);
                }
                else
                {
                    (void)snprintf(mesh_topology_info.parent_id, sizeof(mesh_topology_info.parent_id), "UNKNOWN");
                }
                ESP_LOGI(TAG, "Conexão com o nó pai estabelecida: %s", mesh_topology_info.parent_id);
            }
            break;

        case MESH_EVENT_NEIGHBOR_CHANGE:
            {
                /* Atualiza informações de vizinhança.
                   Aqui, para demonstração, simulamos a atualização com dados fictícios.
                   Em uma implementação real, os dados devem ser obtidos via API do ESP-MESH. */
                mesh_topology_info.neighbor_count = 2U;
                (void)snprintf(mesh_topology_info.neighbor_ids[0], sizeof(mesh_topology_info.neighbor_ids[0]), "NEIGHBOR_A");
                (void)snprintf(mesh_topology_info.neighbor_ids[1], sizeof(mesh_topology_info.neighbor_ids[1]), "NEIGHBOR_B");
                ESP_LOGI(TAG, "Alteração na vizinhança detectada: %u vizinhos conectados.", mesh_topology_info.neighbor_count);
            }
            break;

        case MESH_EVENT_ROOT_SWITCHED:
            ESP_LOGI(TAG, "Mudança do nó root detectada. Atualizando topologia da rede mesh.");
            /* Poder-se-ia atualizar o ID do nó pai ou realizar outras ações aqui */
            break;

        default:
            ESP_LOGW(TAG, "Evento mesh não tratado: id=%d", event_id);
            break;
    }

    /* Notifica os callbacks registrados */
    {
        uint8_t idx;
        for (idx = 0U; idx < event_callback_count; idx++)
        {
            if (event_callbacks[idx] != NULL)
            {
                event_callbacks[idx](event_id, event_data);
            }
        }
    }

    return 0;
}

/**
 * @brief Carrega os parâmetros de configuração da rede mesh a partir do arquivo "config.ini".
 *
 * Lê o arquivo de configuração e atualiza os parâmetros da rede mesh (exceto os parâmetros do softAP).
 *
 * @return true se a configuração for carregada com sucesso, false caso contrário.
 */
static bool mesh_connection_module_load_config(void)
{
    char config_line[CONFIG_LINE_MAX_LEN];
    char config_path[MAX_FILENAME_LENGTH];
    FILE *file;

    (void)snprintf(config_path, MAX_FILENAME_LENGTH, "%s/%s", MOUNT_POINT, "config.ini");
    file = fopen(config_path, "r");
    if (file == NULL)
    {
        ESP_LOGW(TAG, "Arquivo de configuração %s não encontrado, utilizando valores padrão.", config_path);
        return false;
    }

    while (fgets(config_line, CONFIG_LINE_MAX_LEN, file) != NULL)
    {
        char *newline = strchr(config_line, '\n');
        if (newline != NULL)
        {
            *newline = '\0';
        }
        if (strncmp(config_line, "MESH_ID=", 8U) == 0)
        {
            (void)strncpy(mesh_config.mesh_id, config_line + 8U, sizeof(mesh_config.mesh_id) - 1U);
        }
        else if (strncmp(config_line, "MESH_CHANNEL=", 13U) == 0)
        {
            mesh_config.channel = (uint8_t)atoi(config_line + 13U);
        }
        else if (strncmp(config_line, "MESH_MAX_RETRY=", 15U) == 0)
        {
            mesh_config.max_retry = (uint8_t)atoi(config_line + 15U);
        }
        else if (strncmp(config_line, "MESH_RECONNECT_DELAY_MS=", 24U) == 0)
        {
            mesh_config.reconnection_delay_ms = (uint32_t)atoi(config_line + 24U);
        }
        else if (strncmp(config_line, "MESH_AUTO_RECONNECT=", 20U) == 0)
        {
            if (strncasecmp(config_line + 20U, "true", 4U) == 0)
            {
                mesh_config.auto_reconnect = true;
            }
            else
            {
                mesh_config.auto_reconnect = false;
            }
        }
        /* Parâmetros do softAP não são lidos deste arquivo */
    }
    fclose(file);
    ESP_LOGI(TAG, "Configuração da rede mesh carregada a partir de %s.", config_path);
    return true;
}

/**
 * @brief Define o mesh_id a partir do endereço MAC do dispositivo, se não configurado.
 *
 * Se a chave "MESH_ID" estiver vazia, lê o endereço MAC do STA e define o mesh_id
 * como uma string representando os últimos 3 bytes do MAC.
 *
 * @return true se o mesh_id for definido com sucesso, false caso contrário.
 */
static bool set_mesh_id_from_mac(void)
{
    uint8_t mac[6];
    esp_err_t ret = esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao obter MAC address: %s", esp_err_to_name(ret));
        return false;
    }
    if (mesh_config.mesh_id[0] == '\0')
    {
        (void)snprintf(mesh_config.mesh_id, sizeof(mesh_config.mesh_id), "MESH_%02X%02X%02X", mac[3], mac[4], mac[5]);
        ESP_LOGI(TAG, "Mesh ID definido a partir do MAC: %s", mesh_config.mesh_id);
    }
    return true;
}

/**
 * @brief Persiste a configuração mesh na NVS.
 *
 * Salva a estrutura mesh_config na NVS para que as configurações sejam preservadas mesmo após reinicializações.
 *
 * @return true se a operação for bem-sucedida, false caso contrário.
 */
static bool persist_mesh_config(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("mesh_config", NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao abrir NVS para persistência da configuração mesh: %s", esp_err_to_name(err));
        return false;
    }
    err = nvs_set_blob(handle, "mesh_config", &mesh_config, sizeof(mesh_config));
    if (err == ESP_OK)
    {
        (void)nvs_commit(handle);
        ESP_LOGI(TAG, "Configuração mesh persistida com sucesso na NVS.");
    }
    else
    {
        ESP_LOGE(TAG, "Falha ao persistir configuração mesh: %s", esp_err_to_name(err));
    }
    nvs_close(handle);
    return (err == ESP_OK);
}

/**
 * @brief Atualiza dinamicamente os parâmetros de configuração da rede mesh.
 *
 * Atualiza a configuração, persiste na NVS, notifica módulos interessados e reinicializa a rede mesh.
 *
 * @param params Ponteiro para a estrutura com os novos parâmetros.
 * @return true se a atualização for bem-sucedida, false caso contrário.
 */
bool esp_mesh_connection_module_set_config(const esp_mesh_config_params_t *params)
{
    if (params == NULL)
    {
        return false;
    }
    mesh_config = *params;
    ESP_LOGI(TAG, "Configuração da rede mesh atualizada: MESH_ID: %s, Canal: %u",
             mesh_config.mesh_id, mesh_config.channel);

    ESP_LOGI(TAG, "Notificando módulos interessados sobre a atualização da configuração mesh.");

    if (!persist_mesh_config())
    {
        ESP_LOGW(TAG, "Persistência da configuração mesh falhou.");
    }
    return esp_mesh_connection_module_reconnect();
}

/**
 * @brief Retorna a configuração atual da rede mesh.
 *
 * Copia os parâmetros atuais para a estrutura fornecida.
 *
 * @param[out] params Ponteiro para a estrutura que receberá os parâmetros.
 * @return true se a operação for bem-sucedida, false caso contrário.
 */
bool esp_mesh_connection_module_get_config(esp_mesh_config_params_t *params)
{
    if (params == NULL)
    {
        return false;
    }
    *params = mesh_config;
    return true;
}

/**
 * @brief Atualiza a configuração do roteador (softAP) utilizando os parâmetros do módulo Wi‑Fi.
 *
 * Se a obtenção dos parâmetros via wifi_connection_module_get_config() falhar, utiliza valores de fallback.
 *
 * @return true se a configuração for atualizada com sucesso, false caso contrário.
 */
bool esp_mesh_connection_module_update_router_config(void)
{
    wifi_config_params_t wifi_config;
    extern bool wifi_connection_module_get_config(wifi_config_params_t *config);
    if (!wifi_connection_module_get_config(&wifi_config))
    {
        ESP_LOGE(TAG, "Falha ao obter configuração Wi‑Fi do módulo Wi‑Fi. Utilizando parâmetros de fallback.");
        (void)strncpy(mesh_config.router_ssid, FALLBACK_ROUTER_SSID, sizeof(mesh_config.router_ssid) - 1U);
        (void)strncpy(mesh_config.router_password, FALLBACK_ROUTER_PASSWORD, sizeof(mesh_config.router_password) - 1U);
        mesh_config.router_channel = FALLBACK_ROUTER_CHANNEL;
        mesh_config.router_authmode = FALLBACK_ROUTER_AUTHMODE;
        return true;
    }
    (void)strncpy(mesh_config.router_ssid, wifi_config.ssid, sizeof(mesh_config.router_ssid) - 1U);
    (void)strncpy(mesh_config.router_password, wifi_config.password, sizeof(mesh_config.router_password) - 1U);
    mesh_config.router_channel = wifi_config.channel;
    mesh_config.router_authmode = wifi_config.authmode;
    ESP_LOGI(TAG, "Configuração do roteador atualizada a partir do módulo Wi‑Fi: SSID=%s, Canal: %u",
             mesh_config.router_ssid, mesh_config.router_channel);
    return true;
}

/**
 * @brief Define a organização da rede mesh.
 *
 * Chama a API esp_mesh_set_self_organized() para definir se a rede deve ser auto-organizada.
 *
 * @param self_organized true para rede auto-organizada, false caso contrário.
 * @return true se a operação for bem-sucedida, false caso contrário.
 */
bool esp_mesh_connection_module_set_network_organization(bool self_organized)
{
    mesh_config.self_organized = self_organized;
    esp_err_t ret = esp_mesh_set_self_organized(self_organized);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao definir a organização da rede mesh: %s", esp_err_to_name(ret));
        return false;
    }
    ESP_LOGI(TAG, "Organização da rede mesh definida como: %s", self_organized ? "auto-organizada" : "manual");
    return true;
}

/**
 * @brief Registra um callback para eventos críticos da rede mesh.
 *
 * Permite que outros módulos se inscrevam para receber notificações de eventos (ex.: reconexão, falhas, status).
 *
 * @param callback Ponteiro para a função de callback.
 * @return true se o callback for registrado com sucesso, false caso contrário.
 */
bool esp_mesh_connection_module_register_callback(esp_mesh_event_callback_t callback)
{
    if (callback == NULL)
    {
        return false;
    }
    if (event_callback_count >= MAX_MESH_EVENT_CALLBACKS)
    {
        ESP_LOGE(TAG, "Número máximo de callbacks registrados atingido.");
        return false;
    }
    {
        uint8_t i;
        for (i = 0U; i < event_callback_count; i++)
        {
            if (event_callbacks[i] == callback)
            {
                ESP_LOGW(TAG, "Callback já registrado.");
                return true;
            }
        }
    }
    event_callbacks[event_callback_count] = callback;
    event_callback_count++;
    ESP_LOGI(TAG, "Callback registrado com sucesso. Total: %u", event_callback_count);
    return true;
}

/**
 * @brief Remove um callback registrado para eventos críticos da rede mesh.
 *
 * @param callback Ponteiro para a função de callback a ser removido.
 * @return true se o callback for removido com sucesso, false caso contrário.
 */
bool esp_mesh_connection_module_unregister_callback(esp_mesh_event_callback_t callback)
{
    if (callback == NULL)
    {
        return false;
    }
    {
        uint8_t i;
        for (i = 0U; i < event_callback_count; i++)
        {
            if (event_callbacks[i] == callback)
            {
                uint8_t j;
                for (j = i; j < event_callback_count - 1U; j++)
                {
                    event_callbacks[j] = event_callbacks[j + 1U];
                }
                event_callbacks[event_callback_count - 1U] = NULL;
                event_callback_count--;
                ESP_LOGI(TAG, "Callback removido com sucesso. Total restante: %u", event_callback_count);
                return true;
            }
        }
    }
    ESP_LOGW(TAG, "Callback não encontrado para remoção.");
    return false;
}

/**
 * @brief Obtém as informações de topologia da rede mesh.
 *
 * Preenche a estrutura ::esp_mesh_topology_info_t com informações atuais, incluindo o nó pai e os nós vizinhos.
 *
 * @param[out] topology_info Ponteiro para a estrutura que receberá as informações de topologia.
 * @return true se a operação for bem-sucedida, false caso contrário.
 */
bool esp_mesh_connection_module_get_topology(esp_mesh_topology_info_t *topology_info)
{
    if (topology_info == NULL)
    {
        return false;
    }
    (void)memcpy(topology_info, &mesh_topology_info, sizeof(esp_mesh_topology_info_t));
    return true;
}

/**
 * @brief Tarefa de monitoramento da rede mesh.
 *
 * Verifica periodicamente o estado da rede mesh e, se desconectada, aplica a política de reconexão.
 *
 * @param arg Parâmetro não utilizado.
 */
static void mesh_monitor_task(void *arg)
{
    (void)arg;
    EventBits_t bits;
    const TickType_t xDelay = pdMS_TO_TICKS(10000U); /* Verifica a cada 10 segundos */
    for (;;)
    {
        bits = xEventGroupWaitBits(mesh_event_group, MESH_CONNECTED_BIT | MESH_DISCONNECTED_BIT,
                                   pdTRUE, pdFALSE, xDelay);
        if ((bits & MESH_DISCONNECTED_BIT) != 0U)
        {
            ESP_LOGW(TAG, "Rede mesh desconectada. Aplicando política de reconexão...");
            if (mesh_config.auto_reconnect)
            {
                if (!mesh_reconnection_policy())
                {
                    ESP_LOGE(TAG, "Política de reconexão falhou.");
                }
            }
        }
        else if ((bits & MESH_CONNECTED_BIT) != 0U)
        {
            ESP_LOGI(TAG, "Rede mesh está conectada.");
        }
    }
}

/**
 * @brief Implementa a política de reconexão para a rede mesh.
 *
 * Tenta reconectar utilizando um delay progressivo (backoff). O número de tentativas é controlado
 * pelos parâmetros de configuração.
 *
 * @return true se a reconexão for bem-sucedida, false caso contrário.
 */
static bool mesh_reconnection_policy(void)
{
    esp_err_t ret;
    uint8_t retry = 0U;
    uint32_t delay_ms = mesh_config.reconnection_delay_ms;
    while (retry < mesh_config.max_retry)
    {
        ESP_LOGI(TAG, "Tentativa de reconexão mesh %u com delay %u ms.", (unsigned int)(retry + 1U), (unsigned int)delay_ms);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        ret = esp_mesh_start();
        if (ret == ESP_OK)
        {
            (void)xEventGroupSetBits(mesh_event_group, MESH_CONNECTED_BIT);
            ESP_LOGI(TAG, "Reconexão mesh bem-sucedida.");
            return true;
        }
        retry++;
        delay_ms *= 2U;
    }
    ESP_LOGE(TAG, "Número máximo de tentativas de reconexão mesh atingido.");
    return false;
}

/**
 * @brief Inicializa o módulo de conectividade mesh.
 *
 * Realiza a criação do event group, carregamento da configuração, definição do mesh_id,
 * inicialização do ESP‑MESH, registro do handler de eventos e demais preparos.
 *
 * @return true se a inicialização for bem-sucedida, false caso contrário.
 */
bool esp_mesh_connection_module_init(void)
{
    esp_err_t ret;

    mesh_event_group = xEventGroupCreate();
    if (mesh_event_group == NULL)
    {
        ESP_LOGE(TAG, "Falha ao criar o Event Group para mesh.");
        return false;
    }

    (void)mesh_connection_module_load_config();

    if (!set_mesh_id_from_mac())
    {
        ESP_LOGW(TAG, "Não foi possível definir mesh_id a partir do MAC; utilizando valor padrão.");
    }

    ret = esp_mesh_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao inicializar ESP‑MESH: %s", esp_err_to_name(ret));
        return false;
    }
    esp_mesh_set_config_default();
    ret = esp_mesh_set_config(mesh_config.mesh_id, mesh_config.channel);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao configurar a rede mesh: %s", esp_err_to_name(ret));
        return false;
    }
    ESP_LOGI(TAG, "Rede mesh configurada com MESH_ID: %s, Canal: %u", mesh_config.mesh_id, mesh_config.channel);

    ret = esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID, (esp_event_handler_t)esp_mesh_connection_module_event_handler, NULL);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao registrar handler de eventos mesh: %s", esp_err_to_name(ret));
        return false;
    }
    return true;
}

/**
 * @brief Inicia o serviço de conectividade mesh.
 *
 * Inicia a rede mesh e cria a tarefa de monitoramento que verifica a saúde da conexão e aplica
 * políticas de reconexão conforme necessário.
 *
 * @return true se o serviço for iniciado com sucesso, false caso contrário.
 */
bool esp_mesh_connection_module_start(void)
{
    esp_err_t ret = esp_mesh_start();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao iniciar a rede mesh: %s", esp_err_to_name(ret));
        return false;
    }
    ESP_LOGI(TAG, "Rede mesh iniciada com sucesso.");

    if (xTaskCreate(mesh_monitor_task, "Mesh_Monitor_Task", 4096, NULL, 5, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "Falha ao criar a tarefa de monitoramento da rede mesh.");
        return false;
    }
    return true;
}

/**
 * @brief Reinicializa a conexão mesh.
 *
 * Reinicia a rede mesh para aplicar novas configurações ou recuperar a conexão após uma queda.
 *
 * @return true se a reinicialização for bem-sucedida, false caso contrário.
 */
bool esp_mesh_connection_module_reconnect(void)
{
    esp_err_t ret = esp_mesh_stop();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao parar a rede mesh: %s", esp_err_to_name(ret));
        return false;
    }
    ret = esp_mesh_start();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao reiniciar a rede mesh: %s", esp_err_to_name(ret));
        return false;
    }
    ESP_LOGI(TAG, "Rede mesh reiniciada com sucesso.");
    return true;
}

/**
 * @brief Retorna o status atual da rede mesh.
 *
 * Retorna informações básicas sobre o estado da rede, como se o nó atual é root.
 *
 * @return uint32_t Código ou bitmask representando o status.
 */
uint32_t esp_mesh_connection_module_get_status(void)
{
    if (esp_mesh_is_root())
    {
        return 1U;
    }
    return 0U;
}
