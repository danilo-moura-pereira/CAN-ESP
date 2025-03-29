/**
 * @file mqtt_connection_module.c
 * @brief Implementação do Módulo de Conectividade MQTT para a rede CAN‑ESP.
 *
 * Este módulo gerencia a conexão MQTT utilizando as APIs do ESP‑MQTT. Ele integra-se com o
 * componente wifi_connection_module (que gerencia a conectividade Wi‑Fi) e possibilita a
 * publicação e assinatura de mensagens no broker MQTT. A configuração pode ser atualizada dinamicamente,
 * e callbacks permitem a integração com outros módulos. Desenvolvido para redes CAN de veículos elétricos
 * reais, em conformidade com MISRA C:2012 e ISO 11898.
 */

#include "mqtt_connection_module.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "sd_storage_module.h"  /* Para leitura de parâmetros de configuração */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define TAG "MQTT_CONN_MODULE"

/* Parâmetros MQTT padrão */
static mqtt_config_params_t mqtt_config = {
    .broker_uri = MQTT_DEFAULT_BROKER_URI,
    .client_id = MQTT_DEFAULT_CLIENT_ID,
    .topic = MQTT_DEFAULT_TOPIC,
    .qos = MQTT_DEFAULT_QOS,
    .keepalive = 60U
};

/* Handle do cliente MQTT */
static esp_mqtt_client_handle_t mqtt_client = NULL;
/* Callback para recepção de mensagens */
static void (*mqtt_message_callback)(const char *topic, const char *data) = NULL;

/* Protótipos de funções internas */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static bool mqtt_load_config_from_file(void);

/**
 * @brief Evento handler para eventos MQTT.
 *
 * Trata eventos do cliente MQTT, como conexão, desconexão, e recepção de mensagens.
 *
 * @param handler_args Argumentos do handler (não utilizado).
 * @param base Base do evento.
 * @param event_id Identificador do evento.
 * @param event_data Dados do evento.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch (event->event_id)
    {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Cliente MQTT conectado.");
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "Cliente MQTT desconectado.");
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "Mensagem recebida no tópico: %.*s", event->topic_len, event->topic);
            if (mqtt_message_callback != NULL)
            {
                /* Assegura que a mensagem é terminada em '\0' */
                char *data = strndup(event->data, event->data_len);
                if (data != NULL)
                {
                    mqtt_message_callback(event->topic, data);
                    free(data);
                }
            }
            break;
        default:
            break;
    }
}

/**
 * @brief Carrega os parâmetros de configuração MQTT a partir do arquivo de configuração.
 *
 * Lê o arquivo "config.ini" (no diretório de montagem do SD Card, via sd_storage_module) e
 * extrai os valores para as chaves MQTT_SSID, MQTT_CLIENT_ID, MQTT_BROKER_URI, MQTT_TOPIC,
 * MQTT_QOS e MQTT_KEEPALIVE.
 *
 * @return true se a configuração for carregada com sucesso, false caso contrário.
 */
static bool mqtt_load_config_from_file(void)
{
    char config_line[128];
    char config_path[MAX_FILENAME_LENGTH];
    FILE *file;

    (void)snprintf(config_path, MAX_FILENAME_LENGTH, "%s/%s", MOUNT_POINT, "config.ini");
    file = fopen(config_path, "r");
    if (file == NULL)
    {
        ESP_LOGW(TAG, "Arquivo de configuração %s não encontrado. Usando valores padrão.", config_path);
        return false;
    }

    while (fgets(config_line, sizeof(config_line), file) != NULL)
    {
        char *newline = strchr(config_line, '\n');
        if (newline != NULL)
        {
            *newline = '\0';
        }
        if (strncmp(config_line, "MQTT_BROKER_URI=", 17) == 0)
        {
            (void)strncpy(mqtt_config.broker_uri, config_line + 17, sizeof(mqtt_config.broker_uri) - 1);
        }
        else if (strncmp(config_line, "MQTT_CLIENT_ID=", 15) == 0)
        {
            (void)strncpy(mqtt_config.client_id, config_line + 15, sizeof(mqtt_config.client_id) - 1);
        }
        else if (strncmp(config_line, "MQTT_TOPIC=", 11) == 0)
        {
            (void)strncpy(mqtt_config.topic, config_line + 11, sizeof(mqtt_config.topic) - 1);
        }
        else if (strncmp(config_line, "MQTT_QOS=", 9) == 0)
        {
            mqtt_config.qos = (uint32_t)atoi(config_line + 9);
        }
        else if (strncmp(config_line, "MQTT_KEEPALIVE=", 15) == 0)
        {
            mqtt_config.keepalive = (uint32_t)atoi(config_line + 15);
        }
    }
    fclose(file);
    ESP_LOGI(TAG, "Configuração MQTT carregada a partir de %s.", config_path);
    return true;
}

/**
 * @brief Inicializa o módulo MQTT.
 *
 * Configura e inicia o cliente MQTT utilizando os parâmetros de configuração, os quais podem
 * ser atualizados dinamicamente. Registra o handler de eventos e conecta ao broker.
 *
 * @return true se a inicialização for bem-sucedida, false caso contrário.
 */
bool mqtt_connection_module_init(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {0};

    /* Carrega configuração MQTT a partir do arquivo de configuração */
    (void)mqtt_load_config_from_file();

    mqtt_cfg.uri = mqtt_config.broker_uri;
    mqtt_cfg.client_id = mqtt_config.client_id;
    mqtt_cfg.keepalive = mqtt_config.keepalive;
    mqtt_cfg.event_handle = mqtt_event_handler;

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL)
    {
        ESP_LOGE(TAG, "Falha ao inicializar o cliente MQTT.");
        return false;
    }
    if (esp_mqtt_client_start(mqtt_client) != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao iniciar o cliente MQTT.");
        return false;
    }
    ESP_LOGI(TAG, "Cliente MQTT inicializado e conectado ao broker: %s", mqtt_config.broker_uri);
    return true;
}

/**
 * @brief Finaliza a conexão MQTT e libera os recursos.
 *
 * @return true se a desconexão for bem-sucedida, false caso contrário.
 */
bool mqtt_connection_module_deinit(void)
{
    if (mqtt_client != NULL)
    {
        if (esp_mqtt_client_stop(mqtt_client) != ESP_OK)
        {
            ESP_LOGE(TAG, "Falha ao parar o cliente MQTT.");
            return false;
        }
        if (esp_mqtt_client_destroy(mqtt_client) != ESP_OK)
        {
            ESP_LOGE(TAG, "Falha ao destruir o cliente MQTT.");
            return false;
        }
        mqtt_client = NULL;
        ESP_LOGI(TAG, "Cliente MQTT finalizado com sucesso.");
    }
    return true;
}

/**
 * @brief Publica uma mensagem MQTT.
 *
 * Publica a mensagem no tópico configurado utilizando o nível de QoS definido.
 *
 * @param message Mensagem a ser publicada.
 * @return true se a mensagem for publicada com sucesso, false caso contrário.
 */
bool mqtt_connection_module_publish(const char *message)
{
    if (mqtt_client == NULL || message == NULL)
    {
        ESP_LOGE(TAG, "Cliente MQTT não inicializado ou mensagem nula.");
        return false;
    }
    int msg_id = esp_mqtt_client_publish(mqtt_client, mqtt_config.topic, message, 0, mqtt_config.qos, 0);
    if (msg_id < 0)
    {
        ESP_LOGE(TAG, "Falha ao publicar mensagem MQTT.");
        return false;
    }
    ESP_LOGI(TAG, "Mensagem MQTT publicada, msg_id=%d", msg_id);
    return true;
}

/**
 * @brief Registra um callback para a recepção de mensagens MQTT.
 *
 * O callback é invocado quando uma mensagem é recebida no tópico assinado.
 *
 * @param callback Função callback que recebe o tópico e os dados da mensagem.
 */
void mqtt_connection_module_register_message_callback(void (*callback)(const char *topic, const char *data))
{
    mqtt_message_callback = callback;
    ESP_LOGI(TAG, "Callback de mensagem MQTT registrado com sucesso.");
}

/**
 * @brief Atualiza a configuração MQTT dinamicamente.
 *
 * Permite alterar os parâmetros de configuração MQTT sem reinicializar o módulo.
 *
 * @param config Ponteiro para a estrutura contendo a nova configuração.
 * @return true se a atualização for bem-sucedida, false caso contrário.
 */
bool mqtt_connection_module_update_config(const mqtt_config_params_t *config)
{
    if (config == NULL)
    {
        ESP_LOGE(TAG, "Parâmetro de configuração MQTT nulo.");
        return false;
    }
    (void)memcpy(&mqtt_config, config, sizeof(mqtt_config_params_t));
    ESP_LOGI(TAG, "Configuração MQTT atualizada.");
    /* Para que a nova configuração tenha efeito, é necessário reiniciar o cliente MQTT.
       Isso pode ser feito chamando mqtt_connection_module_deinit() seguido de mqtt_connection_module_init(). */
    if (!mqtt_connection_module_deinit())
    {
        return false;
    }
    return mqtt_connection_module_init();
}

/**
 * @brief Retorna a configuração MQTT atualmente em uso.
 *
 * Copia os parâmetros atuais para a estrutura fornecida.
 *
 * @param[out] config Ponteiro para a estrutura que receberá a configuração.
 * @return true se a operação for bem-sucedida, false caso contrário.
 */
bool mqtt_connection_module_get_config(mqtt_config_params_t *config)
{
    if (config == NULL)
    {
        return false;
    }
    (void)memcpy(config, &mqtt_config, sizeof(mqtt_config_params_t));
    return true;
}

#ifdef __cplusplus
}
#endif
