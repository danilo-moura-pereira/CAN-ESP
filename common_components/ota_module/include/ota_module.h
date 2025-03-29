/**
 * @file ota_module.h
 * @brief Interface pública para o módulo OTA do projeto CAN-ESP.
 *
 * Este módulo gerencia a atualização remota do firmware via OTA, utilizando MQTT para notificação,
 * ESP-MESH para distribuição e a API OTA do ESP-IDF para aplicação da atualização nos nós leaf.
 * O componente é desenvolvido para aplicações em veículos elétricos reais, seguindo as diretrizes
 * do MISRA C:2012 e os padrões ISO 11898.
 *
 * As configurações de OTA – versões de firmware para cada ECU, tópicos MQTT e intervalo de verificação –
 * são carregadas dinamicamente a partir do arquivo "config.ini". Uma API de refresh permite a atualização
 * desses parâmetros em tempo de execução sem reinicializar a ECU.
 *
 * @note As funções deste módulo assumem que os módulos de conexão, roteamento e armazenamento
 * já foram devidamente inicializados.
 */

#ifndef OTA_MODULE_H
#define OTA_MODULE_H

#include <stdint.h>
#include <stdbool.h>

/* Tamanho dos pacotes para segmentação do firmware (em bytes) */
#define OTA_PACKET_SIZE 1024U

/* Comprimento máximo para os tópicos MQTT */
#define TOPIC_MAX_LEN 64U

/* Comprimento máximo para nomes de arquivo */
#define MAX_FILENAME_LENGTH 128U

/**
 * @brief Status do processo OTA.
 */
typedef enum {
    OTA_STATUS_IDLE,
    OTA_STATUS_UPDATE_AVAILABLE,
    OTA_STATUS_DOWNLOADING,
    OTA_STATUS_DISTRIBUTING,
    OTA_STATUS_APPLYING,
    OTA_STATUS_SUCCESS,
    OTA_STATUS_FAILURE,
    OTA_STATUS_ROLLBACK
} ota_status_t;

/**
 * @brief Tipo de callback para notificação de eventos OTA.
 *
 * @param status Status atual do processo OTA.
 * @param ecu_id Identificador da ECU relacionada ao evento.
 * @param data Ponteiro para dados adicionais (pode ser NULL).
 */
typedef void (*ota_event_callback_t)(ota_status_t status, const char *ecu_id, const void *data);

/**
 * @brief Inicializa o módulo OTA.
 *
 * Configura os recursos necessários para a atualização OTA, integrando os módulos de conexão,
 * roteamento e armazenamento, e carrega os parâmetros de configuração do arquivo "config.ini".
 *
 * @return true se a inicialização for bem-sucedida, false caso contrário.
 */
bool ota_module_init(void);

/**
 * @brief Registra um callback para eventos OTA.
 *
 * Permite que outros módulos sejam notificados sobre alterações no status do processo OTA.
 *
 * @param callback Ponteiro para a função de callback.
 * @return true se o callback for registrado com sucesso, false caso contrário.
 */
bool ota_module_register_callback(ota_event_callback_t callback);

/**
 * @brief Verifica periodicamente se há uma nova atualização de firmware disponível para a ECU "monitor_ecu".
 *
 * Obtém a versão disponível do firmware em formato numérico por meio do módulo MQTT e compara com a versão instalada.
 *
 * @return true se uma atualização estiver disponível, false caso contrário.
 */
bool ota_module_check_update(void);

/**
 * @brief Verifica sob demanda se a versão disponível do firmware é superior à instalada para a ECU especificada.
 *
 * @param ecu_id Identificador da ECU.
 * @param available_version Versão disponível do firmware.
 * @return true se a versão disponível for superior à instalada, false caso contrário.
 */
bool ota_module_check_version(const char *ecu_id, uint32_t available_version);

/**
 * @brief Inicia o download do firmware para a ECU especificada.
 *
 * Realiza o download do firmware via MQTT/HTTP e grava-o no SD Card utilizando um nome padronizado,
 * conforme o padrão:
 * - firmware_monitor_ecu_vX.bin
 * - firmware_acceleration_control_ecu_vX.bin
 * - firmware_steering_control_ecu_vX.bin
 * - firmware_motor_control_ecu_vX.bin
 * - firmware_brake_control_ecu_vX.bin
 *
 * Onde _vX é substituído pelo número da versão instalada.
 *
 * Após gravar o arquivo, o firmware é carregado para a memória por meio de ota_module_load_firmware().
 *
 * @param ecu_id Identificador da ECU que receberá a atualização.
 * @return true se o download e o carregamento forem realizados com sucesso, false caso contrário.
 */
bool ota_module_download_firmware(const char *ecu_id);

/**
 * @brief Carrega o firmware do SD Card para a memória.
 *
 * Lê o arquivo especificado do SD Card e armazena os dados no contexto OTA.
 *
 * @param filename Nome do arquivo a ser lido.
 * @return true se o firmware for carregado com sucesso, false caso contrário.
 */
bool ota_module_load_firmware(const char *filename);

/**
 * @brief Segmenta o firmware em pacotes para distribuição via ESP-MESH.
 *
 * Divide o firmware carregado em blocos de tamanho OTA_PACKET_SIZE e armazena os segmentos
 * em um array dinâmico do tipo ota_segment_t.
 *
 * @param firmware_data Ponteiro para os dados do firmware.
 * @param firmware_size Tamanho total do firmware em bytes.
 * @return true se a segmentação for realizada com sucesso, false caso contrário.
 */
bool ota_module_segment_firmware(const uint8_t *firmware_data, uint32_t firmware_size);

/**
 * @brief Distribui os pacotes de firmware para a ECU especificada via ESP-MESH.
 *
 * Itera sobre o array de segmentos gerado por ota_module_segment_firmware e envia cada pacote para a ECU utilizando a função
 * routing_module_send_message(), que encaminha a mensagem via ESP-MESH.
 *
 * @param ecu_id Identificador da ECU destino.
 * @return true se todos os pacotes forem enviados com sucesso, false caso contrário.
 */
bool ota_module_distribute_firmware(const char *ecu_id);

/**
 * @brief Aplica a atualização do firmware utilizando a API OTA do ESP-IDF.
 *
 * Grava o firmware na partição OTA. Em caso de erro, o status é definido como OTA_STATUS_FAILURE
 * e o rollback é acionado.
 *
 * @param ecu_id Identificador da ECU que aplicará a atualização.
 * @return true se a atualização for aplicada com sucesso, false caso contrário.
 */
bool ota_module_apply_update(const char *ecu_id);

/**
 * @brief Atualiza as configurações OTA, persistindo dados no arquivo "config.ini".
 *
 * Utiliza o módulo de armazenamento para atualizar os parâmetros de configuração, como as versões de firmware,
 * os tópicos MQTT e o intervalo de verificação.
 *
 * @return true se as configurações forem atualizadas com sucesso, false caso contrário.
 */
bool ota_module_update_config(void);

/**
 * @brief Atualiza dinamicamente os parâmetros de configuração OTA.
 *
 * Lê o arquivo "config.ini" e atualiza os parâmetros internos do módulo OTA sem reinicializar a ECU.
 *
 * @return true se os parâmetros forem atualizados com sucesso, false caso contrário.
 */
bool ota_module_refresh_config(void);

/**
 * @brief Realiza o rollback do firmware em caso de falha irreversível na atualização.
 *
 * Este método aciona o rollback do firmware, restaurando a versão anterior ou adotando outra estratégia de fallback.
 *
 * @param ecu_id Identificador da ECU.
 * @return true se o rollback for bem-sucedido, false caso contrário.
 */
bool ota_module_rollback_update(const char *ecu_id);

#endif /* OTA_MODULE_H */
