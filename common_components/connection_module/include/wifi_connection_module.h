/**
 * @file wifi_connection_module.h
 * @brief Cabeçalho do Módulo de Conectividade Wi‑Fi para a rede CAN‑ESP.
 *
 * Este módulo gerencia a conexão Wi‑Fi do ESP32 utilizando parâmetros lidos do arquivo
 * "config.ini" (via sd_storage_module), implementa monitoramento ativo, reconexão com backoff
 * progressivo e persistência dos dados da última conexão na NVS. Desenvolvido para ser aplicado
 * em redes CAN de veículos elétricos reais, em conformidade com MISRA C:2012 e ISO 11898.
 *
 * @author
 * @date
 */

#ifndef WIFI_CONNECTION_MODULE_H
#define WIFI_CONNECTION_MODULE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Estrutura de parâmetros de configuração Wi‑Fi.
 */
typedef struct {
    char ssid[32];         /**< SSID da rede Wi‑Fi */
    char password[64];     /**< Senha da rede Wi‑Fi */
    uint8_t channel;       /**< Canal Wi‑Fi */
    uint8_t authmode;      /**< Modo de autenticação (ver esp_wifi_types.h) */
    bool auto_connect;     /**< Flag para conexão automática */
    uint32_t maximum_retry;/**< Número máximo de tentativas de reconexão */
} wifi_config_params_t;

/**
 * @brief Estrutura para informações de conexão persistentes.
 */
typedef struct {
    char ssid[32];         /**< SSID da rede conectada */
    char ip[16];           /**< Endereço IP atribuído (formato xxx.xxx.xxx.xxx) */
} wifi_connection_info_t;

/**
 * @brief Inicializa o módulo de conexão Wi‑Fi.
 *
 * Configura o Wi‑Fi utilizando parâmetros lidos do arquivo de configuração (via sd_storage_module),
 * inicializa a pilha TCP/IP, registra handlers de eventos e tenta conectar.
 *
 * @return true se a inicialização for bem-sucedida, false caso contrário.
 */
bool wifi_connection_module_init(void);

/**
 * @brief Finaliza a conexão Wi‑Fi e libera os recursos alocados.
 *
 * @return true se a operação for bem-sucedida, false caso contrário.
 */
bool wifi_connection_module_deinit(void);

/**
 * @brief Obtém os parâmetros de configuração Wi‑Fi atualmente em uso.
 *
 * Copia os parâmetros da configuração atual para a estrutura fornecida.
 *
 * @param[out] config Ponteiro para a estrutura que receberá os parâmetros.
 * @return true se a operação for bem-sucedida, false caso contrário.
 */
bool wifi_connection_module_get_config(wifi_config_params_t *config);

/**
 * @brief Inicia a tarefa de reconexão Wi‑Fi com backoff progressivo.
 *
 * Esta tarefa monitora a conexão e, em caso de desconexão, tenta reconectar aplicando um
 * delay progressivo entre as tentativas.
 */
void wifi_connection_module_start_reconnect_task(void);

/**
 * @brief Persiste as informações da última conexão Wi‑Fi bem-sucedida na NVS.
 *
 * Salva os dados (por exemplo, SSID e IP) na NVS para posterior recuperação.
 *
 * @param info Estrutura contendo as informações de conexão.
 * @return true se os dados forem salvos com sucesso, false caso contrário.
 */
bool wifi_connection_module_store_connection_info(const wifi_connection_info_t *info);

/**
 * @brief Carrega as informações da última conexão Wi‑Fi bem-sucedida da NVS.
 *
 * Recupera os dados de conexão salvos na NVS e os copia para a estrutura fornecida.
 *
 * @param info Ponteiro para a estrutura que receberá os dados de conexão.
 * @return true se os dados forem carregados com sucesso, false caso contrário.
 */
bool wifi_connection_module_load_connection_info(wifi_connection_info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_CONNECTION_MODULE_H */
