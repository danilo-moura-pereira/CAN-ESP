/*
 * logger_module.h
 * Cabeçalho do Módulo de Registro de Logs para a ECU de Monitoramento e Diagnóstico.
 * Projeto acadêmico para Mestrado Profissional em Engenharia Elétrica - UnB.
 * Adaptado para conformidade com MISRA C:2012 e padrões automotivos (ISO 11898).
 */

#ifndef LOGGER_MODULE_H
#define LOGGER_MODULE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Tamanho máximo de uma mensagem de log */
#define LOGGER_MSG_MAX_SIZE      (128U)
/* Número máximo de entradas no buffer de log */
#define LOGGER_MAX_ENTRIES       (100U)
/* Tamanho máximo para nomes de arquivos/diretórios */
#define MAX_FILENAME_LENGTH      (64U)
/* Período para flush periódico dos logs críticos (em milissegundos) */
#define LOGGER_PERSISTENCE_PERIOD_MS   (60000U)

/* Período para monitoramento do logger (em milissegundos) */
#define LOGGER_MONITOR_PERIOD_MS (30000U)

/**
 * @brief Enumeração dos níveis de log.
 */
typedef enum {
    LOGGER_LEVEL_INFO = 0,     /**< Nível de informação */
    LOGGER_LEVEL_WARNING,      /**< Nível de aviso */
    LOGGER_LEVEL_CRITICAL      /**< Nível crítico */
} logger_level_t;

/**
 * @brief Estrutura para armazenar uma entrada de log.
 */
typedef struct {
    uint32_t timestamp;                /**< Timestamp em milissegundos (obtido via RTC DS3231) */
    logger_level_t level;              /**< Nível da mensagem */
    char message[LOGGER_MSG_MAX_SIZE]; /**< Mensagem de log */
} LoggerEntry_t;

/**
 * @brief Tipo de callback para notificações de alerta.
 *
 * Essa função será chamada sempre que um novo alerta for registrado.
 *
 * @param entry Ponteiro para a entrada de log recém registrada.
 */
typedef void (*logger_alert_callback_t)(const LoggerEntry_t *entry);

/**
 * @brief Inicializa o módulo de log.
 *
 * Inicializa o buffer de log, os recursos de sincronização, carrega a configuração do arquivo
 * "config.ini" (utilizado pelo módulo sd_storage_module), configura o RTC DS3231 e inicia as tarefas
 * de flush periódico, gravação assíncrona e monitoramento.
 *
 * @return true se a inicialização for bem-sucedida, false caso contrário.
 */
bool logger_module_init(void);

/**
 * @brief Registra uma mensagem de log.
 *
 * Se o nível da mensagem for inferior ao nível mínimo configurado, a mensagem é descartada.
 * Utiliza o RTC DS3231 para obter um timestamp sincronizado.
 *
 * @param level Nível da mensagem (LOGGER_LEVEL_INFO, LOGGER_LEVEL_WARNING ou LOGGER_LEVEL_CRITICAL).
 * @param format String de formato (estilo printf).
 * @param ... Argumentos para formatação.
 */
void logger_module_log(logger_level_t level, const char *format, ...);

/**
 * @brief Registra uma mensagem de alerta.
 *
 * Prepara a mensagem com o prefixo "ALERTA:" e a registra com o nível informado.
 *
 * @param level Nível do alerta (LOGGER_LEVEL_WARNING ou LOGGER_LEVEL_CRITICAL).
 * @param message Mensagem descritiva do alerta.
 */
void logger_module_log_alert(logger_level_t level, const char *message);

/**
 * @brief Registra um callback para notificações de alerta.
 *
 * Permite que módulos externos sejam notificados imediatamente quando um alerta for registrado.
 *
 * @param callback Função callback que receberá a entrada de log de alerta.
 */
void logger_module_register_alert_callback(logger_alert_callback_t callback);

/**
 * @brief Imprime todas as entradas de log armazenadas no buffer.
 */
void logger_module_print_logs(void);

/**
 * @brief Envia os logs armazenados para um sistema externo.
 *
 * Essa função é um placeholder para integração com protocolos como MQTT.
 */
void logger_module_send_logs(void);

/**
 * @brief Define o nível mínimo de log a ser registrado.
 *
 * Mensagens com nível inferior ao definido serão descartadas.
 *
 * @param level Nível mínimo de log.
 */
void logger_module_set_log_level(logger_level_t level);

/**
 * @brief Configura o diretório padrão para gravação dos logs no SD Card.
 *
 * Este diretório será utilizado pelas funções de salvamento para persistência dos logs.
 *
 * @param dirname Nome do diretório.
 */
void logger_module_set_sd_directory(const char *dirname);

/**
 * @brief Salva todas as entradas de log do buffer no SD Card.
 *
 * Itera sobre o buffer interno e utiliza o mecanismo de gravação com rotação do módulo de
 * armazenamento em SD Card para persistir os logs.
 *
 * @return true se os logs forem salvos com sucesso, false caso contrário.
 */
bool logger_module_save_logs_to_sd(void);

/**
 * @brief Carrega as configurações do módulo de log a partir do arquivo "config.ini".
 *
 * Lê os parâmetros de configuração, incluindo as chaves RTC_SDA, RTC_SCL, RTC_I2C_PORT e
 * MAX_LOG_FILE_SIZE, do arquivo "config.ini" localizado no diretório MOUNT_POINT.
 *
 * @return true se a configuração for carregada com sucesso, false caso contrário.
 */
bool logger_module_load_config(void);

/**
 * @brief Configura o RTC DS3231 utilizando a biblioteca ds3231.h.
 *
 * Inicializa a comunicação I2C com o RTC DS3231 utilizando os parâmetros obtidos do arquivo de configuração.
 *
 * @return true se o RTC for configurado com sucesso, false caso contrário.
 */
bool logger_module_configure_rtc(void);

/**
 * @brief Obtém um timestamp sincronizado com o RTC DS3231.
 *
 * Utiliza a biblioteca ds3231.h para obter o horário atual e converte para timestamp
 * (milissegundos desde a época Unix). Em caso de falha, retorna 0.
 *
 * @return uint32_t Timestamp em milissegundos.
 */
uint32_t logger_module_get_rtc_timestamp(void);

/**
 * @brief Salva os logs críticos (WARNING e CRITICAL) na NVS para preservação.
 *
 * Essa função salva os logs críticos do buffer na NVS, garantindo que dados essenciais
 * não sejam perdidos em caso de falha do sistema.
 *
 * @return true se os logs críticos forem salvos com sucesso, false caso contrário.
 */
bool logger_module_save_critical_logs_to_nvs(void);

/**
 * @brief Carrega os logs críticos salvos da NVS.
 *
 * @return true se os logs críticos forem carregados com sucesso, false caso contrário.
 */
bool logger_module_load_critical_logs_from_nvs(void);

/**
 * @brief Inicia a tarefa de flush periódico dos logs críticos para NVS.
 *
 * Cria uma tarefa que periodicamente salva os logs críticos na NVS para preservação.
 */
void logger_module_start_persistent_flush_task(void);

/**
 * @brief Inicia a tarefa de gravação assíncrona dos logs no SD Card.
 *
 * Cria uma tarefa que processa a fila de requisições de gravação assíncrona.
 */
void logger_module_start_async_write_task(void);

/**
 * @brief Envia uma requisição de gravação assíncrona dos logs para o SD Card.
 *
 * A requisição é enfileirada e processada pela tarefa de gravação assíncrona.
 *
 * @param data Dados a serem gravados.
 * @return true se a requisição for enfileirada com sucesso, false caso contrário.
 */
bool logger_module_async_write(const char *data);

/**
 * @brief Exporta os logs armazenados no buffer para um arquivo CSV.
 *
 * Utiliza as funções do módulo SD Storage para salvar os logs em formato CSV,
 * facilitando a integração com ferramentas externas.
 *
 * @return true se os logs forem exportados com sucesso, false caso contrário.
 */
bool logger_module_export_logs_csv(void);

/**
 * @brief Exporta os logs armazenados no buffer para um arquivo JSON.
 *
 * Utiliza as funções do módulo SD Storage para salvar os logs em formato JSON,
 * facilitando a integração com ferramentas externas.
 *
 * @return true se os logs forem exportados com sucesso, false caso contrário.
 */
bool logger_module_export_logs_json(void);

/**
 * @brief Atualiza dinamicamente o tamanho máximo dos arquivos de log.
 *
 * Permite alterar o parâmetro MAX_LOG_FILE_SIZE sem reinicializar a ECU.
 *
 * @param max_size Novo tamanho máximo do arquivo de log em bytes.
 */
void logger_module_set_max_log_file_size(uint32_t max_size);

#ifdef __cplusplus
}
#endif

#endif /* LOGGER_MODULE_H */
