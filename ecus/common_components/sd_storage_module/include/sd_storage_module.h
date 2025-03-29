/*
 * sd_storage_module.h
 * Cabeçalho do Módulo de Armazenamento em SD Card para a ECU de Monitoramento e Diagnóstico.
 * Projeto acadêmico para Mestrado Profissional em Engenharia Elétrica - UnB.
 * Adaptado para conformidade com MISRA C:2012 e padrões automotivos (ISO 11898).
 */

#ifndef SD_STORAGE_MODULE_H
#define SD_STORAGE_MODULE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Diretório de montagem do SD Card */
#define MOUNT_POINT              "/sdcard"
/* Tamanho máximo para nomes de arquivos e diretórios */
#define MAX_FILENAME_LENGTH      64U
/* Tamanho máximo para linhas do arquivo de configuração */
#define MAX_CONFIG_LINE_LENGTH   128U
/* Caminho do arquivo de configuração */
#define CONFIG_FILE_PATH         "/sdcard/config.ini"
/* Período de monitoramento (em milissegundos) */
#define MONITOR_PERIOD_MS        30000U
/* Nome do arquivo de teste para verificação do SD Card */
#define TEST_FILENAME            "test.txt"
/* Timeout para o Watchdog (em segundos) */
#define WDT_TIMEOUT_SECONDS      10U
/* Extensão padrão para arquivos de log */
#define LOG_FILE_EXT             ".txt"
/* Tamanho máximo padrão para um arquivo de log (em bytes) */
#define DEFAULT_MAX_LOG_FILE_SIZE 10240U
/* Tamanho máximo para dados de requisição assíncrona */
#define ASYNC_WRITE_MAX_DATA_LENGTH 256U
/* Limite crítico padrão de espaço livre (em bytes) */
#define FREE_SPACE_THRESHOLD_DEFAULT (50U * 1024U)

/* GPIOs SPI interface */
#define DEFAULT_MOSI_PIN_GPIO   (23U)
#define DEFAULT_MISO_PIN_GPIO   (19U)
#define DEFAULT_SCLK_PIN_GPIO   (18U)
#define DEFAULT_CS_PIN_GPIO     (5U)

/**
 * @brief Estrutura de configuração do SD Card (lida via arquivo INI)
 */

typedef struct {
    int mosi_pin;
    int miso_pin;
    int sclk_pin;
    int cs_pin;
    uint32_t max_log_file_size;
    uint32_t free_space_threshold;
} sd_config_t;

/**
 * @brief Estrutura para requisições de gravação assíncrona.
 */
typedef struct {
    char dirname[MAX_FILENAME_LENGTH];
    char file_prefix[MAX_FILENAME_LENGTH];
    char data[ASYNC_WRITE_MAX_DATA_LENGTH];
} sd_async_write_req_t;

/**
 * @brief Callback para notificação de gravação no SD Card.
 *
 * @param filename Nome do arquivo onde os dados foram gravados.
 * @param data Dados gravados.
 */
typedef void (*sd_storage_write_callback_t)(const char *filename, const char *data);

/**
 * @brief Callback para alerta de espaço livre crítico.
 *
 * @param free_space Espaço livre atual (em bytes).
 */
typedef void (*sd_storage_free_space_callback_t)(uint32_t free_space);

/*==============================================================================
                   FUNÇÕES DE GERENCIAMENTO DO SD CARD
 ==============================================================================*/

/**
 * @brief Inicializa o módulo de armazenamento em SD Card.
 *
 * Configura o barramento SPI, monta o sistema de arquivos e inicializa os recursos internos.
 *
 * @return true se a inicialização for bem-sucedida, false caso contrário.
 */
bool sd_storage_module_init(void);

/**
 * @brief Grava dados no SD Card em um arquivo especificado.
 *
 * Abre o arquivo em modo "append" e grava a string fornecida, seguida de uma nova linha.
 *
 * @param filename Nome do arquivo.
 * @param data Dados a serem gravados.
 * @return true se a gravação for bem-sucedida, false caso contrário.
 */
bool sd_storage_module_write(const char *filename, const char *data);

/**
 * @brief Lê um arquivo do SD Card e retorna seu conteúdo.
 *
 * Abre o arquivo em modo leitura e copia o conteúdo para o buffer fornecido.
 *
 * @param filename Nome do arquivo.
 * @param buffer Buffer onde os dados lidos serão armazenados.
 * @param max_len Tamanho máximo do buffer.
 * @return true se a leitura for bem-sucedida, false caso contrário.
 */
bool sd_storage_module_read(const char *filename, char *buffer, size_t max_len);

/**
 * @brief Finaliza o uso do SD Card e desmonta o sistema de arquivos.
 */
void sd_storage_module_deinit(void);

/**
 * @brief Inicia a tarefa de monitoramento do SD Card.
 *
 * Cria uma tarefa que verifica periodicamente o estado do SD Card, o espaço livre e realiza a limpeza automática
 * dos arquivos de log antigos, se necessário.
 */
void sd_storage_module_start_monitor_task(void);

/**
 * @brief Inicia a tarefa do watchdog do sistema ESP32.
 *
 * Cria uma tarefa que alimenta periodicamente o watchdog, utilizando o Task Watchdog e o RTC WDT.
 */
void sd_storage_module_start_watchdog_task(void);

/*==============================================================================
         FUNÇÕES DE GERENCIAMENTO DE DIRETÓRIOS, ROTAÇÃO E GRAVAÇÃO ASSÍNCRONA
 ==============================================================================*/

/**
 * @brief Cria um diretório no SD Card se ele não existir.
 *
 * @param dirname Nome do diretório a ser criado (relativo ao MOUNT_POINT).
 * @return true se o diretório foi criado ou já existe, false caso contrário.
 */
bool sd_storage_module_create_directory(const char *dirname);

/**
 * @brief Define o tamanho máximo configurável para um arquivo de log.
 *
 * Se o arquivo exceder esse tamanho, um novo arquivo será criado automaticamente (log rotation).
 *
 * @param max_size Tamanho máximo do arquivo em bytes.
 */
void sd_storage_module_set_max_file_size(uint32_t max_size);

/**
 * @brief Grava dados em um arquivo com rotação automática.
 *
 * Se o arquivo atual exceder o tamanho máximo configurado, um novo arquivo é criado automaticamente
 * com um sufixo baseado no timestamp.
 *
 * @param dirname Diretório onde os dados serão gravados.
 * @param file_prefix Prefixo para o nome do arquivo.
 * @param data Dados a serem gravados.
 * @return true se a gravação for bem-sucedida, false caso contrário.
 */
bool sd_storage_module_write_with_rotation(const char *dirname, const char *file_prefix, const char *data);

/**
 * @brief Envia uma requisição de gravação assíncrona.
 *
 * A requisição é enfileirada e processada por uma tarefa dedicada.
 *
 * @param dirname Diretório onde os dados serão gravados. Se NULL, o diretório padrão será usado.
 * @param file_prefix Prefixo para o nome do arquivo.
 * @param data Dados a serem gravados.
 * @return true se a requisição for enfileirada com sucesso, false caso contrário.
 */
bool sd_storage_module_async_write(const char *dirname, const char *file_prefix, const char *data);

/**
 * @brief Inicia a tarefa de gravação assíncrona.
 */
void sd_storage_module_start_async_write_task(void);

/**
 * @brief Define o limite crítico de espaço livre no SD Card.
 *
 * Se o espaço livre for inferior a esse valor (em bytes), um alerta será disparado.
 *
 * @param free_space_threshold Limite crítico em bytes.
 */
void sd_storage_module_set_free_space_threshold(uint32_t free_space_threshold);

/**
 * @brief Registra um callback para alerta de espaço livre crítico.
 *
 * O callback é chamado quando o espaço livre no SD Card for inferior ao limiar configurado.
 *
 * @param callback Função callback que recebe o espaço livre atual (em bytes).
 */
void sd_storage_module_register_free_space_callback(sd_storage_free_space_callback_t callback);

/**
 * @brief Registra um callback para notificação de gravação.
 *
 * O callback é invocado sempre que uma nova gravação for concluída com sucesso.
 *
 * @param callback Função callback que recebe o nome do arquivo e os dados gravados.
 */
void sd_storage_module_register_write_callback(sd_storage_write_callback_t callback);

/**
 * @brief Define dinamicamente um diretório padrão para as gravações.
 *
 * Se o parâmetro dirname não for nulo, este diretório será usado como padrão para operações de escrita
 * quando nenhum diretório específico for fornecido.
 *
 * @param dirname Nome do diretório padrão.
 */
void sd_storage_module_set_default_directory(const char *dirname);

/**
 * @brief Realiza a limpeza automática dos arquivos de log antigos.
 *
 * Percorre o diretório especificado e remove os arquivos mais antigos até que o espaço livre seja superior
 * ao limiar crítico configurado.
 *
 * @param dirname Diretório onde os arquivos de log estão armazenados.
 * @return true se a limpeza for bem-sucedida, false caso contrário.
 */
bool sd_storage_module_cleanup_logs(const char *dirname);

/*==============================================================================
        NOVAS FUNÇÕES PARA TIMESTAMPS E FORMATOS PADRONIZADOS
 ==============================================================================*/

/**
 * @brief Obtém o timestamp atual formatado.
 *
 * O timestamp é obtido a partir do RTC sincronizado e formatado no padrão "YYYY-MM-DD HH:MM:SS".
 *
 * @param buffer Buffer onde o timestamp formatado será armazenado.
 * @param max_len Tamanho máximo do buffer.
 * @return true se o timestamp for obtido com sucesso, false caso contrário.
 */
bool sd_storage_module_get_formatted_timestamp(char *buffer, size_t max_len);

/**
 * @brief Grava uma entrada de log em formato CSV.
 *
 * A função formata a entrada como CSV, prefixando com o timestamp atual, e utiliza a rotação automática de arquivos.
 *
 * @param dirname Diretório onde os dados serão gravados.
 * @param file_prefix Prefixo para o nome do arquivo.
 * @param csv_entry Entrada de log em formato CSV (sem o timestamp).
 * @return true se a gravação for bem-sucedida, false caso contrário.
 */
bool sd_storage_module_write_csv(const char *dirname, const char *file_prefix, const char *csv_entry);

/**
 * @brief Grava uma entrada de log em formato JSON.
 *
 * A função encapsula a entrada de log em um objeto JSON, incluindo o timestamp atual, e utiliza a rotação automática de arquivos.
 *
 * @param dirname Diretório onde os dados serão gravados.
 * @param file_prefix Prefixo para o nome do arquivo.
 * @param json_entry Entrada de log em formato JSON (sem o campo timestamp).
 * @return true se a gravação for bem-sucedida, false caso contrário.
 */
bool sd_storage_module_write_json(const char *dirname, const char *file_prefix, const char *json_entry);

#ifdef __cplusplus
}
#endif

#endif /* SD_STORAGE_MODULE_H */
