/**
 * @file motor_control_ecu.h
 * @brief Módulo de controle do motor para veículos elétricos.
 *
 * Este módulo implementa as funções de controle do motor, incluindo o gerenciamento
 * de mensagens CAN conforme a norma ISO 11898. O código está em conformidade com MISRA C:2012.
 *
 * @note Este componente destina-se à aplicação em veículos elétricos reais.
 */

#ifndef MOTOR_CONTROL_ECU_H
#define MOTOR_CONTROL_ECU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Estrutura de mensagem CAN.
 *
 * Representa uma mensagem CAN conforme a norma ISO 11898.
 */
typedef struct
{
    uint32_t id;          /**< Identificador da mensagem CAN */
    uint8_t dlc;          /**< Código de tamanho dos dados (Data Length Code) */
    uint8_t data[8];      /**< Campo de dados */
} CAN_Message_t;

/**
 * @brief Códigos de erro do controle do motor.
 */
typedef enum
{
    MOTOR_CONTROL_OK = 0U,           /**< Sem erro */
    MOTOR_CONTROL_ERROR_OVERHEAT,    /**< Sobreaquecimento do motor */
    MOTOR_CONTROL_ERROR_OVERCURRENT, /**< Sobrecorrente no motor */
    MOTOR_CONTROL_ERROR_CAN,         /**< Erro de comunicação CAN */
    MOTOR_CONTROL_ERROR_UNKNOWN      /**< Erro desconhecido */
} MotorControl_Error_t;

/**
 * @brief Estados do motor.
 */
typedef enum
{
    MOTOR_STATE_OFF = 0U,  /**< Motor desligado */
    MOTOR_STATE_ON,        /**< Motor ligado */
    MOTOR_STATE_FAULT      /**< Motor em condição de falha */
} MotorControl_State_t;

/**
 * @brief Inicializa o módulo Motor Control ECU.
 *
 * Configura as variáveis internas, a comunicação CAN e os parâmetros do controle do motor.
 */
void MotorControl_ECU_Init(void);

/**
 * @brief Define a velocidade desejada do motor.
 *
 * Atualiza a velocidade alvo para o motor. Se o valor for maior que zero,
 * o estado do motor é definido como ligado.
 *
 * @param speed Velocidade desejada (em RPM).
 */
void MotorControl_ECU_SetSpeed(uint16_t speed);

/**
 * @brief Obtém a velocidade atual do motor.
 *
 * Retorna a velocidade medida do motor.
 *
 * @return uint16_t Velocidade atual (em RPM).
 */
uint16_t MotorControl_ECU_GetSpeed(void);

/**
 * @brief Obtém o estado atual do motor.
 *
 * Retorna o estado corrente do motor.
 *
 * @return MotorControl_State_t Estado do motor.
 */
MotorControl_State_t MotorControl_ECU_GetState(void);

/**
 * @brief Recupera o status de erro atual.
 *
 * Retorna o código de erro atual do módulo.
 *
 * @return MotorControl_Error_t Código de erro.
 */
MotorControl_Error_t MotorControl_ECU_GetError(void);

/**
 * @brief Processa uma mensagem CAN recebida.
 *
 * Esta função interpreta a mensagem CAN e atualiza os parâmetros de controle
 * do motor conforme o conteúdo recebido.
 *
 * @param msg Ponteiro para a estrutura da mensagem CAN.
 */
void MotorControl_ECU_HandleCANMessage(const CAN_Message_t *msg);

/**
 * @brief Função de atualização periódica do controle do motor.
 *
 * Deve ser chamada periodicamente (por exemplo, em um loop principal ou por um timer)
 * para realizar tarefas de monitoramento, ajuste da velocidade e detecção de falhas.
 */
void MotorControl_ECU_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_ECU_H */
