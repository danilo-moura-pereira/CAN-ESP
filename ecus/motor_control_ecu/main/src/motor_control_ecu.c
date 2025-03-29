/**
 * @file motor_control_ecu.c
 * @brief Implementação do módulo Motor Control ECU.
 *
 * Este arquivo contém a implementação das funções que realizam o controle do motor
 * e o gerenciamento de comunicação CAN, conforme a norma ISO 11898.
 * O código segue as diretrizes do MISRA C:2012.
 *
 * @note Destina-se à aplicação em veículos elétricos reais.
 *
 * @see motor_control_ecu.h
 */

#include "motor_control_ecu.h"

/* Definições de constantes para evitar números mágicos (MISRA C:2012) */
#define CAN_CMD_SET_SPEED   ((uint32_t)0x100U)  /**< ID CAN para comando de definição de velocidade */
#define CAN_FAULT_MSG       ((uint32_t)0x200U)  /**< ID CAN para mensagem de falha do motor */

#define MOTOR_SPEED_STEP    ((uint16_t)10U)      /**< Incremento/decremento de velocidade por atualização (RPM) */

/* Variáveis estáticas internas */
static volatile uint16_t desiredSpeed = 0U;                /**< Velocidade desejada */
static volatile uint16_t currentSpeed = 0U;                /**< Velocidade atual */
static volatile MotorControl_State_t motorState = MOTOR_STATE_OFF;  /**< Estado atual do motor */
static volatile MotorControl_Error_t motorError = MOTOR_CONTROL_OK; /**< Status de erro atual */

/* Protótipo de função interna */
static void MotorControl_ECU_ProcessFault(uint8_t faultCode);

/**
 * @brief Inicializa o módulo Motor Control ECU.
 *
 * Configura as variáveis internas e prepara a interface de comunicação com o motor.
 */
void MotorControl_ECU_Init(void)
{
    /* Inicialização das variáveis de controle */
    desiredSpeed = 0U;
    currentSpeed = 0U;
    motorState = MOTOR_STATE_OFF;
    motorError = MOTOR_CONTROL_OK;

    /* Inicialização de interfaces de hardware (ex.: CAN, PWM) deve ser implementada aqui */
    /* O código de inicialização específico do hardware não é incluído nesta implementação genérica */
}

/**
 * @brief Define a velocidade desejada do motor.
 *
 * Atualiza o valor alvo e, se necessário, altera o estado do motor para ligado.
 *
 * @param speed Velocidade desejada (em RPM).
 */
void MotorControl_ECU_SetSpeed(uint16_t speed)
{
    desiredSpeed = speed;
    if (speed > 0U)
    {
        motorState = MOTOR_STATE_ON;
    }
    else
    {
        motorState = MOTOR_STATE_OFF;
    }
}

/**
 * @brief Obtém a velocidade atual do motor.
 *
 * Retorna o valor medido da velocidade do motor.
 *
 * @return uint16_t Velocidade atual (em RPM).
 */
uint16_t MotorControl_ECU_GetSpeed(void)
{
    return currentSpeed;
}

/**
 * @brief Obtém o estado atual do motor.
 *
 * Retorna o estado operacional atual do motor.
 *
 * @return MotorControl_State_t Estado do motor.
 */
MotorControl_State_t MotorControl_ECU_GetState(void)
{
    return motorState;
}

/**
 * @brief Recupera o status de erro atual.
 *
 * Retorna o código de erro vigente no módulo de controle do motor.
 *
 * @return MotorControl_Error_t Código de erro.
 */
MotorControl_Error_t MotorControl_ECU_GetError(void)
{
    return motorError;
}

/**
 * @brief Processa uma mensagem CAN recebida.
 *
 * Interpreta e age de acordo com o conteúdo da mensagem CAN:
 * - Se o ID corresponder a CAN_CMD_SET_SPEED, extrai e define a velocidade desejada.
 * - Se o ID corresponder a CAN_FAULT_MSG, processa o código de falha recebido.
 *
 * @param msg Ponteiro para a mensagem CAN.
 */
void MotorControl_ECU_HandleCANMessage(const CAN_Message_t *msg)
{
    if (msg == (const CAN_Message_t *)0)
    {
        /* Se o ponteiro for nulo, não processa a mensagem */
        return;
    }

    switch (msg->id)
    {
        case CAN_CMD_SET_SPEED:
            if (msg->dlc >= 2U)
            {
                /* Extrai os dois primeiros bytes para compor a velocidade desejada */
                uint16_t speedValue = (uint16_t)((((uint16_t)msg->data[0]) << 8U) |
                                                  ((uint16_t)msg->data[1]));
                MotorControl_ECU_SetSpeed(speedValue);
            }
            break;

        case CAN_FAULT_MSG:
            if (msg->dlc >= 1U)
            {
                MotorControl_ECU_ProcessFault(msg->data[0]);
            }
            break;

        default:
            /* IDs CAN desconhecidos são ignorados */
            break;
    }
}

/**
 * @brief Função de atualização periódica do controle do motor.
 *
 * Ajusta a velocidade atual em direção à velocidade desejada, aplicando um incremento/decremento
 * definido, e executa tarefas periódicas de monitoramento e detecção de falhas.
 */
void MotorControl_ECU_Update(void)
{
    /* Algoritmo simples de controle: aproxima a velocidade atual da desejada */
    if (motorState == MOTOR_STATE_ON)
    {
        if (currentSpeed < desiredSpeed)
        {
            currentSpeed += MOTOR_SPEED_STEP;
            if (currentSpeed > desiredSpeed)
            {
                currentSpeed = desiredSpeed;
            }
        }
        else if (currentSpeed > desiredSpeed)
        {
            if (currentSpeed > MOTOR_SPEED_STEP)
            {
                currentSpeed -= MOTOR_SPEED_STEP;
            }
            else
            {
                currentSpeed = 0U;
            }
        }
    }
    else
    {
        /* Se o motor estiver desligado, desacelera até zero */
        if (currentSpeed > MOTOR_SPEED_STEP)
        {
            currentSpeed -= MOTOR_SPEED_STEP;
        }
        else
        {
            currentSpeed = 0U;
        }
    }

    /* Outras tarefas periódicas, como monitoramento de falhas, podem ser adicionadas aqui */
}

/**
 * @brief Processa o código de falha recebido via mensagem CAN.
 *
 * Atualiza o status de erro do módulo conforme o código de falha e
 * transita o estado do motor para FAULT se necessário.
 *
 * @param faultCode Código de falha recebido.
 */
static void MotorControl_ECU_ProcessFault(uint8_t faultCode)
{
    /* Exemplo de processamento de falhas */
    switch (faultCode)
    {
        case 0U:
            motorError = MOTOR_CONTROL_OK;
            break;
        case 1U:
            motorError = MOTOR_CONTROL_ERROR_OVERHEAT;
            break;
        case 2U:
            motorError = MOTOR_CONTROL_ERROR_OVERCURRENT;
            break;
        default:
            motorError = MOTOR_CONTROL_ERROR_UNKNOWN;
            break;
    }

    /* Se houver erro, o estado do motor é atualizado para FAULT */
    if (motorError != MOTOR_CONTROL_OK)
    {
        motorState = MOTOR_STATE_FAULT;
    }
}
