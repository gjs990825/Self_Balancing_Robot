#include "motor.h"

#define MOTOR_L_CTLA_PORT GPIOB
#define MOTOR_L_CTLA_PIN GPIO_PIN_14
#define MOTOR_L_CTLA_CLKEN __HAL_RCC_GPIOB_CLK_ENABLE

#define MOTOR_L_CTLB_PORT GPIOA
#define MOTOR_L_CTLB_PIN GPIO_PIN_12
#define MOTOR_L_CTLB_CLKEN __HAL_RCC_GPIOA_CLK_ENABLE

#define MOTOR_R_CTLA_PORT GPIOB
#define MOTOR_R_CTLA_PIN GPIO_PIN_15
#define MOTOR_R_CTLA_CLKEN __HAL_RCC_GPIOB_CLK_ENABLE

#define MOTOR_R_CTLB_PORT GPIOB
#define MOTOR_R_CTLB_PIN GPIO_PIN_4
#define MOTOR_R_CTLB_CLKEN __HAL_RCC_GPIOB_CLK_ENABLE


void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    MOTOR_L_CTLA_CLKEN();
    MOTOR_L_CTLB_CLKEN();
    MOTOR_R_CTLA_CLKEN();
    MOTOR_R_CTLB_CLKEN();

    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStructure.Pin = MOTOR_L_CTLA_PIN;
    HAL_GPIO_Init(MOTOR_L_CTLA_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = MOTOR_L_CTLB_PIN;
    HAL_GPIO_Init(MOTOR_L_CTLB_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = MOTOR_R_CTLA_PIN;
    HAL_GPIO_Init(MOTOR_R_CTLA_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = MOTOR_R_CTLB_PIN;
    HAL_GPIO_Init(MOTOR_R_CTLB_PORT, &GPIO_InitStructure);

    HAL_GPIO_WritePin(MOTOR_L_CTLA_PORT, MOTOR_L_CTLA_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_L_CTLB_PORT, MOTOR_L_CTLB_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_CTLA_PORT, MOTOR_R_CTLA_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_CTLB_PORT, MOTOR_R_CTLB_PIN, GPIO_PIN_RESET);

    //////////////////////////////
    // pwm pin test
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStructure.Pin = GPIO_PIN_8 | GPIO_PIN_11;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_11, GPIO_PIN_SET);
}

void Motor_ControlLeftMotor(bool dir)
{
    HAL_GPIO_WritePin(MOTOR_L_CTLA_PORT, MOTOR_L_CTLA_PIN, (GPIO_PinState)dir);
    HAL_GPIO_WritePin(MOTOR_L_CTLB_PORT, MOTOR_L_CTLB_PIN, (GPIO_PinState)(!dir));
}

void Motor_ControlRightMotor(bool dir)
{
    HAL_GPIO_WritePin(MOTOR_R_CTLA_PORT, MOTOR_R_CTLA_PIN, (GPIO_PinState)dir);
    HAL_GPIO_WritePin(MOTOR_R_CTLB_PORT, MOTOR_R_CTLB_PIN, (GPIO_PinState)(!dir));
}

void Motor_Turn(bool dir)
{
    Motor_ControlLeftMotor(dir);
    Motor_ControlRightMotor(!dir);
}

void Motor_Go(bool dir)
{
    Motor_ControlLeftMotor(dir);
    Motor_ControlRightMotor(dir);
}
