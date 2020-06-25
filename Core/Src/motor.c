#include "motor.h"
#include "utils.h"
#include <stdlib.h>

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

// PWM speed control private variables
TIM_HandleTypeDef TIM1_Handle;
int16_t base_speed_ = 0;
int16_t turnning_speed_ = 0;

void Motor_Init(void)
{
    Motor_GPIOInit();
    Motor_PWMConfiguration();
    Motor_EncoderInit();

    // Stop motor
    Motor_Control(0, 0);
}

void Motor_GPIOInit(void)
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
}

void Motor_ControlLeftMotor(int16_t speed)
{
    GPIO_PinState state = (GPIO_PinState)(speed > 0);
    uint16_t abs_speed = (uint16_t)abs(speed);

    if (abs_speed > _MOTOR_PWM_MAX_)
        abs_speed = _MOTOR_PWM_MAX_;

    __HAL_TIM_SET_COMPARE(&TIM1_Handle, TIM_CHANNEL_1, abs_speed);
    HAL_GPIO_WritePin(MOTOR_L_CTLA_PORT, MOTOR_L_CTLA_PIN, state);
    HAL_GPIO_WritePin(MOTOR_L_CTLB_PORT, MOTOR_L_CTLB_PIN, (GPIO_PinState)!state);
}

void Motor_ControlRightMotor(int16_t speed)
{
    GPIO_PinState state = (GPIO_PinState)(speed > 0);
    uint16_t abs_speed = (uint16_t)abs(speed);

    if (abs_speed > _MOTOR_PWM_MAX_)
        abs_speed = _MOTOR_PWM_MAX_;

    __HAL_TIM_SET_COMPARE(&TIM1_Handle, TIM_CHANNEL_4, abs_speed);
    HAL_GPIO_WritePin(MOTOR_R_CTLA_PORT, MOTOR_R_CTLA_PIN, state);
    HAL_GPIO_WritePin(MOTOR_R_CTLB_PORT, MOTOR_R_CTLB_PIN, (GPIO_PinState)!state);
}

void Motor_PWMConfiguration(void)
{
    // TIM1 CH1-PA8, CH4-PA11, 10KHz, Fill range from 0 to 99.
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_OC_InitTypeDef TIM_OCInitStructure;

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Pin = GPIO_PIN_11 | GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM1_Handle.Instance = TIM1;
    TIM1_Handle.Init.Prescaler = 72 - 1;
    TIM1_Handle.Init.Period = _MOTOR_PWM_MAX_ - 1;
    TIM1_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM1_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM1_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&TIM1_Handle);

    TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_RESET;
    TIM_OCInitStructure.OCFastMode = TIM_OCFAST_ENABLE;
    TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
    TIM_OCInitStructure.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    TIM_OCInitStructure.OCNIdleState = TIM_OCIDLESTATE_RESET;
    TIM_OCInitStructure.Pulse = 0;
    HAL_TIM_PWM_ConfigChannel(&TIM1_Handle, &TIM_OCInitStructure, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&TIM1_Handle, &TIM_OCInitStructure, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&TIM1_Handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TIM1_Handle, TIM_CHANNEL_4);
}

void Motor_Control(int16_t base_speed, int16_t turnning_speed)
{
    base_speed_ = base_speed;
    turnning_speed_ = turnning_speed;

    Motor_ControlLeftMotor(base_speed_ + turnning_speed_);
    Motor_ControlRightMotor(base_speed_ - turnning_speed_);
}

int16_t Motor_GetBaseSpeed(void) { return base_speed_; }
int16_t Motor_GetTurnningSpeed(void) { return turnning_speed_; }

TIM_HandleTypeDef TIM4_Handle;
TIM_HandleTypeDef TIM2_Handle;

void Motor_EncoderInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_Encoder_InitTypeDef TIM_EncoderInitStructure;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();

    // TIM4 CH1-PB6 CH2-PB7
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    // TIM2 Pin Remap -> (Partial remap (CH1/ETR/PA15, CH2/PB3, CH3/PA2, CH4/PA3))
    __HAL_AFIO_REMAP_TIM2_PARTIAL_1();

    // TIM2 CH1E-PA15
    GPIO_InitStructure.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
    // TIM2 CH2-PB3
    GPIO_InitStructure.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    // TIM4 Encoder config
    TIM4_Handle.Instance = TIM4;
    TIM4_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM4_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM4_Handle.Init.Prescaler = 0;
    TIM4_Handle.Init.Period = 0xFFFF;
    TIM4_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    TIM_EncoderInitStructure.EncoderMode = TIM_ENCODERMODE_TI12;
    TIM_EncoderInitStructure.IC1Filter = 0;
    TIM_EncoderInitStructure.IC1Polarity = TIM_ICPOLARITY_BOTHEDGE;
    TIM_EncoderInitStructure.IC1Prescaler = TIM_ICPSC_DIV1;
    TIM_EncoderInitStructure.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    TIM_EncoderInitStructure.IC2Filter = 0;
    TIM_EncoderInitStructure.IC2Polarity = TIM_ICPOLARITY_BOTHEDGE;
    TIM_EncoderInitStructure.IC2Prescaler = TIM_ICPSC_DIV1;
    TIM_EncoderInitStructure.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    HAL_TIM_Encoder_Init(&TIM4_Handle, &TIM_EncoderInitStructure);
    HAL_TIM_Encoder_Start(&TIM4_Handle, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&TIM4_Handle, TIM_CHANNEL_2);

    // TIM2 Encoder config
    TIM2_Handle.Instance = TIM2;
    TIM2_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM2_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM2_Handle.Init.Prescaler = 0;
    TIM2_Handle.Init.Period = 0xFFFF;
    TIM2_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    TIM_EncoderInitStructure.EncoderMode = TIM_ENCODERMODE_TI12;
    TIM_EncoderInitStructure.IC1Filter = 0;
    TIM_EncoderInitStructure.IC1Polarity = TIM_ICPOLARITY_BOTHEDGE;
    TIM_EncoderInitStructure.IC1Prescaler = TIM_ICPSC_DIV1;
    TIM_EncoderInitStructure.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    TIM_EncoderInitStructure.IC2Filter = 0;
    TIM_EncoderInitStructure.IC2Polarity = TIM_ICPOLARITY_BOTHEDGE;
    TIM_EncoderInitStructure.IC2Prescaler = TIM_ICPSC_DIV1;
    TIM_EncoderInitStructure.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    HAL_TIM_Encoder_Init(&TIM2_Handle, &TIM_EncoderInitStructure);
    HAL_TIM_Encoder_Start(&TIM2_Handle, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&TIM2_Handle, TIM_CHANNEL_2);
}

int16_t Motor_EncoderReadLeft(void)
{
    // this side need to reverse sign to fit the real direction
    return -(int16_t)__HAL_TIM_GET_COUNTER(&TIM4_Handle);
}

int16_t Motor_EncoderReadRight(void)
{
    return (int16_t)__HAL_TIM_GET_COUNTER(&TIM2_Handle);
}
