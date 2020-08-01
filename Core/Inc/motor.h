#if !defined(_MOTOR_H_)
#define _MOTOR_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h>

#define _MOTOR_PWM_MAX_ 1000

#define Motor_ShutDown() Motor_Control(0, 0)

// Motor basic direction control
void Motor_Init(void);
void Motor_GPIOInit(void);
void Motor_SetLeftMotorSpeed(int16_t speed);
void Motor_SetRightMotorSpeed(int16_t speed);

// Motor speed control
void Motor_PWMConfiguration(void);
void Motor_Control(int16_t speed, int16_t turnningSpeed);
int16_t Motor_GetBaseSpeed(void);
int16_t Motor_GetTurnningSpeed(void);

// Motor Encoder
void Motor_EncoderInit(void);
int16_t Motor_EncoderReadLeft(void);
int16_t Motor_EncoderReadRight(void);
int Motor_GetSpeed(void);

#endif // _MOTOR_H_
