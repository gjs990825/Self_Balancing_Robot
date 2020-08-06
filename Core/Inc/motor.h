#if !defined(_MOTOR_H_)
#define _MOTOR_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <control.h>

#define _MOTOR_PWM_MAX_ 1000
#define _MOTOR_MAX_SPEED_ENCODER_COUNT_PER_SECOND 14500.0

#if defined(_CONTROL_H_)

#define _MOTOR_SPEED_ENCODER_RATICIAL (_MOTOR_PWM_MAX_ / (_MOTOR_MAX_SPEED_ENCODER_COUNT_PER_SECOND / _CONTROL_ENCODER_SAMPLE_RATE))

#endif // _CONTROL_H_


// Motor basic direction control
void Motor_Init(void);
void Motor_GPIOInit(void);
void Motor_SetLeftMotorSpeed(int16_t speed);
void Motor_SetRightMotorSpeed(int16_t speed);

// Motor speed control
void Motor_PWMConfiguration(void);
void Motor_Control(int16_t move_speed, int16_t turnningSpeed);
int16_t Motor_GetBaseSpeed(void);
int16_t Motor_GetTurnningSpeed(void);
#define Motor_ShutDown() Motor_Control(0, 0)

// Motor Encoder
void Motor_EncoderInit(void);
int16_t Motor_EncoderReadLeft(void);
int16_t Motor_EncoderReadRight(void);
int16_t Motor_GetEncoder(void);
float Motor_GetSpeed(void);

#endif // _MOTOR_H_
