#if !defined(_MOTOR_H_)
#define _MOTOR_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h>

#define _MOTOR_PWM_MAX_ 100

// Motor direction control
void Motor_Init(void);

// Motor speed control
void Motor_Control(int16_t speed, int16_t turnningSpeed);

// Motor information
int16_t Motor_GetBaseSpeed(void);
int16_t Motor_GetTurnningSpeed(void);

#endif // _MOTOR_H_
