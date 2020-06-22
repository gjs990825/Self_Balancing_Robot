#if !defined(_MOTOR_H_)
#define _MOTOR_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h>

void Motor_Init(void);
void Motor_Turn(bool dir);
void Motor_Go(bool dir);

#endif // _MOTOR_H_
