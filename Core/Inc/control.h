#if !defined(_CONTROL_H_)
#define _CONTROL_H_

#include "stm32f1xx_hal.h"

void Control_UpdateTurnningSpeed(int16_t speed);
void Control_UpdateTargetSpeed(int16_t speed);

#endif // _CONTROL_H_
