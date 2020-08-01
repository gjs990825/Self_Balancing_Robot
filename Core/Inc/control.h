#if !defined(_CONTROL_H_)
#define _CONTROL_H_

#include "stm32f1xx_hal.h"

// maximum operation angle
#define _MAX_ANGLE_ 35

// re standup after _RE_STANDUP_DELAY_ ms
#define _RE_STANDUP_DELAY_ 500

void Control_UpdateTurnningSpeed(int16_t speed);
void Control_UpdateMoveSpeed(int16_t speed);
void Control_EnterIdleState(void);

#endif // _CONTROL_H_
