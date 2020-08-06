#if !defined(_CONTROL_H_)
#define _CONTROL_H_

#include "stm32f1xx_hal.h"

// maximum operation angle
#define _CONTROL_MAXIMUM_ANGLE 35
#define _CONTROL_ENCODER_SAMPLE_RATE 200.0 

#define _CONTROL_MAX_SPEED_ 700
#define _CONTROL_MAX_TURNNING_ 300

// restandup after _CONTORL_RESTANDUP_DELAY ms
#define _CONTORL_RESTANDUP_DELAY 500

void Control_UpdateTurnningSpeed(int16_t speed);
void Control_UpdateMoveSpeed(int16_t speed);
void Control_EnterIdleState(void);

#endif // _CONTROL_H_
