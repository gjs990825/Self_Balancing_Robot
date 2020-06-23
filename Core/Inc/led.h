#if !defined(_LED_H_)
#define _LED_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h>

void LED_GPIOInit(void);
void LED_Toggle(void);
void LED_Set(bool state);

#endif // _LED_H_
