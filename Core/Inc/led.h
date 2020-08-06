#if !defined(_LED_H_)
#define _LED_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h>

struct _led_blink_status
{
    bool status;
    int delay;
};

typedef struct _led_blink_sequence
{
    struct _led_blink_status *blink_status;
    int status_count;
} blink_sequence_t;

extern blink_sequence_t _system_fault_blink_sequence;

void LED_GPIOInit(void);
void LED_Toggle(void);
void LED_SetStatus(bool state);

void LED_BlinkSequence(blink_sequence_t sequence);

#endif // _LED_H_
