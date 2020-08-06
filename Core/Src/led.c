#include "led.h"
#include <stdbool.h>

struct _led_blink_status _sysfault_blink_status[] = {
    {SET, 100},
    {RESET, 100},
    {SET, 100},
    {RESET, 100},
    {SET, 300},
    {RESET, 100},
    {SET, 300},
    {RESET, 100},
};

blink_sequence_t _system_fault_blink_sequence =
    {_sysfault_blink_status,
     sizeof(_sysfault_blink_status) / sizeof(struct _led_blink_status)};

#define LED0_Pin GPIO_PIN_13
#define LED0_GPIO_Port GPIOC
#define LED0_CLK_ENABLE __HAL_RCC_GPIOC_CLK_ENABLE

void LED_GPIOInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    LED0_CLK_ENABLE();
    LED_SetStatus(false);

    GPIO_InitStruct.Pin = LED0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);
}

void LED_Toggle(void)
{
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
}

void LED_SetStatus(bool state)
{
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, (GPIO_PinState)state);
}

void LED_BlinkSequence(blink_sequence_t sequence)
{
    for (int i = 0; i < sequence.status_count; i++)
    {
        LED_SetStatus(sequence.blink_status[i].status);
        HAL_Delay(sequence.blink_status[i].delay);
    }
}
