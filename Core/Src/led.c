#include "led.h"
#include <stdbool.h>


#define LED0_Pin GPIO_PIN_13
#define LED0_GPIO_Port GPIOC
#define LED0_CLK_ENABLE __HAL_RCC_GPIOC_CLK_ENABLE

void LED_GPIOInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    LED0_CLK_ENABLE();
    LED_Set(false);

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

void LED_Set(bool state)
{
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, (GPIO_PinState)state);
}
