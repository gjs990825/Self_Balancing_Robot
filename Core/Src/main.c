#include "main.h"
#include "bsp.h"
#include "debug.h"
#include "motor.h"
#include "uart.h"
#include <stdio.h>


int main(void)
{
    BSP_Config();
    LED_GPIO_Init();
    USART1_UART_Init();

    Motor_Init();

    bool dir = 0;

    while (1)
    {
        LED_Toggle();
        Motor_Go(dir);
        printf((dir ? "go\r\n" : "back\r\n"));
        dir = !dir;

        HAL_Delay(2000);
    }
}
