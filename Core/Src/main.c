#include "main.h"
#include "bsp.h"
#include "debug.h"
#include "motor.h"
#include "uart.h"
#include <stdio.h>

int main(void)
{
    BSP_Config();

    LED_GPIOInit();
    USART1_UARTInit();
    Motor_Init();

    bool dir = 1;
    int16_t t_speed;
    while (1)
    {
        for (int16_t j = 0; j < 2; j++)
        {
            for (uint16_t i = 0; i < 100; i++)
            {
                dir ? t_speed++ : t_speed--;
                Motor_Control(t_speed, 0);
                HAL_Delay(10);
            }
            dir = !dir;
        }
        dir = !dir;

        LED_Toggle();
        // HAL_Delay(500);
    }
}
