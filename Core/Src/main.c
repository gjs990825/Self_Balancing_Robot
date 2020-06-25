#include "main.h"
#include "bsp.h"
#include "debug.h"
#include "motor.h"
#include "uart.h"
#include <stdio.h>
#include "mpu6050.h"


int main(void)
{
    BSP_Config();
    USART1_UARTInit();
    LED_GPIOInit();
    Motor_Init();

    MPU6050_Init();

    printf("\r\nSystem Setup OK\r\n");

    // bool dir = 1;
    // int16_t t_speed;
    while (1)
    {
        // // test PWM
        // for (int16_t j = 0; j < 2; j++)
        // {
        //     for (uint16_t i = 0; i < 100; i++)
        //     {
        //         dir ? t_speed++ : t_speed--;
        //         Motor_Control(t_speed, 0);
        //         HAL_Delay(10);
        //     }
        //     dir = !dir;
        // }
        // dir = !dir;

        LED_Toggle();
        // printf("Encoder: L:%5d\tR:%5d\r\n", Motor_EncoderReadLeft(), Motor_EncoderReadRight());
        printf("Gyro_X:%5d\tAccel_Y:%5d\r\n", MPU6050_GetGyroX(), MPU6050_GetAccelY());
        HAL_Delay(100);
    }
}
