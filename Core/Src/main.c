#include "main.h"
#include "bsp.h"
#include "debug.h"
#include "motor.h"
#include "uart.h"
#include <stdio.h>
#include "mpu6050.h"
#include <stdlib.h>
#include <math.h>
#include "utils.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

int main(void)
{
    BSP_Config();
    USART1_UARTInit();
    LED_GPIOInit();
    Motor_Init();

    // Wait for MPU6050 to start up
    HAL_Delay(300);
    MPU6050_Init();
    MPU6050_DMPInit();
    MPU6050_EXTIInit();

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

        // LED_Toggle();
        HAL_Delay(5);
        // printf("Encoder: L:%5d\tR:%5d\r\n", Motor_EncoderReadLeft(), Motor_EncoderReadRight());
    }
}
