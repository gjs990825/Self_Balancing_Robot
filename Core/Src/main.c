#include "main.h"
#include "debug.h"
#include "uart.h"
#include "led.h"
#include "mpu6050.h"
#include "control.h"
#include "nrf24l01.h"
#include "motor.h"
#include "communication.h"

void SystemClock_Config(void);

int main(void)
{
    // System setup
    HAL_Init();
    SystemClock_Config();

    // Basic debug tool setup
    USART1_UARTInit();
    LED_GPIOInit();
    Motor_Init();

    // Delay for external modules to start up
    HAL_Delay(300);

    // NRF24L01 module for wireless control
    NRF24L01_Init();
    log_info("NRF24L01 Module %s\r\n", NRF24L01_Check() ? "OK" : "ERROR");

    // MP6050 module for motion detection
    MPU6050_Init();
    MPU6050_DMPInit();
    MPU6050_EXTIInit();

    log_info("System Setup Finished\r\nRunning now...\r\n");

    for (;;)
    {
        Communication_CheckMessage();
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the CPU, AHB and APB busses clocks 
  */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks 
  */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}
