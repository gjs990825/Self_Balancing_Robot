#include "main.h"
#include "bsp.h"
#include "debug.h"

UART_HandleTypeDef huart1;

void USART1_UART_Init(void);


int main(void)
{
    BSP_Config();
    LED_GPIO_Init();
    USART1_UART_Init();
    

    while (1)
    {
        HAL_Delay(500);
        LED_Toggle();
        HAL_UART_Transmit(&huart1, "hello\r\n", 8, 100);
    }
}

void USART1_UART_Init(void)
{
    // UART Init
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}
