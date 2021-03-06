#include "debug.h"
#include "stm32f1xx_hal.h"
#include "led.h"

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

    log_error("System error!\r\n");
    for (;;)
    {
        LED_Toggle();
        HAL_Delay(50);
    }

    /* USER CODE END Error_Handler_Debug */
}

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(unsigned char *file, unsigned int line)
{
    log_error("Wrong parameters value: file %s on line %d\r\n", file, line);
    for (;;)
    {
        LED_Toggle();
        HAL_Delay(50);
    }
}
