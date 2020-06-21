#include "main.h"
#include "bsp.h"


int main(void)
{
    BSP_Config();

    LED_GPIO_Init();
    

    while (1)
    {
        HAL_Delay(500);
        LED_Toggle();
    }
}
