#if !defined(_UART_H_)
#define _UART_H_

#include "stm32f1xx_hal.h"

extern UART_HandleTypeDef UartHandle;

void USART1_UARTInit(void);
char UART_read(void);
void UART_write(char);

#endif // _UART_H_
