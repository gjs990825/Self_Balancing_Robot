#if !defined(_DEBUG_H_)
#define _DEBUG_H_

#include <stdio.h>
#include <stdarg.h>
#include "stm32f1xx_hal.h"
#include "led.h"

// Choosing debug output
#define _DEBUG_ENABLE_INFO_OUTPUT
#define _DEBUG_ENABLE_ERROR_OUTPUT

// #define _DEBUG_OUOTPUT_CONTROL_DATA

#if defined(_DEBUG_ENABLE_INFO_OUTPUT)
#define log_info printf
#else
static void log_info(const char *__restrict __fmt, ...){};
#endif // _ENABLE_DEBUG_INFO_OUTPUT_

#if defined(_DEBUG_ENABLE_ERROR_OUTPUT)
static void log_error(const char *__restrict __fmt, ...)
{
    va_list ap;
    va_start(ap, __fmt);

    printf("\r\n\r\nError: ");
    vprintf(__fmt, ap);
    printf("\r\n\r\n");

    va_end(ap);
};
#else
static void log_error(const char *__restrict __fmt, ...){};
#endif // _ENABLE_DEBUG_ERROR_OUTPUT_

void Error_Handler(void);

#endif // _DEBUG_H_
