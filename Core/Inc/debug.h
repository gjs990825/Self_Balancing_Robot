#if !defined(_DEBUG_H_)
#define _DEBUG_H_

#include <stdio.h>
#include <stdarg.h>
#include "stm32f1xx_hal.h"

// Choosing debug output
#define _ENABLE_DEBUG_INFO_OUTPUT_
#define _ENABLE_DEBUG_ERROR_OUTPUT_

#if defined(_ENABLE_DEBUG_INFO_OUTPUT_)
#define log_info printf
#else
static void log_info(const char *__restrict __fmt, ...){};
#endif // _ENABLE_DEBUG_INFO_OUTPUT_

#if defined(_ENABLE_DEBUG_ERROR_OUTPUT_)
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
