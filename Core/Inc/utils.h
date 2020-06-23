#if !defined(_UTILS_H_)
#define _UTILS_H_

#include "stm32f1xx_hal.h"

#define GET_ARRAY_LENGEH(array) (sizeof(array) / sizeof(array[0]))

int constrain_int(int x, int a, int b);
float constrain_float(float x, float a, float b);
void bubble_sort(uint16_t arr[], uint16_t len);
float map(float x, float in_min, float in_max, float out_min, float out_max);

int setbit(int num, int bit);
int clearbit(int num, int bit);
bool testbit(int num, int bit);

#endif // _UTILS_H_
