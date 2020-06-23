#include "utils.h"
#include <stdlib.h>
#include <math.h>

int constrain_int(int x, int a, int b)
{
	if ((x >= a) && (x <= b))
	{
		return x;
	}
	else
	{
		return (x < a) ? a : b;
	}
}

float constrain_float(float x, float a, float b)
{
	if ((x >= a) && (x <= b))
	{
		return x;
	}
	else
	{
		return (x < a) ? a : b;
	}
}

void bubble_sort(uint16_t arr[], uint16_t len)
{
	uint16_t i, j, temp;
	for (i = 0; i < len - 1; i++)
		for (j = 0; j < len - 1 - i; j++)
			if (arr[j] > arr[j + 1])
			{
				temp = arr[j];
				arr[j] = arr[j + 1];
				arr[j + 1] = temp;
			}
}

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int setbit(int num, int bit)
{
	return num |= (1 << bit);
}

int clearbit(int num, int bit)
{
	return num &= ~(1 << bit);
}

bool testbit(int num, int bit)
{
	return num &= (1 << bit);
}
