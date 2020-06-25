#if !defined(_MPU6050_H)
#define _MPU6050_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>

// MPU6050 configuration
void MPU6050_Init(void);
void MPU6050_IICConfig(void);
bool MPU6050_Check(void);

// MPU6050 data collection
int16_t MPU6050_GetData(uint8_t addr);
int16_t MPU6050_GetGyroX(void);
int16_t MPU6050_GetAccelY(void);


#endif // _MPU6050_H
