#include "mpu6050.h"
#include "debug.h"
#include "kalman_filter.h"
#include <stdio.h>
#include <math.h>

const float kMPU6050AngleYZOffset = 2.34f;
const float kMPU6050GyroXOffset = -63.46f;

I2C_HandleTypeDef I2C1_Handle;

void MPU6050_IICConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration    
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_I2C1_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    I2C1_Handle.Instance = I2C1;
    I2C1_Handle.Init.ClockSpeed = 400000;
    I2C1_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    I2C1_Handle.Init.OwnAddress1 = MPU6050_ADDRESS;
    I2C1_Handle.Init.OwnAddress2 = 0;
    I2C1_Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2C1_Handle.Init.DutyCycle = I2C_DUTYCYCLE_2;
    I2C1_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2C1_Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&I2C1_Handle) != HAL_OK)
    {
        Error_Handler();
    }
}

void MPU6050_Init(void)
{
    MPU6050_IICConfig();

    MPU6050_Check() ? (void)0 : Error_Handler();

    uint8_t regValue;

    regValue = 0x00;
    HAL_I2C_Mem_Write(&I2C1_Handle, MPU6050_ADDRESS, MPU_RA_PWR_MGMT_1,
                      I2C_MEMADD_SIZE_8BIT, &regValue, 1, 5); // Wake Up
    regValue = 0x01;
    HAL_I2C_Mem_Write(&I2C1_Handle, MPU6050_ADDRESS, MPU_RA_SMPLRT_DIV,
                      I2C_MEMADD_SIZE_8BIT, &regValue, 1, 5); // Sampling rate 500Hz(1000 / (1 + 1))
    regValue = 0x02;
    HAL_I2C_Mem_Write(&I2C1_Handle, MPU6050_ADDRESS, MPU_RA_CONFIG,
                      I2C_MEMADD_SIZE_8BIT, &regValue, 1, 5); // FSYNC input disabled, DLPF set to 94Hz, delay 3.0ms
    regValue = 0x00;
    HAL_I2C_Mem_Write(&I2C1_Handle, MPU6050_ADDRESS, MPU_RA_ACCEL_CONFIG,
                      I2C_MEMADD_SIZE_8BIT, &regValue, 1, 5); // No self test, accelerometer range from -2g to +2g.
    regValue = 0x00;
    HAL_I2C_Mem_Write(&I2C1_Handle, MPU6050_ADDRESS, MPU_RA_GYRO_CONFIG,
                      I2C_MEMADD_SIZE_8BIT, &regValue, 1, 5); // No selt test, gyroscope range from -250 to +250 digree/second
}

bool MPU6050_Check(void)
{
    uint8_t buf[1];

    HAL_I2C_Mem_Read(&I2C1_Handle, MPU6050_ADDRESS, MPU_RA_WHO_AM_I,
                     I2C_MEMADD_SIZE_8BIT, buf, 1, 5);

    return buf[0] == 0x68;
}

int16_t MPU6050_GetData(uint8_t addr)
{
    uint8_t buf[2];

    HAL_I2C_Mem_Read(&I2C1_Handle, MPU6050_ADDRESS, addr,
                     I2C_MEMADD_SIZE_8BIT, buf, 2, 10);
    return (int16_t)(buf[0] << 8 | buf[1]);
}

kalman1_state kalman1AngleYZ;
kalman1_state kalman1GyroX;

kalman2_state kalman2AngleYZ;
kalman2_state kalman2GyroX;

#define _MPU6050_DEBUG_FILTER_

float MPU6050_GetAngleYZ(void)
{
    //     float _angleYZ = atan2(MPU6050_GetData(MPU_RA_ACCEL_YOUT_H), MPU6050_GetData(MPU_RA_ACCEL_ZOUT_H)) * RAD_TO_DEG - kMPU6050AngleYZOffset;
    //     float k1angleYZ = kalman1_filter(&kalman1AngleYZ, _angleYZ);
    //     float k2angleYZ = kalman2_filter(&kalman2AngleYZ, _angleYZ);

    // #if defined(_MPU6050_DEBUG_FILTER_)

    //     printf("%f\t%f\t%f\t", _angleYZ, k1angleYZ, k2angleYZ);

    // #endif // _MPU6050_DEBUG_FILTER_

    //     return k1angleYZ;
    //     // return k2angleYZ;

    return atan2(MPU6050_GetData(MPU_RA_ACCEL_YOUT_H), MPU6050_GetData(MPU_RA_ACCEL_ZOUT_H)) * RAD_TO_DEG - kMPU6050AngleYZOffset;
}

float MPU6050_GetGyroX(void)
{
//     float _gyroX = (float)(MPU6050_GetData(MPU_RA_GYRO_XOUT_H) - kMPU6050GyroXOffset) * MPU6050A_2mg;
//     float k1gyroX = kalman1_filter(&kalman1GyroX, _gyroX);
//     float k2gyroX = kalman2_filter(&kalman2GyroX, _gyroX);

// #if defined(_MPU6050_DEBUG_FILTER_)

//     printf("%f\t%f\t%f\t", _gyroX, k1gyroX, k2gyroX);

// #endif // _MPU6050_DEBUG_FILTER_

//     // return k1gyroX;
//     return k2gyroX;
    return (float)(MPU6050_GetData(MPU_RA_GYRO_XOUT_H) - kMPU6050GyroXOffset) * MPU6050G_s250dps;
}
