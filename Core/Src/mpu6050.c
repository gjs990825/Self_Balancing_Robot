#include "mpu6050.h"
#include "debug.h"
#include "kalman_filter.h"
#include <stdio.h>
#include <math.h>
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "led.h"

const float kMPU6050AngleYZOffset = 2.34f;
const float kMPU6050GyroXOffset = -63.46f;

float Pitch, Roll;

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
    return atan2(MPU6050_GetData(MPU_RA_ACCEL_YOUT_H), MPU6050_GetData(MPU_RA_ACCEL_ZOUT_H)) * RAD_TO_DEG - kMPU6050AngleYZOffset;
}

float MPU6050_GetGyroX(void)
{
    return (float)(MPU6050_GetData(MPU_RA_GYRO_XOUT_H) - kMPU6050GyroXOffset) * MPU6050G_s250dps;
}

// For dmp data exchange↓

int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data)
{
    if (HAL_I2C_Mem_Write(&I2C1_Handle, slave_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, length, 10) == HAL_OK)
        return 0;
    else
        return -1;
}

int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
    if (HAL_I2C_Mem_Read(&I2C1_Handle, slave_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, length, 10) == HAL_OK)
        return 0;
    else
        return -1;
}

const int8_t gyro_orientation[9] = {1, 0, 0,
                                    0, 1, 0,
                                    0, 0, 1};

uint8_t inv_row_2_scale(const int8_t *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7; // error
    return b;
}

uint16_t inv_orientation_matrix_to_scalar(const int8_t *mtx)
{
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}

void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7)
    {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
        printf("setting bias succesfully ......\r\n");
    }
}

void MPU6050_DMPInit(void)
{
    if (!mpu_init(NULL))
    {
        if (!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
            printf("mpu_set_sensor complete ......\r\n");
        if (!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
            printf("mpu_configure_fifo complete ......\r\n");
        if (!mpu_set_sample_rate(200)) // 200Hz
            printf("mpu_set_sample_rate complete ......\r\n");
        if (!dmp_load_motion_driver_firmware())
            printf("dmp_load_motion_driver_firmware complete ......\r\n");
        if (!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
            printf("dmp_set_orientation complete ......\r\n");
        if (!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                                DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                                DMP_FEATURE_GYRO_CAL))
            printf("dmp_enable_feature complete ......\r\n");
        if (!dmp_set_fifo_rate(200)) // 200Hz
            printf("dmp_set_fifo_rate complete ......\r\n");
        run_self_test();
        if (!mpu_set_dmp_state(1))
            printf("mpu_set_dmp_state complete ......\r\n");
    }
    else
    {
        printf("mpu init failed\r\n");
    }
}

void MPU6050_EXTICallBack(void)
{
    MPU6050_ReadDMP();
    printf("Pitch:%5f\t Roll:%5f\t\t\n", Pitch, Roll);
}

EXTI_HandleTypeDef EXTI_A1HandleStruct;

void MPU6050_EXTIInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_ConfigTypeDef EXTI_A1ConfigStruct;

    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    EXTI_A1ConfigStruct.Line = EXTI_LINE_1;
    EXTI_A1ConfigStruct.Mode = EXTI_MODE_INTERRUPT;
    EXTI_A1ConfigStruct.Trigger = EXTI_TRIGGER_FALLING;
    EXTI_A1ConfigStruct.GPIOSel = EXTI_GPIOA;

    HAL_EXTI_RegisterCallback(&EXTI_A1HandleStruct, HAL_EXTI_COMMON_CB_ID, MPU6050_EXTICallBack);
    HAL_EXTI_SetConfigLine(&EXTI_A1HandleStruct, &EXTI_A1ConfigStruct);

    HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 2);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

void EXTI1_IRQHandler(void)
{
    HAL_EXTI_IRQHandler(&EXTI_A1HandleStruct);
}

void MPU6050_ReadDMP(void)
{
#define q30 1073741824.0f

    float q0, q1, q2, q3;
    short gyro[3], accel[3], sensors;
    unsigned long sensor_timestamp;
    unsigned char more;
    long quat[4];

    dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
    if (sensors & INV_WXYZ_QUAT)
    {
        q0 = quat[0] / q30;
        q1 = quat[1] / q30;
        q2 = quat[2] / q30;
        q3 = quat[3] / q30;
        Pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * RAD_TO_DEG;
        Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * RAD_TO_DEG;
    }
}
