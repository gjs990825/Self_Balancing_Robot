#include "control.h"
#include "mpu6050.h"
#include "utils.h"
#include "motor.h"
#include <stdio.h>
#include <stdlib.h>

int16_t _turnning_speed = 0;
int16_t _base_speed = 0;

// angle loop
int Control_Balance(float angle, float gyro)
{
    const float kp = 110, kd = 0.2;

    return kp * angle + kd * gyro;
}

float v_intergral, v_bias;

// velocity loop
int Control_Velocity(int16_t encoder_left, int16_t encoder_right)
{
    const float kp = 15, ki = 0.3;

    float encoder_both = encoder_left + encoder_right;

    v_bias *= 0.8;
    v_bias += encoder_both * 0.2;

    v_intergral += v_bias;
    v_intergral = constrain_float(v_intergral, -1000, 2000);

    return v_bias * kp + v_intergral * ki;
}

void Control_ClearData(void)
{
    v_intergral = 0;
    v_bias = 0;
}

void Control_UpdateTurnningSpeed(int16_t speed) { _turnning_speed = speed; }
void Control_UpdateBaseSpeed(int16_t speed) { _base_speed = speed; }

void Control_MPUIntCallBack(void)
{
    static bool sta = false;

    MPU6050_ReadDMP();

    sta = !sta;
    if (sta)
        return;

    float angle_balance = MPU6050_GetAngle();

    if (abs((int)angle_balance) > 35)
    {
        Motor_Control(0, 0);
        Control_ClearData();
        return;
    }

    float gyro_balance = MPU6050_GetGyro();

    int16_t balance_out = Control_Balance(angle_balance, gyro_balance);
    int16_t velocity_out = Control_Velocity(Motor_EncoderReadLeft(), Motor_EncoderReadRight());

    int16_t final_out = -balance_out + velocity_out;

    Motor_Control(_base_speed + final_out, _turnning_speed);

    printf("%5.1f\t%5.1f\t%5d\t%5d\r\n", angle_balance, gyro_balance, balance_out, velocity_out);
    // printf("Angle:%5.1f\tGyro:%5.1f\tBalance:%5d\tVelocity:%5d\r\n", angle_balance, gyro_balance, balance_out, velocity_out);
}
