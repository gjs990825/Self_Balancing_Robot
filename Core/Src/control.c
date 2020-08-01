#include "control.h"
#include "mpu6050.h"
#include "utils.h"
#include "motor.h"
#include "debug.h"
#include <stdlib.h>

// the target speed
int _target_turnning_speed = 0;
int _target_move_speed = 0;
// the output speed (instant updated smoothed speed)
int _smoothed_move_speed = 0;

// angle loop for balancing
int Control_Balance(float angle, float gyro)
{
    const float kp = 110, kd = 0.2;

    return kp * angle + kd * gyro;
}

const int k_v_intergral_limit = 1000;
float v_intergral, v_bias;

// velocity loop for staying still
int Control_Velocity(int current_speed)
{
    const float kp = 15, ki = 0.3;
    const float k_speed_ratio = 0.1;

    v_bias *= 0.8;
    v_bias += (current_speed - (_smoothed_move_speed * k_speed_ratio)) * 0.2;

    v_intergral += v_bias;
    v_intergral = constrain_float(v_intergral, -k_v_intergral_limit, k_v_intergral_limit);

    return v_bias * kp + v_intergral * ki;
}

void Control_ResetPIDData(void)
{
    v_intergral = 0;
    v_bias = 0;
}

void Control_UpdateTurnningSpeed(int16_t speed) { _target_turnning_speed = speed; }
void Control_UpdateMoveSpeed(int16_t speed) { _target_move_speed = speed; }

void Control_EnterIdleState(void)
{
    Control_UpdateMoveSpeed(0);
    Control_UpdateTurnningSpeed(0);
}

// Speed smooth control for better stability

void Control_SmoothMoveSpeed(void)
{
    const float kp = 0.3, kd = 0.0;
    static int bias = 0;

    bias = _target_move_speed - _smoothed_move_speed;

    int derivative = 0; // do it later

    _smoothed_move_speed += kp * bias + kd * derivative;

    // log_info("T:%d O:%d B:%d\r\n", _target_move_speed, _smoothed_move_speed, bias);
}

void Control_MPUInterruptCallBack(void)
{
    static bool sta = false;
    static bool fail_flag = false;
    static uint32_t manual_standup_time = 0;

    MPU6050_ReadDMP();

    sta = !sta;
    if (sta)
        return;

    float angle = MPU6050_GetAngle();

    if (abs((int)angle) > _MAX_ANGLE_)
    {
        if (!fail_flag)
        {
            fail_flag = true;
            Motor_ShutDown();
            Control_ResetPIDData();
        }
        return;
    }
    else if (fail_flag)
    {
        fail_flag = false;
        manual_standup_time = HAL_GetTick();
    }

    if (HAL_GetTick() - _RE_STANDUP_DELAY_ < manual_standup_time)
        return;


    Control_SmoothMoveSpeed();

    int balance_out = Control_Balance(angle, MPU6050_GetGyro());
    int speed_out = Control_Velocity(Motor_GetSpeed());

    int combined_out = -balance_out + speed_out;

    Motor_Control(combined_out, _target_turnning_speed);

    // log_info("%5.1f\t%5.1f\t%5d\t%5d\r\n", angle, gyro, balance_out, velocity_out);
    // log_info("Angle:%5.1f\tGyro:%5.1f\tBalance:%5d\tVelocity:%5d\r\n", angle, gyro, balance_out, velocity_out);
}
