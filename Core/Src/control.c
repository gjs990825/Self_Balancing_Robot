#include "control.h"
#include "mpu6050.h"
#include "utils.h"
#include "motor.h"
#include "debug.h"
#include <stdlib.h>

// the target speed
int16_t _target_turnning_speed = 0;
int16_t _target_move_speed = 0;
int16_t _target_position = 0;

// angle loop for balancing
int16_t Control_Balance(float angle, float gyro)
{
    const float kp = 115, kd = 0.21;

    return kp * angle + kd * gyro;
}

float v_error = 0.0;
int16_t smooth_speed_last_error = 0;

// velocity loop for speed control
int16_t Control_Velocity(float current_speed)
{
    const float kp = 1.4;
    const float kp_smooth_speed = 1.0;

    int16_t smooth_error = _target_move_speed - current_speed;

    smooth_speed_last_error *= 0.7;
    smooth_speed_last_error += 0.3 * smooth_error;

    int16_t _smoothed_speed = current_speed + smooth_speed_last_error * kp_smooth_speed;

    v_error *= 0.8;
    v_error += (_smoothed_speed - current_speed) * 0.2;

    // log_info("S:%5.0f\tERR:%5.0f\t\r\n", current_speed, v_error);

    return v_error * kp;
}

static int current_position = 0;

// position loop for stay close to the original position
int16_t Control_Position(int16_t current_encoder)
{
    const float kp = 0.3;

    current_position += current_encoder;
    current_position = constrain_int(current_position, -1000, 1000);

    float error = _target_position - current_position;

    return error * kp;
}

void Control_ResetPIDData(void)
{
    v_error = 0;
    smooth_speed_last_error = 0;
    current_position = 0;
}

void Control_UpdateTurnningSpeed(int16_t speed) { _target_turnning_speed = speed; }
void Control_UpdateMoveSpeed(int16_t speed) { _target_move_speed = speed; }

void Control_EnterIdleState(void)
{
    Control_UpdateMoveSpeed(0);
    Control_UpdateTurnningSpeed(0);
}

void Control_MPUInterruptCallBack(void)
{
    static bool sta = false;
    static bool fall_down_flag = false;
    static uint32_t manual_standup_time = 0;

    MPU6050_ReadDMP();

    sta = !sta;
    if (sta)
        return;

    float angle = MPU6050_GetAngle();

    if (abs((int)angle) > _CONTROL_MAXIMUM_ANGLE)
    {
        if (!fall_down_flag)
        {
            fall_down_flag = true;
            Motor_ShutDown();
            Control_ResetPIDData();
            log_info("Loosing balance at %d, shutting down...\r\n", HAL_GetTick());
        }
        return;
    }
    else if (fall_down_flag)
    {
        fall_down_flag = false;
        manual_standup_time = HAL_GetTick();
        log_info("Manual Standup at %d, restart in %dms...\r\n", manual_standup_time, _CONTORL_RESTANDUP_DELAY);
    }

    if (HAL_GetTick() - _CONTORL_RESTANDUP_DELAY < manual_standup_time)
        return;

    int16_t current_encoder = Motor_GetEncoder();
    float current_speed = current_encoder * _MOTOR_SPEED_ENCODER_RATICIAL;

    int16_t balance_out = Control_Balance(angle, MPU6050_GetGyro());
    int16_t velocity_out = Control_Velocity(current_speed);
    int16_t position_out = Control_Position(current_encoder);


#if defined(_DEBUG_OUOTPUT_CONTROL_DATA)

    log_info("Angle:%5.1f\tGyro:%d\tBalance:%5d\tVelocity:%5d\r\n", angle, gyro, balance_out, velocity_out);

#else

    int16_t combined_out = -balance_out - velocity_out - position_out;
    Motor_Control(combined_out, _target_turnning_speed);

#endif // _DEBUG_OUOTPUT_CONTROL_DATA_
}
