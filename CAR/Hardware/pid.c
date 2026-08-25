#include "ti_msp_dl_config.h"
#include "pid.h"
#include "gyro_serial.h"   /* Yaw() */
#include "motor.h"          /* Car_Move(左,右) */
#include "encoder.h"        /* Get_Encoder_countA/B */
#include <math.h>           /* fabsf */

/* ============================================================
 * 角度环 PID (移植自 2024QuestionH pid.c Angle_Calculate)
 *   实测参数 KP=80 KI=0.5 KD=3, 限幅 4500, 10ms 节拍跑直稳定.
 *   normalize_angle 把误差折回 [-180,180], 避免跨越 ±180 时跳变.
 *
 *   速度环 -> encoder.c Velocity_A/B (Velcity_Kp/Ki/Kff)
 *   循迹转向环 -> track.c Track_Steering_Compute (TRACK_KP/KD)
 *   本文件只保留角度环, 其余环在各自模块.
 * ============================================================ */

/* 角度环 PID 参数 (VOFA+ 可在线调参, 命令 P1/I1/D1) */
float ANGLE_KP = 80.0f;
float ANGLE_KI = 0.5f;
float ANGLE_KD = 3.0f;

static float angle_integral    = 0.0f;
static float angle_prev_error  = 0.0f;

static float normalize_angle(float angle)
{
    while (angle >  180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

void Angle_PID_Reset(void)
{
    angle_integral   = 0.0f;
    angle_prev_error = 0.0f;
}

float Angle_Calculate(float target, float current, float dt)
{
    float error = normalize_angle(target - current);

    float P = ANGLE_KP * error;
    angle_integral += error * dt;
    if (angle_integral >  100.0f) angle_integral =  100.0f;
    if (angle_integral < -100.0f) angle_integral = -100.0f;
    float I = ANGLE_KI * angle_integral;
    float derivative = (error - angle_prev_error) / dt;
    float D = ANGLE_KD * derivative;
    angle_prev_error = error;

    float output = P + I + D;
    if (output >  MAX_STEERING) output =  MAX_STEERING;
    if (output < -MAX_STEERING) output = -MAX_STEERING;
    return output;
}
