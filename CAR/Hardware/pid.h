#ifndef __PID_H
#define __PID_H

#include "ti_msp_dl_config.h"

/* ============================================================
 * 角度环 PID (锁航向模式用, 循迹模式不跑)
 *   输入: target/current = 目标/当前 yaw (deg), dt = 控制周期 (秒)
 *   输出: steering 转向量 (PWM 量纲, 限幅 ±MAX_STEERING)
 *
 *   三套环分布:
 *     角度环   -> pid.c  (本文件, ANGLE_KP/KI/KD, VOFA P1/I1/D1)
 *     速度环   -> encoder.c (Velocity_A/B, Velcity_Kp/Ki/Kff, VOFA P2/I2/D2)
 *     循迹转向环 -> track.c  (Track_Steering_Compute, TRACK_KP/KD, VOFA P3/D3)
 * ============================================================ */
extern float ANGLE_KP;
extern float ANGLE_KI;
extern float ANGLE_KD;
#define MAX_STEERING    4500

float Angle_Calculate(float target, float current, float dt);
void  Angle_PID_Reset(void);

#endif /* __PID_H */
