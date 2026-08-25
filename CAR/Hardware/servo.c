/**
 * @file  servo.c
 * @brief 舵机驱动实现 (50Hz PWM, 角度控制)
 *
 * 来源: mspm0g3507_car/Hardware/Servo.c
 * 移植: 2026-07-23 → CAR/Hardware/
 * 适配:
 *   - 去掉 #include "main.h", 用本工程 servo.h
 *   - 定时器实例 TIMA0 (PWM_1_INST/PWM_2_INST) → TIMG7 (SERVO_TIMER_INST)
 *     原因: CAR 的 TIMA0 已被电机 PWM 占用
 *   - API 修正: DL_TimerG_setCaptureCompareValue → DL_Timer_setCaptureCompareValue
 *     (skill 黑名单: DL_TimerG_setCaptureCompareValue 不存在;
 *      通用版 DL_Timer_setCaptureCompareValue 在 CAR motor.c 已验证可用)
 *
 * 角度→计数换算 (period=1000, 50Hz/20ms):
 *   0°  → 0.5ms  → 计数 25   (min_count = 2.5% × 1000)
 *   180°→ 2.5ms  → 计数 125  (max_count = 12.5% × 1000)
 *   count = 25 + (angle/180) × 100
 *
 * 引脚/定时器状态: 用户暂不接舵机, 宏占位待 SysConfig 配置。
 */
#include "servo.h"

unsigned int Servo1_Angle = 0;   /* Servo1 当前角度 (缓存) */
unsigned int Servo2_Angle = 0;   /* Servo2 当前角度 (缓存) */

/* 计算角度对应的 PWM 计数值 (period=1000) */
static inline uint32_t Servo_Angle_To_Count(unsigned int angle)
{
    uint32_t period    = 1000;
    float    min_count = 2.5f  * 0.01f * period;   /* 25  (0.5ms) */
    float    max_count = 12.5f * 0.01f * period;   /* 125 (2.5ms) */
    float    range     = max_count - min_count;     /* 100 */
    return (uint32_t)(min_count + (((float)angle / 180.0f) * range) + 0.5f);
}

/* 设置 Servo1 角度 (0~180°), 超出自动限幅 */
void Servo1_PWM_Set_Angle(unsigned int angle)
{
    if (angle > 180)
    {
        angle = 180;   /* 限幅 */
    }
    Servo1_Angle = angle;

    DL_Timer_setCaptureCompareValue(SERVO_TIMER_INST,
                                    Servo_Angle_To_Count(angle),
                                    SERVO1_CC_IDX);
}

/* 设置 Servo2 角度 (0~180°) */
void Servo2_PWM_Set_Angle(unsigned int angle)
{
    if (angle > 180)
    {
        angle = 180;
    }
    Servo2_Angle = angle;

    DL_Timer_setCaptureCompareValue(SERVO_TIMER_INST,
                                    Servo_Angle_To_Count(angle),
                                    SERVO2_CC_IDX);
}
