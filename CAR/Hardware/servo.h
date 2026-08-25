/**
 * @file  servo.h
 * @brief 舵机驱动 (50Hz PWM, 角度控制)
 *
 * 来源: mspm0g3507_car/Hardware/Servo.h
 * 移植: 2026-07-23 → CAR/Hardware/
 * 适配:
 *   - 去掉 #include "main.h"
 *   - 源项目用 PWM_1_INST/PWM_2_INST (TIMA0), 但 CAR 的 TIMA0 已被电机 PWM 占用
 *     → 改用 TIMG7 (用户选定, 50Hz PWM, 未被占用)
 *   - API 修正: DL_TimerG_setCaptureCompareValue 不存在 (skill 黑名单)
 *     → 改用 DL_Timer_setCaptureCompareValue (通用版, CAR motor.c 已验证可用)
 *
 * 引脚/定时器状态: 用户暂不接舵机, 宏占位待确认。
 *
 * ⚠️ 接硬件前必须在 SysConfig (empty.syscfg) 中做以下配置:
 *    1. 添加一个 TIMER 实例, 选 TIMG7 (或你选定的定时器)
 *    2. 模式 = PWM, 频率 = 50Hz (周期 20ms, 计数周期 period=1000 对应 0.5~2.5ms 高电平)
 *    3. 添加 2 个 PWM 通道 (Servo1 用 CCP0, Servo2 用 CCP1), 分配引脚
 *    4. 生成代码后, SysConfig 会产出 PWM_1_INST / GPIO_PWM_1_C0_IDX 等宏
 *
 * 角度→占空比换算 (period=1000):
 *   0.5ms 高电平 → 计数 25  (对应 0°)
 *   2.5ms 高电平 → 计数 125 (对应 180°)
 *   线性插值: count = 25 + (angle/180)*100
 */
#ifndef __SERVO_H__
#define __SERVO_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ============ 定时器/通道宏 (待用户在 SysConfig 配置后替换) ============
 * 以下宏为占位, 实际值由 SysConfig 生成。
 * 在 SysConfig 配好 TIMG7 50Hz PWM + 2 通道后, 用生成的宏名替换:
 *   PWM_1_INST          → SysConfig 生成的舵机定时器实例 (如 TIMG7)
 *   GPIO_PWM_1_C0_IDX   → Servo1 的 CCP 通道索引 (如 DL_TIMER_CC_0_INDEX)
 *   GPIO_PWM_2_C0_IDX   → Servo2 的 CCP 通道索引
 */
#define SERVO_TIMER_INST         (TIMG7)                 /* 待确认: SysConfig 配的实例 */
#define SERVO1_CC_IDX           (DL_TIMER_CC_0_INDEX)   /* 待确认: Servo1 通道索引 */
#define SERVO2_CC_IDX           (DL_TIMER_CC_1_INDEX)   /* 待确认: Servo2 通道索引 */

/* ============ API ============ */
void Servo1_PWM_Set_Angle(unsigned int angle);   /* 设置 Servo1 角度 (0~180°) */
void Servo2_PWM_Set_Angle(unsigned int angle);   /* 设置 Servo2 角度 (0~180°) */

#endif /* __SERVO_H__ */
