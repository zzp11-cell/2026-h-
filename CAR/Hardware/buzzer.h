/**
 * @file  buzzer.h
 * @brief 蜂鸣器驱动 (GPIO 电平控制)
 *
 * 来源: mspm0g3507_car/Hardware/Buzzer.h
 * 移植: 2026-07-23 → CAR/Hardware/
 * 适配: 引脚宏从源项目 syscfg 宏改为本工程独立定义, 用户确认接线
 *
 * 引脚确认: PB17 (低电平响)
 *   - BUZZER_PORT = GPIOB
 *   - BUZZER_PIN  = DL_GPIO_PIN_17
 *
 * 注意: 需在 SysConfig (empty.syscfg) 中把 PB17 配为 GPIO 输出, 或在 ti_msp_dl_config
 *       中手动初始化。本驱动只做电平操作, 不负责引脚初始化。
 */
#ifndef __BUZZER_H__
#define __BUZZER_H__

#include "ti_msp_dl_config.h"

/* ============ 引脚定义 (用户确认: PB17, 低电平响) ============ */
/* 如需改引脚, 只改这两行即可 */
#define BUZZER_PORT        (GPIOB)
#define BUZZER_PIN         (DL_GPIO_PIN_17)

/* ============ API ============ */
void Buzzer_on(void);     /* 开蜂鸣器 (低电平) */
void Buzzer_off(void);    /* 关蜂鸣器 (高电平) */
void Buzzer_turn(void);   /* 响 100ms 后自动关 */

#endif /* __BUZZER_H__ */
