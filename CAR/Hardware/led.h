/**
 * @file  led.h
 * @brief LED 指示灯驱动 (GPIO 电平控制 + 状态慢闪)
 *
 * 来源: mspm0g3507_car/Hardware/LED.c + WHEELTEC 例程 LED_Flash 风格
 * 移植: 2026-07-23 → CAR/Hardware/
 *
 * 两个 LED:
 *   - LED1 = PA7  (高电平点亮), 原有指示灯
 *   - LED2 = PB22 (用户灯, 待机状态慢闪), syscfg GPIO4 组名 "user"
 *
 * 状态慢闪 (LED2_StatusFlash):
 *   - 照搬 WHEELTEC 例程 LED_Flash(time) 思路: 计数到 time 翻转一次
 *   - 节拍来源: SysTick 1ms (LED2_Tick), 不占用 TIMER_0 (10ms PID 节拍不动)
 *   - 调用方式: 在 SysTick_Handler 里每 1ms 调一次 LED2_StatusFlash(500)
 *     → 500ms 亮、500ms 灭, 慢闪, 用来看程序是否在待机跑
 *
 * 注意: PB22 需在 SysConfig 中配为 GPIO 输出 (GPIO4 组 associatedPins[1] name="user")。
 *       Rebuild 后生成 LED_user_PIN / LED_user_IOMUX 宏, 本驱动用 DL_GPIO_PIN_22 直操作。
 *
 * 电平约定 (用户确认): 高电平点亮
 *   - LEDx_ON  = setPins   (拉高 → 亮)
 *   - LEDx_OFF = clearPins (拉低 → 灭)
 */
#ifndef __LED_H__
#define __LED_H__

#include "ti_msp_dl_config.h"

/* ============ 引脚定义 (用户确认) ============ */
/* LED1: PA7 (高电平点亮) */
#define LED_PORT        (GPIOA)
#define LED1_PIN        (DL_GPIO_PIN_7)

/* LED2: PB22 用户灯 (syscfg GPIO4 组 name="user") */
#define LED2_PORT       (GPIOB)
#define LED2_PIN        (DL_GPIO_PIN_22)

/* ============ API ============ */
void LED1_ON(void);     /* 点亮 LED1 (高电平) */
void LED1_OFF(void);    /* 熄灭 LED1 (低电平) */
void LED1_Turn(void);   /* 亮 100ms 后自动灭 */

void LED2_ON(void);                 /* 点亮 LED2 (PB22) */
void LED2_OFF(void);                /* 熄灭 LED2 */
void LED2_Toggle(void);             /* 翻转 LED2 */
void LED2_StatusFlash(uint16_t ms); /* 状态慢闪: 每 ms 毫秒翻转一次 (SysTick 1ms 调) */

#endif /* __LED_H__ */
