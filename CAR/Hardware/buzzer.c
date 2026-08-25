/**
 * @file  buzzer.c
 * @brief 蜂鸣器驱动实现
 *
 * 来源: mspm0g3507_car/Hardware/Buzzer.c
 * 移植: 2026-07-23 → CAR/Hardware/
 * 适配:
 *   - 去掉 #include "main.h", 用本工程 buzzer.h
 *   - 源项目宏 Buzzer_PORT/Buzzer_PIN_PIN 改为 buzzer.h 的 BUZZER_PORT/BUZZER_PIN
 *   - delay_ms() 由 CAR board.c 提供 (含在 ti_msp_dl_config.h 链路里, 需 include board.h)
 *
 * 电平约定 (用户确认): 低电平响
 *   - Buzzer_on  = clearPins (拉低 → 响)
 *   - Buzzer_off = setPins   (拉高 → 停)
 */
#include "buzzer.h"
#include "board.h"   /* delay_ms() 来自 CAR board.c */

/* 开蜂鸣器: 拉低引脚 (低电平响) */
void Buzzer_on(void)
{
    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
}

/* 关蜂鸣器: 拉高引脚 */
void Buzzer_off(void)
{
    DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN);
}

/* 响 100ms 后自动关 */
void Buzzer_turn(void)
{
    Buzzer_on();
    delay_ms(100);
    Buzzer_off();
}
