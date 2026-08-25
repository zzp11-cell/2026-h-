/**
 * @file  led.c
 * @brief LED 指示灯驱动实现
 *
 * 来源: mspm0g3507_car/Hardware/LED.c + WHEELTEC 例程 LED_Flash 风格
 * 移植: 2026-07-23 → CAR/Hardware/
 * 适配:
 *   - 去掉 #include "main.h", 用本工程 led.h + board.h
 *   - 源项目宏 GPIO_LED_PORT/GPIO_LED_Light1_PIN 改为 led.h 的 LED_PORT/LED1_PIN
 *   - LED1 (PA7) 保留原有电平控制
 *   - LED2 (PB22) 新增: 状态慢闪, 照搬 WHEELTEC LED_Flash(time) 计数翻转思路
 *   - delay_ms() 由 CAR board.c 提供
 *
 * 电平约定 (用户确认): 高电平点亮
 */
#include "led.h"
#include "board.h"   /* delay_ms() */

/* ======================== LED1: PA7 (原有) ======================== */

/* 点亮 LED1: 拉高引脚 (高电平点亮) */
void LED1_ON(void)
{
    DL_GPIO_setPins(LED_PORT, LED1_PIN);
}

/* 熄灭 LED1: 拉低引脚 */
void LED1_OFF(void)
{
    DL_GPIO_clearPins(LED_PORT, LED1_PIN);
}

/* 亮 100ms 后自动灭 */
void LED1_Turn(void)
{
    LED1_ON();
    delay_ms(100);
    LED1_OFF();
}

/* ======================== LED2: PB22 用户灯 (状态慢闪) ======================== */
/* 照搬 WHEELTEC 例程 LED_Flash(time): 静态计数, 计到 time 翻转一次
 * 节拍由调用方提供 (SysTick 1ms), 不占用 TIMER_0 (10ms PID 节拍) */

/* 点亮 LED2: 拉高引脚 (高电平点亮) */
void LED2_ON(void)
{
    DL_GPIO_setPins(LED2_PORT, LED2_PIN);
}

/* 熄灭 LED2: 拉低引脚 */
void LED2_OFF(void)
{
    DL_GPIO_clearPins(LED2_PORT, LED2_PIN);
}

/* 翻转 LED2 */
void LED2_Toggle(void)
{
    DL_GPIO_togglePins(LED2_PORT, LED2_PIN);
}

/**
 * @brief  LED2 状态慢闪 (照搬 WHEELTEC LED_Flash 思路)
 * @param  ms: 翻转周期 (毫秒). 每 ms 毫秒翻转一次 → 亮 ms、灭 ms 循环
 *             典型 500 → 500ms 亮 500ms 灭, 慢闪看待机状态
 * @note   必须在固定 1ms 节拍里调用 (SysTick_Handler), 传 ms=0 则常亮
 *         不占用 TIMER_0 (10ms PID 节拍不受影响)
 */
void LED2_StatusFlash(uint16_t ms)
{
    static uint16_t temp = 0;
    if (ms == 0) {
        LED2_ON();        /* ms=0: 常亮 */
        temp = 0;
    } else if (++temp >= ms) {
        LED2_Toggle();    /* 计满 ms 次翻转一次 */
        temp = 0;
    }
}
