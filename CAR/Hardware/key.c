#include "key.h"
#include "ti_msp_dl_config.h"
#include "bsp_systick.h"

/* ============================================================
 * 按键扫描 (纯驱动, 不含任务逻辑)
 *   引脚用 syscfg 宏 GPIO_KEY_K1/K2/K3_PORT/PIN (KEY 实例, 引脚名 K1/K2/K3)
 *   按键上拉, 按下读到低电平
 *   K1=PB27, K2=PA25, K3=PA26 (用户确认实物)
 * ============================================================ */

uint8_t Key_Scan(void)
{
    /* 非阻塞: 返回当前按下的键号, 0=无 */
    if (DL_GPIO_readPins(KEY_K1_PORT, KEY_K1_PIN) == 0) return 1;
    if (DL_GPIO_readPins(KEY_K2_PORT, KEY_K2_PIN) == 0) return 2;
    if (DL_GPIO_readPins(KEY_K3_PORT, KEY_K3_PIN) == 0) return 3;
    return 0;
}

uint8_t Key_GetNum(void)
{
    /* 阻塞去抖: 检测到按下 → 20ms 去抖 → 等松开 → 20ms 去抖 → 返回键号
     * 注意: 本函数阻塞, 不要在 10ms PID 中断内调用, 仅在主循环用 */
    uint8_t KeyNum = 0;

    if (DL_GPIO_readPins(KEY_K1_PORT, KEY_K1_PIN) == 0) {
        delay_ms(20);
        if (DL_GPIO_readPins(KEY_K1_PORT, KEY_K1_PIN) == 0) {  /* 二次确认 */
            while (DL_GPIO_readPins(KEY_K1_PORT, KEY_K1_PIN) == 0);  /* 等松开 */
            delay_ms(20);
            KeyNum = 1;
        }
    }
    else if (DL_GPIO_readPins(KEY_K2_PORT, KEY_K2_PIN) == 0) {
        delay_ms(20);
        if (DL_GPIO_readPins(KEY_K2_PORT, KEY_K2_PIN) == 0) {
            while (DL_GPIO_readPins(KEY_K2_PORT, KEY_K2_PIN) == 0);
            delay_ms(20);
            KeyNum = 2;
        }
    }
    else if (DL_GPIO_readPins(KEY_K3_PORT, KEY_K3_PIN) == 0) {
        delay_ms(20);
        if (DL_GPIO_readPins(KEY_K3_PORT, KEY_K3_PIN) == 0) {
            while (DL_GPIO_readPins(KEY_K3_PORT, KEY_K3_PIN) == 0);
            delay_ms(20);
            KeyNum = 3;
        }
    }

    return KeyNum;
}
