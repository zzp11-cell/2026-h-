#ifndef __BSP_SYSTICK_H
#define __BSP_SYSTICK_H

#include "ti_msp_dl_config.h"

//Systick最大计数值,24位
#define SysTickMAX_COUNT 0xFFFFFF

//Systick计数频率
#define SysTickFre 80000000

//将systick的计数值转换为具体的时间单位
#define SysTick_MS(x)  ((SysTickFre/1000U)*(uint32_t)(x))
#define SysTick_US(x)  ((SysTickFre/1000000U)*(uint32_t)(x))

uint32_t Systick_getTick(void);
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);

#endif
