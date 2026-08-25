#include "ti_msp_dl_config.h"
#include "board.h"

volatile unsigned long tick_ms;
volatile uint32_t start_time;

void SysTick_Init(void)
{
    DL_SYSTICK_config(CPUCLK_FREQ/1000);
    NVIC_SetPriority(SysTick_IRQn, 0);
}


#if !defined(__MICROLIB)
//��ʹ��΢��Ļ�����Ҫ��������ĺ���
#if (__ARMCLIB_VERSION <= 6000000)
//�����������AC5  �Ͷ�����������ṹ��
struct __FILE
{
	int handle;
};
#endif

FILE __stdout;

//����_sys_exit()�Ա���ʹ�ð�����ģʽ
void _sys_exit(int x)
{
	x = x;
}
#endif

/* 蜂鸣器引脚: 用户已移除蜂鸣器, PA9 现用作 PWMB (电机B通道PWM)。
 * Buzzer_On/Off 保留空实现, 不驱动任何引脚, 防止旧代码调用报错。 */
#ifndef Buzzer_PORT
#define Buzzer_PORT  GPIOA
#define Buzzer_PIN_1_PIN   DL_GPIO_PIN_9   /* 仅占位, 实际不使用; PA9=PWMB */
#endif

void Buzzer_On(void)
{
    /* 蜂鸣器已移除, PA9 用于循迹传感器 CH5, 不再驱动该引脚 */
}

void Buzzer_Off(void)
{
    /* 蜂鸣器已移除, 空实现 */
}
