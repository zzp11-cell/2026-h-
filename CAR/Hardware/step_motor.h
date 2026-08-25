/**
 * @file  step_motor.h
 * @brief 步进电机驱动 (双 4 相 8 拍, 支持两个步进电机)
 *
 * 来源: mspm0g3507_car/Hardware/Step_Motor.h
 * 移植: 2026-07-23 → CAR/Hardware/
 * 适配:
 *   - 去掉 #include "main.h"
 *   - 引脚宏独立定义 (源项目用 syscfg 宏 Step_Motor_IN1_PORT 等)
 *   - 延时 mspm0_delay_ms() → delay_ms() (CAR board.c)
 *
 * 引脚状态: 用户暂不接步进电机, 引脚宏全部用 【待确认】 占位。
 *           接硬件时只需修改下方 8 个引脚宏即可 (4+4)。
 *
 * 工作模式:
 *   mode=0  4相8拍   (步进角小, 运行平滑, 每圈4096步)
 *   mode=1  4相单4拍 (步进角大, 扭矩小)
 *   mode=2  4相双4拍 (步进角同单4拍, 扭矩大)
 *
 * 电平约定 (源项目): 低电平导通 (clearPins = 该相通电)
 *                    —— ULN2003/三极管驱动板常用此约定, 接硬件后若转向反, 检查驱动板电平
 */
#ifndef __STEP_MOTOR_H__
#define __STEP_MOTOR_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ============ 步进电机实例结构 ============ */
struct STEP_MOTOR
{
    uint8_t current_step;   /* 当前步 (节拍索引) */
    int     remain_steps;   /* 剩余步 (角度换算后的余数) */
    uint8_t number;         /* 电机编号 */
};

extern struct STEP_MOTOR Step_Motor_one;   /* 第一个步进电机 */
extern struct STEP_MOTOR Step_Motor_two;   /* 第二个步进电机 */

/* 三角函数侧边运动用的状态量 */
extern int Step_Motor_one_Original_Angle;
extern int Step_Motor_two_Original_Angle;
extern int tem_one_angle;
extern int tem_two_angle;
extern int trigonometry_a;
extern int step_motor_delay_time;

/* ==================== 第一组电机引脚 (待确认) ==================== */
/* 接硬件后改为实际值, 例如:
 *   #define Step_Motor_IN1_PORT   (GPIOA)
 *   #define Step_Motor_IN1_PIN    (DL_GPIO_PIN_15)
 * 4 个引脚建议选同一端口 (如 GPIOA 或 GPIOB), 避开已占用:
 *   PA2~PA6(时钟禁用), PA7(LED), PA8/PA9(电机PWM), PA10/PA11(UART0),
 *   PA12/PA13(电机AIN), PB2/PB3(电机BIN), PA22/PA24/PA17(编码器),
 *   PA27/PA25/PA26(按键), PA28/PA31(OLED), PB4/PB5(IMU), PB17(蜂鸣器)
 */
#define Step_Motor_IN1_PORT   (GPIOA)   /* PAxx — 待确认 */
#define Step_Motor_IN1_PIN    (DL_GPIO_PIN_0)   /* 待确认 */
#define Step_Motor_IN2_PORT   (GPIOA)   /* PAxx — 待确认 */
#define Step_Motor_IN2_PIN    (DL_GPIO_PIN_0)   /* 待确认 */
#define Step_Motor_IN3_PORT   (GPIOA)   /* PAxx — 待确认 */
#define Step_Motor_IN3_PIN    (DL_GPIO_PIN_0)   /* 待确认 */
#define Step_Motor_IN4_PORT   (GPIOA)   /* PAxx — 待确认 */
#define Step_Motor_IN4_PIN    (DL_GPIO_PIN_0)   /* 待确认 */

/* ==================== 第二组电机引脚 (待确认) ==================== */
#define Step_Motor_BIN1_PORT  (GPIOB)   /* PBxx — 待确认 */
#define Step_Motor_BIN1_PIN   (DL_GPIO_PIN_0)   /* 待确认 */
#define Step_Motor_BIN2_PORT  (GPIOB)   /* PBxx — 待确认 */
#define Step_Motor_BIN2_PIN   (DL_GPIO_PIN_0)   /* 待确认 */
#define Step_Motor_BIN3_PORT  (GPIOB)   /* PBxx — 待确认 */
#define Step_Motor_BIN3_PIN   (DL_GPIO_PIN_0)   /* 待确认 */
#define Step_Motor_BIN4_PORT  (GPIOB)   /* PBxx — 待确认 */
#define Step_Motor_BIN4_PIN   (DL_GPIO_PIN_0)   /* 待确认 */

/* ==================== API ==================== */
/* 通用 (第一组) */
void Step_Motor_Init(void);
void Step_Motor_Move(struct STEP_MOTOR *step_motor, int8_t dir);
void Step_Motor_Rhythm_4_1_4(uint8_t step, uint8_t dly);
void Step_Motor_Rhythm_4_2_4(uint8_t step, uint8_t dly);
void Step_Motor_Rhythm_4_1_8(uint8_t step, uint8_t dly);
void Step_Motor_Direction(struct STEP_MOTOR *step_motor, uint8_t dir, uint8_t mode, uint8_t dly);
void Step_Motor_Rotate_Angle(struct STEP_MOTOR *step_motor, uint8_t dir, uint8_t mode, uint16_t angle, uint8_t dly);
void Step_Motor_Stop(struct STEP_MOTOR *step_motor);

/* 第二组 (BIN 引脚) */
void Step_Motor_two_Rhythm_4_1_4(uint8_t step, uint8_t dly);
void Step_Motor_two_Rhythm_4_2_4(uint8_t step, uint8_t dly);
void Step_Motor_two_Rhythm_4_1_8(uint8_t step, uint8_t dly);
void Step_Motor_two_Direction(struct STEP_MOTOR *step_motor, uint8_t dir, uint8_t mode, uint8_t dly);
void Step_Motor_two_Rotate_Angle(struct STEP_MOTOR *step_motor, uint8_t dir, uint8_t mode, uint16_t angle, uint8_t dly);

/* 双电机三角函数侧边运动 (Bresenham 同步) */
void Step_Motor_Trigonometry_Side(int direction_one, int direction_two);

#endif /* __STEP_MOTOR_H__ */
