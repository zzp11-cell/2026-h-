#ifndef _MOTOR_H
#define _MOTOR_H

#include "ti_msp_dl_config.h"
#include "board.h"

/* 电机驱动: Car_Move(PL, PR) 直接输出左右轮PWM (追踪/转向用) */
void Car_Move(double PL, double PR);

/* 底层PWM输出: Set_PWM(pwmA, pwmB), 实物 pwmA=左轮(AIN/PA8), pwmB=右轮(BIN/PA9) */
void Set_PWM(int pwmA, int pwmB);

/* 短接制动: 两通道 IN1/IN2 全 HIGH, PWM=0 (TB6612 brake) */
void Motor_Brake(void);

/* PWM 限幅 (照搬例程, 速度环用) */
int limit_PWM(int value, int low, int high);

/* 增量式 PI 速度环声明统一在 encoder.h (实现也在 encoder.c) */
#endif
