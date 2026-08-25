#ifndef _ENCODER_H
#define _ENCODER_H

#include "ti_msp_dl_config.h"
#include "board.h"

/* ============================================================
 *  编码器硬件参数 (换电机/轮子只改这里)
 * ============================================================
 *  默认值对应 CAR 现车: MG310 电机 + 13线编码器
 *    ENCODER_LINES      编码器线数 (每转脉冲数, 13)
 *    ENCODER_MULTIPLY   倍频数    (2 = 双边沿计数, 单边沿为 1)
 *    GEAR_RATIO         减速比    (电机轴:轮轴 = 30:1)
 *    WHEEL_DIAMETER_MM  轮径      (mm, 65 = MG310 常见配套轮; 换轮改这里)
 *
 *  推导:
 *    每转脉冲(电机轴) = ENCODER_LINES × ENCODER_MULTIPLY = 13×2 = 26
 *    每转脉冲(轮轴)   = 26 × GEAR_RATIO = 26×30 = 780
 *    轮周长(m)        = π × WHEEL_DIAMETER_MM / 1000
 *    速度 m/s         = (脉冲数 × 周长) / (每转脉冲(轮轴) × 采样周期s)
 * ============================================================ */
#define ENCODER_LINES      13      /* 编码器线数 (换编码器改这里) */
#define ENCODER_MULTIPLY   2       /* 倍频: 2=单相双边沿(对齐例程 RISE), 4=双相双边沿, 1=单边沿 */
#define GEAR_RATIO         30      /* 减速比 (电机轴转数 : 轮轴转数) */
#define WHEEL_DIAMETER_MM  65.0f   /* 轮径 mm (用户确认: 85mm) */

/* 采样周期 (ms): 速度环 ISR 取值+清零周期 (main.c TIMER_0=TIMG6/10ms) */
#define ENCODER_SAMPLE_MS  10

/* 照搬例程: 编码器累计脉冲是对外变量 (非函数), ISR 累加, 速度环取值后清零 */
extern volatile int Get_Encoder_countA;
extern volatile int Get_Encoder_countB;

/*
 * 原子读取并清零左右编码器的当前采样计数。
 * 函数仅短暂屏蔽编码器 GPIO 组中断；期间到达的边沿会保持为 pending，
 * 在恢复中断后继续处理，避免主控制 ISR 与编码器 ISR 竞争共享计数。
 */
void Encoder_GetAndClear(int *left, int *right);

/* 计算电机 RPM (转/分) — 调用者自行取变量值传入 */
float Calculate_Motor_RPM(int encoder_count, int sample_time_ms);

/* 左/右轮线速度 (m/s), 备用 (main 当前未用, 语义 TODO 见 encoder.c) */
float Get_Speed_ms_L(void);
float Get_Speed_ms_R(void);

/* 位置式 PI + 前馈 速度环 (encoder.c), VOFA 命令 P2/I2/D2 在线调参 */
extern float Velcity_Kp;
extern float Velcity_Ki;
extern float Velcity_Kff;
int  Velocity_A(int target, int actual);
int  Velocity_B(int target, int actual);
void Vel_PI_Reset_A(void);
void Vel_PI_Reset_B(void);

#endif /* _ENCODER_H */
