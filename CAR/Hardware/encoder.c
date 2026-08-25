#include "encoder.h"
#include <math.h>   /* M_PI */

/* M_PI fallback: 某些编译配置下 math.h 未定义 M_PI, 这里兜底 */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
 * 编码器测速 + RPM/线速度 计算
 *
 * 速度环照搬 WHEELTEC C07A TB6612 例程 (变量式接口):
 *   Get_Encoder_countA/B 是 int 全局变量 (不是函数), ISR 直接 ++/-- 累加,
 *   速度环 ISR 取值后手动清零 (见 main.c TIMER_0_INST_IRQHandler).
 *
 * 硬件接线 (本项目 syscfg):
 *   左编码器: E1A=PA22(中断A相), E1B=PA12(B相)
 *   右编码器: E2A=PA24(中断A相), E2B=PA17(B相)
 *   四相都在 GPIOA, 共用 GROUP1 中断 (GROUP1_IRQHandler)
 *   syscfg 极性 RISE_FALL (双边沿) + 双相 = 4 倍频, encoder.h MULTIPLY=4
 *
 * 硬件参数 (encoder.h 顶部宏):
 *   ENCODER_LINES=13, ENCODER_MULTIPLY=4, GEAR_RATIO=30, WHEEL_DIAMETER_MM=85
 *   每转脉冲(电机轴)=13×4=52, 每转脉冲(轮轴)=52×30=1560
 *   轮周长=π×0.085≈0.2670m
 */

/* 照搬例程: 对外变量 (非函数), ISR 累加, 速度环取值后清零 */
volatile int Get_Encoder_countA = 0;   /* 左轮 (E1A/E1B) 10ms 累计脉冲 */
volatile int Get_Encoder_countB = 0;   /* 右轮 (E2A/E2B) 10ms 累计脉冲 */

/*
 * 原子读取并清零当前 10ms 采样窗口的编码器计数。
 * 只屏蔽编码器所在的 GPIOA 组中断，避免影响 UART 和控制定时器。
 */
void Encoder_GetAndClear(int *left, int *right)
{
    uint32_t irq_was_enabled = NVIC_GetEnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);

    NVIC_DisableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);

    *left  = Get_Encoder_countA;
    *right = Get_Encoder_countB;
    Get_Encoder_countA = 0;
    Get_Encoder_countB = 0;

    if (irq_was_enabled != 0U) {
        NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    }
}

/*
 * 计算轮轴 RPM (转/分)
 *   轮轴RPM = (脉冲数 × 60000) / (线数 × 倍频 × 减速比 × 采样周期ms)
 *   sample_time_ms 传 0 时用默认 ENCODER_SAMPLE_MS
 *   注意: 调用者需自行取 Get_Encoder_countA/B 值传入 (本函数不清零)
 */
float Calculate_Motor_RPM(int encoder_count, int sample_time_ms)
{
    if (sample_time_ms <= 0) sample_time_ms = ENCODER_SAMPLE_MS;

    int pulses_per_rev_motor = ENCODER_LINES * ENCODER_MULTIPLY;   /* 电机轴每转脉冲 = 52 */

    float wheel_rpm = (float)encoder_count * 60000.0f
                    / ((float)pulses_per_rev_motor * (float)GEAR_RATIO * (float)sample_time_ms);
    return wheel_rpm;
}

/*
 * 计算左轮线速度 (m/s) — 备用接口, main 当前未用
 *   v = (脉冲数 × 周长) / (每转脉冲(轮轴) × 采样周期s)
 *   TODO: 此函数读 Get_Encoder_countA 变量, 但该变量会被速度环 ISR 每 10ms 清零,
 *         语义已不正确 (会读到 0 或残值)。用时需重设计 (如改成参数传入脉冲数)。
 */
float Get_Speed_ms_L(void)
{
    int pulses = Get_Encoder_countA;   /* TODO: 变量会被 ISR 清零, 语义待修 */

    float pulses_per_rev_wheel = (float)(ENCODER_LINES * ENCODER_MULTIPLY) * (float)GEAR_RATIO;
    float circumference = (float)M_PI * WHEEL_DIAMETER_MM / 1000.0f;
    float sample_sec = (float)ENCODER_SAMPLE_MS / 1000.0f;

    float speed_ms = ((float)pulses * circumference) / (pulses_per_rev_wheel * sample_sec);
    return speed_ms;
}

/* 计算右轮线速度 (m/s), 同左轮 (同样 TODO 语义问题) */
float Get_Speed_ms_R(void)
{
    int pulses = Get_Encoder_countB;   /* TODO: 变量会被 ISR 清零, 语义待修 */

    float pulses_per_rev_wheel = (float)(ENCODER_LINES * ENCODER_MULTIPLY) * (float)GEAR_RATIO;
    float circumference = (float)M_PI * WHEEL_DIAMETER_MM / 1000.0f;
    float sample_sec = (float)ENCODER_SAMPLE_MS / 1000.0f;

    float speed_ms = ((float)pulses * circumference) / (pulses_per_rev_wheel * sample_sec);
    return speed_ms;
}

/* ============================================================
 *  位置式 PI + 前馈 速度环 (标准做法, 替代例程增量式以加快响应)
 *    err = target - actual
 *    integral += err;  integral 限幅 ±INTEG_MAX (抗积分饱和)
 *    output = Kp*err + Ki*integral + Kff*target
 *    前馈 Kff*target: target 一给, 基础 PWM 立刻到位, PI 只补差 → 响应快
 *    限幅 ±7999 (匹配 PWM timerCount=8000)
 *    返回 int (PWM 控制量, 正值; ISR 里 -Velocity 取反输出)
 * ============================================================ */
float Velcity_Kp  = 2.0f;    /* 比例: 瞬态响应 (err 大时给大输出) */
float Velcity_Ki  = 0.5f;    /* 积分: 消静差 (稳态 err→0 时 integral 维持) */
float Velcity_Kff = 200.0f;  /* 前馈: 电池下稳态P=200×target, Kff=200 让前馈正好匹配, 积分≈0 不抵消 */
#define VEL_INTEG_MAX  4000   /* 积分限幅, 抗饱和 (小于输出限幅 7999 留前馈余量) */

static int vel_A_ctrl = 0, vel_A_last_bias = 0;   /* last_bias 保留兼容, 位置式不用但 Reset 不动 */
static int vel_B_ctrl = 0, vel_B_last_bias = 0;
static int vel_A_integral = 0, vel_B_integral = 0;   /* int 累加(没乘dt, Ki=0.5 配此量级) */

int Velocity_A(int TargetVelocity, int CurrentVelocity)
{
    int err = TargetVelocity - CurrentVelocity;

    int output = (int)(Velcity_Kp * (float)err
                     + Velcity_Ki * (float)vel_A_integral
                     + Velcity_Kff * (float)TargetVelocity);
    /* 抗积分饱和: 输出未触限时才累加积分 */
    if (output > 7999) {
        output = 7999;
    } else if (output < -7999) {
        output = -7999;
    } else {
        vel_A_integral += err;
    }
    if (vel_A_integral >  VEL_INTEG_MAX) vel_A_integral =  VEL_INTEG_MAX;
    if (vel_A_integral < -VEL_INTEG_MAX) vel_A_integral = -VEL_INTEG_MAX;

    vel_A_last_bias = err;   /* 兼容, 位置式实际不用 */
    vel_A_ctrl = output;     /* 供诊断/Reset */
    return output;
}

int Velocity_B(int TargetVelocity, int CurrentVelocity)
{
    int err = TargetVelocity - CurrentVelocity;

    int output = (int)(Velcity_Kp * (float)err
                     + Velcity_Ki * (float)vel_B_integral
                     + Velcity_Kff * (float)TargetVelocity);
    if (output > 7999) {
        output = 7999;
    } else if (output < -7999) {
        output = -7999;
    } else {
        vel_B_integral += err;
    }
    if (vel_B_integral >  VEL_INTEG_MAX) vel_B_integral =  VEL_INTEG_MAX;
    if (vel_B_integral < -VEL_INTEG_MAX) vel_B_integral = -VEL_INTEG_MAX;

    vel_B_last_bias = err;
    vel_B_ctrl = output;
    return output;
}

/* 重置 PI 内部状态 (K1 停车时调, 避免积分残留导致下次启动冲) */
void Vel_PI_Reset_A(void)
{
    vel_A_ctrl = 0;
    vel_A_last_bias = 0;
    vel_A_integral = 0;
}
void Vel_PI_Reset_B(void)
{
    vel_B_ctrl = 0;
    vel_B_last_bias = 0;
    vel_B_integral = 0;
}

/*
 * GPIOA 组中断: 编码器判向 (四相全在 GPIOA, ENCODERA_PORT=ENCODERB_PORT=GPIOA)
 * gpioA/gpioB 实为同一端口两次读取, 靠 PIN 位(22/12/24/17 互不重叠)区分左右轮
 */
void GROUP1_IRQHandler(void)
{
    uint32_t gpioA = DL_GPIO_getEnabledInterruptStatus(ENCODERA_PORT,
        ENCODERA_E1A_PIN | ENCODERA_E1B_PIN);
    uint32_t gpioB = DL_GPIO_getEnabledInterruptStatus(ENCODERB_PORT,
        ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);

    /* 左轮: E1A 跳变 */
    if (gpioA & ENCODERA_E1A_PIN) {
        if (!DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_E1B_PIN)) {
            Get_Encoder_countA--;
        } else {
            Get_Encoder_countA++;
        }
    }
    /* 左轮: E1B 跳变 */
    if (gpioA & ENCODERA_E1B_PIN) {
        if (!DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_E1A_PIN)) {
            Get_Encoder_countA++;
        } else {
            Get_Encoder_countA--;
        }
    }

    /* 右轮: E2A 跳变 */
    if (gpioB & ENCODERB_E2A_PIN) {
        if (!DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_E2B_PIN)) {
            Get_Encoder_countB--;
        } else {
            Get_Encoder_countB++;
        }
    }
    /* 右轮: E2B 跳变 */
    if (gpioB & ENCODERB_E2B_PIN) {
        if (!DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_E2A_PIN)) {
            Get_Encoder_countB++;
        } else {
            Get_Encoder_countB--;
        }
    }

    /* 清除中断标志 */
    DL_GPIO_clearInterruptStatus(ENCODERA_PORT, ENCODERA_E1A_PIN | ENCODERA_E1B_PIN);
    DL_GPIO_clearInterruptStatus(ENCODERB_PORT, ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);
}
