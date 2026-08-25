#include "motor.h"
#include "ti_msp_dl_config.h"

/* 照搬例程: PWM 限幅 (速度环用) */
int limit_PWM(int value, int low, int high)
{
    if (value > high) return high;
    else if (value < low) return low;
    else return value;
}

/*
 * 电机底层驱动 (TB6612FNG)
 * 包含完整的正反转逻辑和限幅
 *
 * 硬件接线 (实物原理图):
 *   A通道 (AIN): AIN1=PB12, AIN2=PB13, PWM=PA8(TIMG8 CCP0) → 接电机 A (左轮)
 *   B通道 (BIN): BIN1=PB2, BIN2=PB3, PWM=PA9(TIMG8 CCP1) → 接电机 B (右轮)
 *   引脚宏由 syscfg Motor 实例生成: Motor_PORT(组级,=GPIOB) + Motor_AIN1/AIN2/BIN1/BIN2_PIN
 *
 * 注意: Set_PWM(pwmA, pwmB) 中 pwmA=左轮(A通道), pwmB=右轮(B通道)
 *       Car_Move(PL, PR) 中 PL=左轮, PR=右轮, 与 Set_PWM 顺序一致
 */

/*
 * 底层PWM输出 (TB6612方向控制 + 定时器PWM)
 */
void Set_PWM(int pwmA, int pwmB)
{
    /* A 通道 (AIN: AIN1=PB12, AIN2=PB13, PWM_C0=PA8)
     * 方向条件按移植源 (CAR 例程) 原样: pwmA>0 → 置AIN2清AIN1.
     * 引脚宏用 CAR syscfg 的 Motor_PORT/Motor_AIN1_PIN.
     * 方向最终对不对由 CALIB_OPEN_LOOP 开环标定确认, 不在此处推断. */
    if (pwmA > 0) {
        DL_GPIO_setPins(Motor_PORT, Motor_AIN1_PIN);
        DL_GPIO_clearPins(Motor_PORT, Motor_AIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, pwmA, GPIO_PWM_0_C0_IDX);
    } else if (pwmA < 0) {
        DL_GPIO_setPins(Motor_PORT, Motor_AIN2_PIN);
        DL_GPIO_clearPins(Motor_PORT, Motor_AIN1_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, -pwmA, GPIO_PWM_0_C0_IDX);
    } else {
        DL_GPIO_clearPins(Motor_PORT, Motor_AIN1_PIN);
        DL_GPIO_clearPins(Motor_PORT, Motor_AIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
    }

    /* B 通道 (BIN: BIN1=PB2, BIN2=PB3, PWM_C1=PA9) — 同 A 通道, 正反转交换适配前驱 */
    if (pwmB > 0) {
        DL_GPIO_setPins(Motor_PORT, Motor_BIN1_PIN);
        DL_GPIO_clearPins(Motor_PORT, Motor_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, pwmB, GPIO_PWM_0_C1_IDX);
    } else if (pwmB < 0) {
        DL_GPIO_setPins(Motor_PORT, Motor_BIN2_PIN);
        DL_GPIO_clearPins(Motor_PORT, Motor_BIN1_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, -pwmB, GPIO_PWM_0_C1_IDX);
    } else {
        DL_GPIO_clearPins(Motor_PORT, Motor_BIN1_PIN);
        DL_GPIO_clearPins(Motor_PORT, Motor_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C1_IDX);
    }
}

/*
 * 短接制动 (TB6612: IN1=HIGH, IN2=HIGH, PWM=0)
 * 用于停车锁0, 比 Car_Move(0,0) 滑行更有力度
 */
void Motor_Brake(void)
{
    /* A 通道短接 */
    DL_GPIO_setPins(Motor_PORT, Motor_AIN1_PIN);
    DL_GPIO_setPins(Motor_PORT, Motor_AIN2_PIN);
    DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);

    /* B 通道短接 */
    DL_GPIO_setPins(Motor_PORT, Motor_BIN1_PIN);
    DL_GPIO_setPins(Motor_PORT, Motor_BIN2_PIN);
    DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C1_IDX);
}

/*
 * 统一电机输出接口
 * @param PL: 左轮PWM (正=前进, 负=后退)
 * @param PR: 右轮PWM (正=前进, 负=后退)
 * 限幅 ±PWM_MAX: 必须匹配 PWM 定时器 timerCount (syscfg PWM_0.timerCount=8000),
 *   比较值超出计数周期会导致 PWM 输出异常 (非满占空比), 表现为大 PWM 反而转得慢.
 */
#define PWM_MAX 7999   /* = PWM_0.timerCount-1, 比较值有效范围 0~7999 */
void Car_Move(double PL, double PR)
{
    int pwm_left  = (int)PL;
    int pwm_right = (int)PR;

    /* 限幅 ±PWM_MAX (匹配 timerCount=8000) */
    if (pwm_left  >  PWM_MAX) pwm_left  =  PWM_MAX;
    if (pwm_left  < -PWM_MAX) pwm_left  = -PWM_MAX;
    if (pwm_right >  PWM_MAX) pwm_right =  PWM_MAX;
    if (pwm_right < -PWM_MAX) pwm_right = -PWM_MAX;

    /* Set_PWM(pwmA, pwmB): 实物 pwmA=左轮(AIN/PA8), pwmB=右轮(BIN/PA9).
     * Car_Move(PL,PR): PL=左, PR=右. 故 Set_PWM(左, 右). */
    Set_PWM(pwm_left, pwm_right);
}
