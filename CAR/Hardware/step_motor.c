/**
 * @file  step_motor.c
 * @brief 步进电机驱动实现 (双 4 相 8 拍)
 *
 * 来源: mspm0g3507_car/Hardware/Step_Motor.c
 * 移植: 2026-07-23 → CAR/Hardware/
 * 适配:
 *   - 去掉 #include "main.h", 用本工程 step_motor.h + board.h
 *   - 延时 mspm0_delay_ms() → delay_ms() (CAR board.c)
 *   - 引脚宏来自 step_motor.h (用户暂未接硬件, 占位)
 *
 * 电平约定: 低电平导通 (clearPins = 通电, setPins = 断电)
 */
#include "step_motor.h"
#include "board.h"   /* delay_ms() */

/* 步进电机每转一圈所需总步数 (1.8° 步进角 4 相 8 拍: 360°/0.0879° ≈ 4096) */
#define STEPS_PER_REVOLUTION 4096

/* 每种工作模式下, 调用一次 Direction 函数产生的步数 */
#define STEPS_PER_CALL_MODE_0 8   /* 4 相 8 拍: 每次 8 步 */
#define STEPS_PER_CALL_MODE_1 4   /* 4 相单 4 拍: 每次 4 步 */
#define STEPS_PER_CALL_MODE_2 4   /* 4 相双 4 拍: 每次 4 步 */

struct STEP_MOTOR Step_Motor_one;
struct STEP_MOTOR Step_Motor_two;

int Step_Motor_one_Original_Angle;
int tem_one_angle;
int Step_Motor_two_Original_Angle;
int tem_two_angle;

int trigonometry_a = 45;            /* 三角函数侧边运动基准步数 */
int step_motor_delay_time = 3;      /* 每步延时 ms (控制转速) */

/* 初始化: 所有相断电 (引脚初始化由 SysConfig 负责, 这里只清状态) */
void Step_Motor_Init(void)
{
    DL_GPIO_clearPins(Step_Motor_IN1_PORT, Step_Motor_IN1_PIN);
    DL_GPIO_clearPins(Step_Motor_IN2_PORT, Step_Motor_IN2_PIN);
    DL_GPIO_clearPins(Step_Motor_IN3_PORT, Step_Motor_IN3_PIN);
    DL_GPIO_clearPins(Step_Motor_IN4_PORT, Step_Motor_IN4_PIN);
}

/* 单步移动 (4 相双 4 拍变种, dir 控制方向) */
void Step_Motor_Move(struct STEP_MOTOR *step_motor, int8_t dir)
{
    step_motor->current_step += 4;
    step_motor->current_step += dir;
    step_motor->current_step %= 4;
    if (step_motor->current_step == 0)
    {
        DL_GPIO_setPins(Step_Motor_IN1_PORT, Step_Motor_IN1_PIN);
        DL_GPIO_setPins(Step_Motor_IN2_PORT, Step_Motor_IN2_PIN);
        DL_GPIO_clearPins(Step_Motor_IN3_PORT, Step_Motor_IN3_PIN);
        DL_GPIO_clearPins(Step_Motor_IN4_PORT, Step_Motor_IN4_PIN);
    }
    else if (step_motor->current_step == 1)
    {
        DL_GPIO_clearPins(Step_Motor_IN1_PORT, Step_Motor_IN1_PIN);
        DL_GPIO_setPins(Step_Motor_IN2_PORT, Step_Motor_IN2_PIN);
        DL_GPIO_setPins(Step_Motor_IN3_PORT, Step_Motor_IN3_PIN);
        DL_GPIO_clearPins(Step_Motor_IN4_PORT, Step_Motor_IN4_PIN);
    }
    else if (step_motor->current_step == 2)
    {
        DL_GPIO_clearPins(Step_Motor_IN1_PORT, Step_Motor_IN1_PIN);
        DL_GPIO_clearPins(Step_Motor_IN2_PORT, Step_Motor_IN2_PIN);
        DL_GPIO_setPins(Step_Motor_IN3_PORT, Step_Motor_IN3_PIN);
        DL_GPIO_setPins(Step_Motor_IN4_PORT, Step_Motor_IN4_PIN);
    }
    else if (step_motor->current_step == 3)
    {
        DL_GPIO_setPins(Step_Motor_IN1_PORT, Step_Motor_IN1_PIN);
        DL_GPIO_clearPins(Step_Motor_IN2_PORT, Step_Motor_IN2_PIN);
        DL_GPIO_clearPins(Step_Motor_IN3_PORT, Step_Motor_IN3_PIN);
        DL_GPIO_setPins(Step_Motor_IN4_PORT, Step_Motor_IN4_PIN);
    }
}

/**
 * @brief 第一组 4 相单 4 拍节拍
 * @param step 节拍步骤 (1-4)
 * @param dly  每步延时 ms
 */
void Step_Motor_Rhythm_4_1_4(uint8_t step, uint8_t dly)
{
    switch (step)
    {
        case 0:  break;  /* 空步骤 */
        case 1:  /* IN1 导通 */
            DL_GPIO_clearPins(Step_Motor_IN1_PORT, Step_Motor_IN1_PIN);
            DL_GPIO_setPins(Step_Motor_IN2_PORT, Step_Motor_IN2_PIN);
            DL_GPIO_setPins(Step_Motor_IN3_PORT, Step_Motor_IN3_PIN);
            DL_GPIO_setPins(Step_Motor_IN4_PORT, Step_Motor_IN4_PIN);
            break;
        case 2:  /* IN2 导通 */
            DL_GPIO_setPins(Step_Motor_IN1_PORT, Step_Motor_IN1_PIN);
            DL_GPIO_clearPins(Step_Motor_IN2_PORT, Step_Motor_IN2_PIN);
            DL_GPIO_setPins(Step_Motor_IN3_PORT, Step_Motor_IN3_PIN);
            DL_GPIO_setPins(Step_Motor_IN4_PORT, Step_Motor_IN4_PIN);
            break;
        case 3:  /* IN3 导通 */
            DL_GPIO_setPins(Step_Motor_IN1_PORT, Step_Motor_IN1_PIN);
            DL_GPIO_setPins(Step_Motor_IN2_PORT, Step_Motor_IN2_PIN);
            DL_GPIO_clearPins(Step_Motor_IN3_PORT, Step_Motor_IN3_PIN);
            DL_GPIO_setPins(Step_Motor_IN4_PORT, Step_Motor_IN4_PIN);
            break;
        case 4:  /* IN4 导通 */
            DL_GPIO_setPins(Step_Motor_IN1_PORT, Step_Motor_IN1_PIN);
            DL_GPIO_setPins(Step_Motor_IN2_PORT, Step_Motor_IN2_PIN);
            DL_GPIO_setPins(Step_Motor_IN3_PORT, Step_Motor_IN3_PIN);
            DL_GPIO_clearPins(Step_Motor_IN4_PORT, Step_Motor_IN4_PIN);
            break;
    }
    delay_ms(dly);   /* 延时控制转速 */
}

/**
 * @brief 第一组 4 相双 4 拍节拍 (两相邻相同时通电, 扭矩大)
 */
void Step_Motor_Rhythm_4_2_4(uint8_t step, uint8_t dly)
{
    switch (step)
    {
        case 0:  break;
        case 1:  /* IN1+IN4 导通 */
            DL_GPIO_clearPins(Step_Motor_IN1_PORT, Step_Motor_IN1_PIN);
            DL_GPIO_setPins(Step_Motor_IN2_PORT, Step_Motor_IN2_PIN);
            DL_GPIO_setPins(Step_Motor_IN3_PORT, Step_Motor_IN3_PIN);
            DL_GPIO_clearPins(Step_Motor_IN4_PORT, Step_Motor_IN4_PIN);
            break;
        case 2:  /* IN1+IN2 导通 */
            DL_GPIO_clearPins(Step_Motor_IN1_PORT, Step_Motor_IN1_PIN);
            DL_GPIO_clearPins(Step_Motor_IN2_PORT, Step_Motor_IN2_PIN);
            DL_GPIO_setPins(Step_Motor_IN3_PORT, Step_Motor_IN3_PIN);
            DL_GPIO_setPins(Step_Motor_IN4_PORT, Step_Motor_IN4_PIN);
            break;
        case 3:  /* IN2+IN3 导通 */
            DL_GPIO_setPins(Step_Motor_IN1_PORT, Step_Motor_IN1_PIN);
            DL_GPIO_clearPins(Step_Motor_IN2_PORT, Step_Motor_IN2_PIN);
            DL_GPIO_clearPins(Step_Motor_IN3_PORT, Step_Motor_IN3_PIN);
            DL_GPIO_setPins(Step_Motor_IN4_PORT, Step_Motor_IN4_PIN);
            break;
        case 4:  /* IN3+IN4 导通 */
            DL_GPIO_setPins(Step_Motor_IN1_PORT, Step_Motor_IN1_PIN);
            DL_GPIO_setPins(Step_Motor_IN2_PORT, Step_Motor_IN2_PIN);
            DL_GPIO_clearPins(Step_Motor_IN3_PORT, Step_Motor_IN3_PIN);
            DL_GPIO_clearPins(Step_Motor_IN4_PORT, Step_Motor_IN4_PIN);
            break;
    }
    delay_ms(dly);
}

/**
 * @brief 第一组 4 相 8 拍节拍 (单双拍交替, 步进角小, 运行平滑)
 */
void Step_Motor_Rhythm_4_1_8(uint8_t step, uint8_t dly)
{
    switch (step)
    {
        case 0:  break;
        case 1:  /* IN1 导通 */
            DL_GPIO_clearPins(Step_Motor_IN1_PORT, Step_Motor_IN1_PIN);
            DL_GPIO_setPins(Step_Motor_IN2_PORT, Step_Motor_IN2_PIN);
            DL_GPIO_setPins(Step_Motor_IN3_PORT, Step_Motor_IN3_PIN);
            DL_GPIO_setPins(Step_Motor_IN4_PORT, Step_Motor_IN4_PIN);
            break;
        case 2:  /* IN1+IN2 导通 */
            DL_GPIO_clearPins(Step_Motor_IN1_PORT, Step_Motor_IN1_PIN);
            DL_GPIO_clearPins(Step_Motor_IN2_PORT, Step_Motor_IN2_PIN);
            DL_GPIO_setPins(Step_Motor_IN3_PORT, Step_Motor_IN3_PIN);
            DL_GPIO_setPins(Step_Motor_IN4_PORT, Step_Motor_IN4_PIN);
            break;
        case 3:  /* IN2 导通 */
            DL_GPIO_setPins(Step_Motor_IN1_PORT, Step_Motor_IN1_PIN);
            DL_GPIO_clearPins(Step_Motor_IN2_PORT, Step_Motor_IN2_PIN);
            DL_GPIO_setPins(Step_Motor_IN3_PORT, Step_Motor_IN3_PIN);
            DL_GPIO_setPins(Step_Motor_IN4_PORT, Step_Motor_IN4_PIN);
            break;
        case 4:  /* IN2+IN3 导通 */
            DL_GPIO_setPins(Step_Motor_IN1_PORT, Step_Motor_IN1_PIN);
            DL_GPIO_clearPins(Step_Motor_IN2_PORT, Step_Motor_IN2_PIN);
            DL_GPIO_clearPins(Step_Motor_IN3_PORT, Step_Motor_IN3_PIN);
            DL_GPIO_setPins(Step_Motor_IN4_PORT, Step_Motor_IN4_PIN);
            break;
        case 5:  /* IN3 导通 */
            DL_GPIO_setPins(Step_Motor_IN1_PORT, Step_Motor_IN1_PIN);
            DL_GPIO_setPins(Step_Motor_IN2_PORT, Step_Motor_IN2_PIN);
            DL_GPIO_clearPins(Step_Motor_IN3_PORT, Step_Motor_IN3_PIN);
            DL_GPIO_setPins(Step_Motor_IN4_PORT, Step_Motor_IN4_PIN);
            break;
        case 6:  /* IN3+IN4 导通 */
            DL_GPIO_setPins(Step_Motor_IN1_PORT, Step_Motor_IN1_PIN);
            DL_GPIO_setPins(Step_Motor_IN2_PORT, Step_Motor_IN2_PIN);
            DL_GPIO_clearPins(Step_Motor_IN3_PORT, Step_Motor_IN3_PIN);
            DL_GPIO_clearPins(Step_Motor_IN4_PORT, Step_Motor_IN4_PIN);
            break;
        case 7:  /* IN4 导通 */
            DL_GPIO_setPins(Step_Motor_IN1_PORT, Step_Motor_IN1_PIN);
            DL_GPIO_setPins(Step_Motor_IN2_PORT, Step_Motor_IN2_PIN);
            DL_GPIO_setPins(Step_Motor_IN3_PORT, Step_Motor_IN3_PIN);
            DL_GPIO_clearPins(Step_Motor_IN4_PORT, Step_Motor_IN4_PIN);
            break;
        case 8:  /* IN4+IN1 导通 */
            DL_GPIO_clearPins(Step_Motor_IN1_PORT, Step_Motor_IN1_PIN);
            DL_GPIO_setPins(Step_Motor_IN2_PORT, Step_Motor_IN2_PIN);
            DL_GPIO_setPins(Step_Motor_IN3_PORT, Step_Motor_IN3_PIN);
            DL_GPIO_clearPins(Step_Motor_IN4_PORT, Step_Motor_IN4_PIN);
            break;
    }
    delay_ms(dly);
}

/**
 * @brief 第一组电机方向+模式控制
 * @param dir  1=正转, 0=反转
 * @param mode 0=4相8拍, 1=单4拍, 2=双4拍
 * @param dly  每步延时 ms
 */
void Step_Motor_Direction(struct STEP_MOTOR *step_motor, uint8_t dir, uint8_t mode, uint8_t dly)
{
    if (dir)  /* 正转: 1→2→3→... */
    {
        switch (mode)
        {
            case 0:  /* 8 拍 1-8 步 */
                for (uint8_t i = 1; i < 9; i++)
                {
                    Step_Motor_Rhythm_4_1_8(i, dly);
                    step_motor->current_step = (step_motor->current_step + 1) % 8;
                }
                break;
            case 1:  /* 单 4 拍 1-4 步 */
                for (uint8_t i = 1; i < 5; i++)
                {
                    Step_Motor_Rhythm_4_1_4(i, dly);
                    step_motor->current_step = (step_motor->current_step + 1) % 4;
                }
                break;
            case 2:  /* 双 4 拍 1-4 步 */
                for (uint8_t i = 1; i < 5; i++)
                {
                    Step_Motor_Rhythm_4_2_4(i, dly);
                    step_motor->current_step = (step_motor->current_step + 1) % 4;
                }
                break;
            default:
                break;
        }
    }
    else  /* 反转: ...→3→2→1 */
    {
        switch (mode)
        {
            case 0:
                for (uint8_t i = 8; i > 0; i--)
                {
                    Step_Motor_Rhythm_4_1_8(i, dly);
                    step_motor->current_step = (step_motor->current_step + 7) % 8;
                }
                break;
            case 1:
                for (uint8_t i = 4; i > 0; i--)
                {
                    Step_Motor_Rhythm_4_1_4(i, dly);
                    step_motor->current_step = (step_motor->current_step + 3) % 4;
                }
                break;
            case 2:
                for (uint8_t i = 4; i > 0; i--)
                {
                    Step_Motor_Rhythm_4_2_4(i, dly);
                    step_motor->current_step = (step_motor->current_step + 3) % 4;
                }
                break;
            default:
                break;
        }
    }
}

/**
 * @brief 第一组电机旋转指定角度
 * @param angle 旋转角度 (度)
 */
void Step_Motor_Rotate_Angle(struct STEP_MOTOR *step_motor, uint8_t dir, uint8_t mode, uint16_t angle, uint8_t dly)
{
    /* 总步数 = 每圈步数 × 角度 / 360° */
    uint16_t steps = (uint16_t)((float)STEPS_PER_REVOLUTION * angle / 360.0f);
    uint16_t steps_per_call = 0;

    switch (mode) {
        case 0: steps_per_call = STEPS_PER_CALL_MODE_0; break;
        case 1: steps_per_call = STEPS_PER_CALL_MODE_1; break;
        case 2: steps_per_call = STEPS_PER_CALL_MODE_2; break;
        default: return;   /* 无效模式 */
    }

    uint16_t calls = steps / steps_per_call;
    step_motor->remain_steps = steps % steps_per_call;

    for (uint16_t i = 0; i < calls; i++)
    {
        Step_Motor_Direction(step_motor, dir, mode, dly);
    }
}

/* 停止: 所有相断电 */
void Step_Motor_Stop(struct STEP_MOTOR *step_motor)
{
    DL_GPIO_clearPins(Step_Motor_IN1_PORT, Step_Motor_IN1_PIN);
    DL_GPIO_clearPins(Step_Motor_IN2_PORT, Step_Motor_IN2_PIN);
    DL_GPIO_clearPins(Step_Motor_IN3_PORT, Step_Motor_IN3_PIN);
    DL_GPIO_clearPins(Step_Motor_IN4_PORT, Step_Motor_IN4_PIN);
    step_motor->remain_steps = 0;
}


/* ====================================================================== */
/* 第二组电机 (BIN 引脚) —— 逻辑与第一组完全相同, 只是操作 BIN1~4 引脚         */
/* ====================================================================== */

void Step_Motor_two_Rhythm_4_1_4(uint8_t step, uint8_t dly)
{
    switch (step)
    {
        case 0:  break;
        case 1:
            DL_GPIO_clearPins(Step_Motor_BIN1_PORT, Step_Motor_BIN1_PIN);
            DL_GPIO_setPins(Step_Motor_BIN2_PORT, Step_Motor_BIN2_PIN);
            DL_GPIO_setPins(Step_Motor_BIN3_PORT, Step_Motor_BIN3_PIN);
            DL_GPIO_setPins(Step_Motor_BIN4_PORT, Step_Motor_BIN4_PIN);
            break;
        case 2:
            DL_GPIO_setPins(Step_Motor_BIN1_PORT, Step_Motor_BIN1_PIN);
            DL_GPIO_clearPins(Step_Motor_BIN2_PORT, Step_Motor_BIN2_PIN);
            DL_GPIO_setPins(Step_Motor_BIN3_PORT, Step_Motor_BIN3_PIN);
            DL_GPIO_setPins(Step_Motor_BIN4_PORT, Step_Motor_BIN4_PIN);
            break;
        case 3:
            DL_GPIO_setPins(Step_Motor_BIN1_PORT, Step_Motor_BIN1_PIN);
            DL_GPIO_setPins(Step_Motor_BIN2_PORT, Step_Motor_BIN2_PIN);
            DL_GPIO_clearPins(Step_Motor_BIN3_PORT, Step_Motor_BIN3_PIN);
            DL_GPIO_setPins(Step_Motor_BIN4_PORT, Step_Motor_BIN4_PIN);
            break;
        case 4:
            DL_GPIO_setPins(Step_Motor_BIN1_PORT, Step_Motor_BIN1_PIN);
            DL_GPIO_setPins(Step_Motor_BIN2_PORT, Step_Motor_BIN2_PIN);
            DL_GPIO_setPins(Step_Motor_BIN3_PORT, Step_Motor_BIN3_PIN);
            DL_GPIO_clearPins(Step_Motor_BIN4_PORT, Step_Motor_BIN4_PIN);
            break;
    }
    delay_ms(dly);
}

void Step_Motor_two_Rhythm_4_2_4(uint8_t step, uint8_t dly)
{
    switch (step)
    {
        case 0:  break;
        case 1:
            DL_GPIO_clearPins(Step_Motor_BIN1_PORT, Step_Motor_BIN1_PIN);
            DL_GPIO_setPins(Step_Motor_BIN2_PORT, Step_Motor_BIN2_PIN);
            DL_GPIO_setPins(Step_Motor_BIN3_PORT, Step_Motor_BIN3_PIN);
            DL_GPIO_clearPins(Step_Motor_BIN4_PORT, Step_Motor_BIN4_PIN);
            break;
        case 2:
            DL_GPIO_clearPins(Step_Motor_BIN1_PORT, Step_Motor_BIN1_PIN);
            DL_GPIO_clearPins(Step_Motor_BIN2_PORT, Step_Motor_BIN2_PIN);
            DL_GPIO_setPins(Step_Motor_BIN3_PORT, Step_Motor_BIN3_PIN);
            DL_GPIO_setPins(Step_Motor_BIN4_PORT, Step_Motor_BIN4_PIN);
            break;
        case 3:
            DL_GPIO_setPins(Step_Motor_BIN1_PORT, Step_Motor_BIN1_PIN);
            DL_GPIO_clearPins(Step_Motor_BIN2_PORT, Step_Motor_BIN2_PIN);
            DL_GPIO_clearPins(Step_Motor_BIN3_PORT, Step_Motor_BIN3_PIN);
            DL_GPIO_setPins(Step_Motor_BIN4_PORT, Step_Motor_BIN4_PIN);
            break;
        case 4:
            DL_GPIO_setPins(Step_Motor_BIN1_PORT, Step_Motor_BIN1_PIN);
            DL_GPIO_setPins(Step_Motor_BIN2_PORT, Step_Motor_BIN2_PIN);
            DL_GPIO_clearPins(Step_Motor_BIN3_PORT, Step_Motor_BIN3_PIN);
            DL_GPIO_clearPins(Step_Motor_BIN4_PORT, Step_Motor_BIN4_PIN);
            break;
    }
    delay_ms(dly);
}

void Step_Motor_two_Rhythm_4_1_8(uint8_t step, uint8_t dly)
{
    switch (step)
    {
        case 0:  break;
        case 1:
            DL_GPIO_clearPins(Step_Motor_BIN1_PORT, Step_Motor_BIN1_PIN);
            DL_GPIO_setPins(Step_Motor_BIN2_PORT, Step_Motor_BIN2_PIN);
            DL_GPIO_setPins(Step_Motor_BIN3_PORT, Step_Motor_BIN3_PIN);
            DL_GPIO_setPins(Step_Motor_BIN4_PORT, Step_Motor_BIN4_PIN);
            break;
        case 2:
            DL_GPIO_clearPins(Step_Motor_BIN1_PORT, Step_Motor_BIN1_PIN);
            DL_GPIO_clearPins(Step_Motor_BIN2_PORT, Step_Motor_BIN2_PIN);
            DL_GPIO_setPins(Step_Motor_BIN3_PORT, Step_Motor_BIN3_PIN);
            DL_GPIO_setPins(Step_Motor_BIN4_PORT, Step_Motor_BIN4_PIN);
            break;
        case 3:
            DL_GPIO_setPins(Step_Motor_BIN1_PORT, Step_Motor_BIN1_PIN);
            DL_GPIO_clearPins(Step_Motor_BIN2_PORT, Step_Motor_BIN2_PIN);
            DL_GPIO_setPins(Step_Motor_BIN3_PORT, Step_Motor_BIN3_PIN);
            DL_GPIO_setPins(Step_Motor_BIN4_PORT, Step_Motor_BIN4_PIN);
            break;
        case 4:
            DL_GPIO_setPins(Step_Motor_BIN1_PORT, Step_Motor_BIN1_PIN);
            DL_GPIO_clearPins(Step_Motor_BIN2_PORT, Step_Motor_BIN2_PIN);
            DL_GPIO_clearPins(Step_Motor_BIN3_PORT, Step_Motor_BIN3_PIN);
            DL_GPIO_setPins(Step_Motor_BIN4_PORT, Step_Motor_BIN4_PIN);
            break;
        case 5:
            DL_GPIO_setPins(Step_Motor_BIN1_PORT, Step_Motor_BIN1_PIN);
            DL_GPIO_setPins(Step_Motor_BIN2_PORT, Step_Motor_BIN2_PIN);
            DL_GPIO_clearPins(Step_Motor_BIN3_PORT, Step_Motor_BIN3_PIN);
            DL_GPIO_setPins(Step_Motor_BIN4_PORT, Step_Motor_BIN4_PIN);
            break;
        case 6:
            DL_GPIO_setPins(Step_Motor_BIN1_PORT, Step_Motor_BIN1_PIN);
            DL_GPIO_setPins(Step_Motor_BIN2_PORT, Step_Motor_BIN2_PIN);
            DL_GPIO_clearPins(Step_Motor_BIN3_PORT, Step_Motor_BIN3_PIN);
            DL_GPIO_clearPins(Step_Motor_BIN4_PORT, Step_Motor_BIN4_PIN);
            break;
        case 7:
            DL_GPIO_setPins(Step_Motor_BIN1_PORT, Step_Motor_BIN1_PIN);
            DL_GPIO_setPins(Step_Motor_BIN2_PORT, Step_Motor_BIN2_PIN);
            DL_GPIO_setPins(Step_Motor_BIN3_PORT, Step_Motor_BIN3_PIN);
            DL_GPIO_clearPins(Step_Motor_BIN4_PORT, Step_Motor_BIN4_PIN);
            break;
        case 8:
            DL_GPIO_clearPins(Step_Motor_BIN1_PORT, Step_Motor_BIN1_PIN);
            DL_GPIO_setPins(Step_Motor_BIN2_PORT, Step_Motor_BIN2_PIN);
            DL_GPIO_setPins(Step_Motor_BIN3_PORT, Step_Motor_BIN3_PIN);
            DL_GPIO_clearPins(Step_Motor_BIN4_PORT, Step_Motor_BIN4_PIN);
            break;
    }
    delay_ms(dly);
}

void Step_Motor_two_Direction(struct STEP_MOTOR *step_motor, uint8_t dir, uint8_t mode, uint8_t dly)
{
    if (dir)
    {
        switch (mode)
        {
            case 0:
                for (uint8_t i = 1; i < 9; i++)
                {
                    Step_Motor_two_Rhythm_4_1_8(i, dly);
                    step_motor->current_step = (step_motor->current_step + 1) % 8;
                }
                break;
            case 1:
                for (uint8_t i = 1; i < 5; i++)
                {
                    Step_Motor_two_Rhythm_4_1_4(i, dly);
                    step_motor->current_step = (step_motor->current_step + 1) % 4;
                }
                break;
            case 2:
                for (uint8_t i = 1; i < 5; i++)
                {
                    Step_Motor_two_Rhythm_4_2_4(i, dly);
                    step_motor->current_step = (step_motor->current_step + 1) % 4;
                }
                break;
            default:
                break;
        }
    }
    else
    {
        switch (mode)
        {
            case 0:
                for (uint8_t i = 8; i > 0; i--)
                {
                    Step_Motor_two_Rhythm_4_1_8(i, dly);
                    step_motor->current_step = (step_motor->current_step + 7) % 8;
                }
                break;
            case 1:
                for (uint8_t i = 4; i > 0; i--)
                {
                    Step_Motor_two_Rhythm_4_1_4(i, dly);
                    step_motor->current_step = (step_motor->current_step + 3) % 4;
                }
                break;
            case 2:
                for (uint8_t i = 4; i > 0; i--)
                {
                    Step_Motor_two_Rhythm_4_2_4(i, dly);
                    step_motor->current_step = (step_motor->current_step + 3) % 4;
                }
                break;
            default:
                break;
        }
    }
}

void Step_Motor_two_Rotate_Angle(struct STEP_MOTOR *step_motor, uint8_t dir, uint8_t mode, uint16_t angle, uint8_t dly)
{
    uint16_t steps = (uint16_t)((float)STEPS_PER_REVOLUTION * angle / 360.0f);
    uint16_t steps_per_call = 0;

    switch (mode) {
        case 0: steps_per_call = STEPS_PER_CALL_MODE_0; break;
        case 1: steps_per_call = STEPS_PER_CALL_MODE_1; break;
        case 2: steps_per_call = STEPS_PER_CALL_MODE_2; break;
        default: return;
    }

    uint16_t calls = steps / steps_per_call;
    step_motor->remain_steps = steps % steps_per_call;

    for (uint16_t i = 0; i < calls; i++)
    {
        Step_Motor_two_Direction(step_motor, dir, mode, dly);
    }
}

/**
 * @brief 双电机三角函数侧边运动 (Bresenham 同步)
 * @details 电机二步数为电机一的 1/3, 形成特定曲线轨迹
 * @param direction_one 电机一方向 (正/负)
 * @param direction_two 电机二方向 (正/负)
 */
void Step_Motor_Trigonometry_Side(int direction_one, int direction_two)
{
    float ratio = 3;
    int total_steps_one = trigonometry_a;
    int total_steps_two = trigonometry_a / ratio;   /* 电机二步数减半 */

    /* Bresenham 误差项 */
    int error = total_steps_two * ratio - total_steps_one;

    int steps_one_executed = 0;
    int steps_two_executed = 0;

    int step_one_pos = 1;
    int step_two_pos = 1;

    int step_one_count = 0;
    int step_two_count = 0;

    while (steps_one_executed < total_steps_one || steps_two_executed < total_steps_two)
    {
        /* 电机一 */
        if (steps_one_executed <= total_steps_one && step_one_count <= 8) {
            int step_one = (direction_one > 0) ? step_one_pos : 9 - step_one_pos;
            Step_Motor_Rhythm_4_1_8(step_one, step_motor_delay_time);
            step_one_pos = (step_one_pos % 8) + 1;
            step_one_count++;

            if (step_one_count >= 8) {
                steps_one_executed++;
                Step_Motor_one_Original_Angle += direction_one;
                step_one_count = 0;
            }
        }

        /* 电机二 */
        if (steps_two_executed <= total_steps_two && step_two_count <= 8) {
            if (error >= 0) {
                int step_two = (direction_two > 0) ? step_two_pos : 9 - step_two_pos;
                Step_Motor_two_Rhythm_4_1_8(step_two, step_motor_delay_time);
                step_two_pos = (step_two_pos % 8) + 1;
                step_two_count++;

                if (step_two_count >= 8) {
                    steps_two_executed++;
                    Step_Motor_two_Original_Angle += direction_two;
                    step_two_count = 0;
                }
            }
        }

        /* 更新误差项 */
        if (error >= 0) {
            error -= total_steps_one;
        }
        error += total_steps_two;
    }
}
