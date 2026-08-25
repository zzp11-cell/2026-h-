/**
 * @file  k230.c
 * @brief K230 视觉板通信驱动实现
 *
 * 来源: mspm0g3507_car/Hardware/K230.c + Hardware/UART3.c (ISR 合并)
 * 移植: 2026-07-23 → CAR/Hardware/
 * 适配:
 *   - 去掉 #include "main.h", 用本工程 k230.h + gyro_serial.h + motor.h
 *   - Yaw_arr[Yaw_real] (int) → Yaw() (float, gyro_serial.h): 直线 PID 误差改用 float
 *   - Set_Speed(l,r) → Set_PWM(pwmA,pwmB) (motor.h, 签名一致)
 *   - 原 UART3.c 的 UART3_IRQHandler 已删除 (ISR 应在 main.c 统一管理, 见下方说明)
 *
 * ⚠️ ISR 说明: 源项目在 UART3.c 里定义 UART3_IRQHandler。
 *    CAR 风格是 ISR 写在 main.c (参考其 UART_1_INST_IRQHandler 模式)。
 *    所以这里不定义 ISR, 只提供 K230_ReceiveData 供 main.c 的 ISR 调用:
 *
 *      void UART_3_INST_IRQHandler(void) {
 *          if (DL_UART_getPendingInterrupt(K230_UART_INST) == DL_UART_IIDX_RX) {
 *              uint8_t b = (uint8_t)DL_UART_receiveData(K230_UART_INST);
 *              K230_ReceiveData(b);
 *          }
 *      }
 */
#include "k230.h"
#include "gyro_serial.h"   /* Yaw() */
#include "motor.h"          /* Set_PWM() */

int detect_count = 0;   /* 检测到停止位的次数 */

static uint8_t         K230_RxBuffer[K230_FRAME_MAX_PAYLOAD];        /* 接收数据数组 */
static uint8_t         K230_FrameBuffer[K230_FRAME_MAX_PAYLOAD];
static volatile uint8_t K230_RxState = 0;        /* 接收状态标志位 */
static uint8_t         K230_RxIndex = 0;          /* 接收数组索引 */
static volatile uint8_t s_k230_frame_ready = 0;
static volatile uint8_t s_k230_frame_len = 0;

static void K230_ResetParser(void)
{
    K230_RxState = 0;
    K230_RxIndex = 0;
}

/* 初始化: 使能 UART RX 中断 (实例由 k230.h 的 K230_UART_INST 指定) */
void K230_Init(void)
{
    NVIC_ClearPendingIRQ(K230_UART_IRQ);
    NVIC_EnableIRQ(K230_UART_IRQ);
    DL_UART_clearInterruptStatus(K230_UART_INST, DL_UART_INTERRUPT_RX);   /* 清 RX 中断标志 */
}

/* 直线行走 PID: 用 Yaw() 做航向闭环, 输出左右轮 PWM */
/* target: 要走的直线目标航向角 (deg) */
static float k230_line_integral  = 0;   /* 积分项 */
static float k230_previous_error = 0;   /* 上一次误差 */
void K230_Straght_Line_PID(int target)
{
    float KP = 1.2f;
    float KI = 0.05f;
    float KD = 0.0f;

    /* 当前误差 = 目标航向 - 当前 Yaw (源项目用 Yaw_arr[Yaw_real], 这里改用 Yaw() float) */
    float error = (float)target - Yaw();

    /* 积分项累加 + 限幅 */
    k230_line_integral += error;
    if (k230_line_integral > 10)       k230_line_integral = 10;
    else if (k230_line_integral < -10) k230_line_integral = -10;

    /* 微分项 */
    float derivative = error - k230_previous_error;

    /* PID 输出 */
    float pid_output = KP * error + KI * k230_line_integral + KD * derivative;

    /* 根据误差分配左右轮速度 (源项目基准 12, 右轮额外 +6 补偿) */
    int left_speed  = 12 - (int)pid_output;
    int right_speed = 12 + (int)pid_output + 6;

    /* 速度限幅 */
    if (left_speed < -50)  left_speed = -50;
    if (left_speed > 50)   left_speed = 50;
    if (right_speed < -50) right_speed = -50;
    if (right_speed > 50)  right_speed = 50;

    /* 输出 (源项目 Set_Speed → CAR Set_PWM, 签名一致) */
    Set_PWM(left_speed, right_speed);

    k230_previous_error = error;
}

/* 帧解析状态机: 在 UART ISR 中逐字节调用 */
void K230_ReceiveData(uint8_t RxData)
{
    if (K230_RxState == 0)          /* 等待起始符 FF */
    {
        if (RxData == 0xFF)
        {
            K230_RxState = 1;
            K230_RxIndex = 0;
        }
    }
    else if (K230_RxState == 1)     /* 接收数据, 直到遇到结束符 FE */
    {
        if (RxData == 0xFE)
        {
            for (uint8_t i = 0; i < K230_RxIndex; i++) {
                K230_FrameBuffer[i] = K230_RxBuffer[i];
            }
            s_k230_frame_len = K230_RxIndex;
            s_k230_frame_ready = 1;
            detect_count++;   /* 帧计数 (供主循环查询) */
            K230_ResetParser();
        }
        else
        {
            if (K230_RxIndex >= K230_FRAME_MAX_PAYLOAD) {
                K230_ResetParser();
            } else {
                K230_RxBuffer[K230_RxIndex++] = RxData;
            }
        }
    }
}

uint8_t K230_FrameAvailable(void)
{
    return s_k230_frame_ready;
}

uint8_t K230_ReadFrame(uint8_t *dst, uint8_t max_len)
{
    uint8_t len;

    if (!s_k230_frame_ready || dst == 0 || max_len == 0) return 0;

    __disable_irq();
    len = s_k230_frame_len;
    if (len > max_len) len = max_len;
    for (uint8_t i = 0; i < len; i++) {
        dst[i] = K230_FrameBuffer[i];
    }
    s_k230_frame_ready = 0;
    __enable_irq();

    return len;
}
