/**
 * @file  gyro_serial.c
 * @brief 串口单轴陀螺仪驱动实现 (0x5A 协议, UART1 PB6=TX / PB7=RX, 115200bps)
 *
 * 移植自 2025 电赛 E题循迹工程 gyro_serial.c, 替换原 MPU6050 DMP yaw.
 * 注意: 本项目 syscfg 给 UART1 开了 loopback (DL_UART_Main_enableLoopbackMode),
 *       且未在 syscfg 里使能 RX 中断. Gyro_Init() 里手动关闭 loopback 并使能
 *       RX 中断 + NVIC, 使陀螺仪外部数据能进入 RX.
 */
#include "gyro_serial.h"
#include <string.h>

/* 陀螺仪数据全局实例 */
SAngle stcAngle;
SGyro  stcGyro;

/* ========== 校准指令定义 ========== */
static const uint8_t CMD_Key[5]      = {0x55, 0xAA, 0x13, 0x8E, 0x5F};  /* 解锁寄存器 */
static const uint8_t CMD_YawZero[5]  = {0x55, 0xAA, 0x15, 0x00, 0x00};  /* Z轴角度归零 */
static const uint8_t CMD_Save[5]     = {0x55, 0xAA, 0x00, 0x00, 0x00};  /* 保存配置 */
static const uint8_t CMD_BiasCal[5]  = {0x55, 0xAA, 0x0A, 0x01, 0x00};  /* 零偏校准 */

/* ========== 内部延时 (基于 CPU 周期) ========== */
static void delay_us_gyro(uint32_t us)
{
    DL_Common_delayCycles((CPUCLK_FREQ / 1000000UL) * us);
}

static void delay_ms_gyro(uint32_t ms)
{
    DL_Common_delayCycles((CPUCLK_FREQ / 1000UL) * ms);
}

/* ========== UART 发送 ========== */
static void Gyro_UART_SendByte(uint8_t data)
{
    DL_UART_Main_transmitDataBlocking(UART_1_INST, data);
}

static void Gyro_UART_SendBuffer(const uint8_t *buf, uint32_t len)
{
    uint32_t i;
    for (i = 0; i < len; i++) {
        Gyro_UART_SendByte(buf[i]);
    }
}

/* ========== 校准指令发送 ========== */

void Gyro_SendCalibrate(void)
{
    Gyro_UART_SendBuffer(CMD_Key, 5);
    delay_ms_gyro(100);
    Gyro_UART_SendBuffer(CMD_YawZero, 5);
    delay_ms_gyro(100);
    Gyro_UART_SendBuffer(CMD_Save, 5);
}

void Gyro_SendBiasCal(void)
{
    Gyro_UART_SendBuffer(CMD_Key, 5);
    delay_ms_gyro(100);
    Gyro_UART_SendBuffer(CMD_BiasCal, 5);
    delay_ms_gyro(21000);   /* 等待 21 秒完成校准, 期间勿移动 */
    Gyro_UART_SendBuffer(CMD_Save, 5);
}

/* ========== 协议解析状态机 ========== */

void Gyro_ParseByte(unsigned char ucData)
{
    static unsigned char ucRxBuffer[5];
    static unsigned char ucRxCnt = 0;

    ucRxBuffer[ucRxCnt++] = ucData;

    /* 帧头必须为 0x5A */
    if (ucRxBuffer[0] != 0x5A) {
        ucRxCnt = 0;
        return;
    }

    /* 等待收满 5 字节 */
    if (ucRxCnt < 5) {
        return;
    }

    unsigned char sum = 0;

    if (ucRxBuffer[1] == 0xAA) {
        /* ---- 角速度帧 (0x5A 0xAA) ---- */
        sum = ucRxBuffer[0] + ucRxBuffer[1]
            + ucRxBuffer[2] + ucRxBuffer[3];

        if (sum != ucRxBuffer[4]) {
            ucRxCnt = 0;
            return;
        }

        short wz = (short)((ucRxBuffer[3] << 8) | ucRxBuffer[2]);
        stcGyro.wz = (float)wz / 32768.0f * 2000.0f;   /* deg/s */
    }
    else if (ucRxBuffer[1] == 0xBB) {
        /* ---- 角度帧 (0x5A 0xBB) ---- */
        sum = ucRxBuffer[0] + ucRxBuffer[1]
            + ucRxBuffer[2] + ucRxBuffer[3];

        if (sum != ucRxBuffer[4]) {
            ucRxCnt = 0;
            return;
        }

        short rawYaw = (short)((ucRxBuffer[3] << 8) | ucRxBuffer[2]);
        stcAngle.Yaw = (float)rawYaw / 32768.0f * 180.0f;   /* deg, -180 ~ +180 */
    }
    /* 其他类型帧静默丢弃 */

    ucRxCnt = 0;   /* 复位, 等下一帧 */
}

/* ========== 初始化 ========== */

void Gyro_Init(void)
{
    /* 清零数据结构 */
    memset(&stcAngle, 0, sizeof(stcAngle));
    memset(&stcGyro,  0, sizeof(stcGyro));

    /*
     * 本项目 syscfg 给 UART1 开了 loopback, 会导致外部 RX 收不到数据.
     * 这里手动关闭 loopback, 让 PB7(RX) 真正接陀螺仪 TX.
     */
    DL_UART_Main_disableLoopbackMode(UART_1_INST);

    /* 使能 UART1 接收中断 (syscfg 未配, 这里补) */
    DL_UART_Main_clearInterruptStatus(UART_1_INST,
        DL_UART_INTERRUPT_RX | DL_UART_INTERRUPT_OVERRUN_ERROR);
    DL_UART_Main_enableInterrupt(UART_1_INST,
        DL_UART_INTERRUPT_RX | DL_UART_INTERRUPT_OVERRUN_ERROR);

    /* 清除并使能 UART1 NVIC 中断 */
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);

    /* 等串口稳定 */
    delay_ms_gyro(100);

    /* 发送 Z 轴归零校准 */
    Gyro_SendCalibrate();
}

/* ========== 数据访问函数 ========== */

float GyroZ(void)
{
    return stcGyro.wz;
}

float Yaw(void)
{
    return stcAngle.Yaw;
}
