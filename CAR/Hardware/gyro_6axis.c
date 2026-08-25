/**
 * @file  gyro_6axis.c
 * @brief 六轴串口陀螺仪驱动实现 (0x5A 协议, 11字节帧, UART1 PB4=TX/PB5=RX)
 *
 * 移植自资料 GyroPID -AXIS6/board.c 的 CopeSerial2Data.
 * syscfg 已给 UART1 配 RX 中断 (见 ti_msp_dl_config.c SYSCFG_DL_UART_1_init),
 * 本驱动 Init 仍补做 clearInterruptStatus + enableInterrupt + disableLoopbackMode,
 * 与单轴 gyro_serial.c 对齐, 确保上电后 pending 干净、RX 中断真正触发。
 */
#include "gyro_6axis.h"
#include <string.h>

/*==== 全局数据实例 ====*/
SAngle6 stcAngle6;
SGyro6  stcGyro6;
SAccel6 stcAccel6;
SQuat6  stcQuat6;


/*==== 校准指令 (5字节) ====*/
static const uint8_t CMD_Key[5]     = {0x55, 0xAA, 0x13, 0x8E, 0x5F}; /* 解锁 */
static const uint8_t CMD_YawZero[5] = {0x55, 0xAA, 0x0A, 0x04, 0x00}; /* Z轴归零 */
static const uint8_t CMD_Save[5]    = {0x55, 0xAA, 0x00, 0x00, 0x00}; /* 保存 */
static const uint8_t CMD_BiasCal[5] = {0x55, 0xAA, 0x0A, 0x01, 0x00}; /* 零偏校准 */

/*==== 延时 (基于CPU周期) ====*/
static void delay_us_g6(uint32_t us) { DL_Common_delayCycles((CPUCLK_FREQ / 1000000UL) * us); }
static void delay_ms_g6(uint32_t ms) { DL_Common_delayCycles((CPUCLK_FREQ / 1000UL) * ms); }

/*==== UART 发送 ====*/
static void Gyro6_UART_SendBuffer(const uint8_t *buf, uint32_t len)
{
    uint32_t i;
    for (i = 0; i < len; i++)
        DL_UART_Main_transmitDataBlocking(UART_1_INST, buf[i]);
}

/*==== 校准指令发送 ====*/
void Gyro6_SendCalibrate(void)
{
    Gyro6_UART_SendBuffer(CMD_Key, 5);
    delay_ms_g6(100);
    Gyro6_UART_SendBuffer(CMD_YawZero, 5);
    delay_ms_g6(100);
    Gyro6_UART_SendBuffer(CMD_Save, 5);
}

void Gyro6_SendBiasCal(void)
{
    Gyro6_UART_SendBuffer(CMD_Key, 5);
    delay_ms_g6(100);
    Gyro6_UART_SendBuffer(CMD_BiasCal, 5);
    delay_ms_g6(21000);   /* 等待21秒, 期间勿移动 */
    Gyro6_UART_SendBuffer(CMD_Save, 5);
}

/*==== 协议解析状态机 (11字节帧) ====*/
void Gyro6_ParseByte(unsigned char ucData)
{
    static unsigned char ucRxBuffer[11];
    static unsigned char ucRxCnt = 0;
    unsigned char sum = 0;

    ucRxBuffer[ucRxCnt++] = ucData;

    if (ucRxBuffer[0] != 0x5A) { ucRxCnt = 0; return; }
    if (ucRxCnt < 11) return;

    switch (ucRxBuffer[1]) {
    case 0xAA: /* 角速度 */
        sum = ucRxBuffer[0]+ucRxBuffer[1]
            + ucRxBuffer[2]+ucRxBuffer[3]
            + ucRxBuffer[4]+ucRxBuffer[5]
            + ucRxBuffer[6]+ucRxBuffer[7]
            + ucRxBuffer[8]+ucRxBuffer[9];
        if (sum != ucRxBuffer[10]) { ucRxCnt = 0; return; }
        {
            short wx = (short)((ucRxBuffer[3] << 8) | ucRxBuffer[2]);
            short wy = (short)((ucRxBuffer[5] << 8) | ucRxBuffer[4]);
            short wz = (short)((ucRxBuffer[7] << 8) | ucRxBuffer[6]);
            stcGyro6.rawWx = wx; stcGyro6.rawWy = wy; stcGyro6.rawWz = wz;
            stcGyro6.wx = (float)wx / 32768.0f * 2000.0f;
            stcGyro6.wy = (float)wy / 32768.0f * 2000.0f;
            stcGyro6.wz = (float)wz / 32768.0f * 2000.0f;
        }
        break;
    case 0xBB: /* 角度 */
        sum = ucRxBuffer[0]+ucRxBuffer[1]
            + ucRxBuffer[2]+ucRxBuffer[3]
            + ucRxBuffer[4]+ucRxBuffer[5]
            + ucRxBuffer[6]+ucRxBuffer[7]
            + ucRxBuffer[8]+ucRxBuffer[9];
        if (sum != ucRxBuffer[10]) { ucRxCnt = 0; return; }
        {
            short roll  = (short)((ucRxBuffer[3] << 8) | ucRxBuffer[2]);
            short pitch = (short)((ucRxBuffer[5] << 8) | ucRxBuffer[4]);
            short yaw   = (short)((ucRxBuffer[7] << 8) | ucRxBuffer[6]);
            stcAngle6.Roll  = (float)roll  / 32768.0f * 180.0f;
            stcAngle6.Pitch = (float)pitch / 32768.0f * 180.0f;
            stcAngle6.Yaw   = (float)yaw   / 32768.0f * 180.0f;
        }
        break;
    case 0xCC: /* 加速度 */
        sum = ucRxBuffer[0]+ucRxBuffer[1]
            + ucRxBuffer[2]+ucRxBuffer[3]
            + ucRxBuffer[4]+ucRxBuffer[5]
            + ucRxBuffer[6]+ucRxBuffer[7]
            + ucRxBuffer[8]+ucRxBuffer[9];
        if (sum != ucRxBuffer[10]) { ucRxCnt = 0; return; }
        {
            short ax = (short)((ucRxBuffer[3] << 8) | ucRxBuffer[2]);
            short ay = (short)((ucRxBuffer[5] << 8) | ucRxBuffer[4]);
            short az = (short)((ucRxBuffer[7] << 8) | ucRxBuffer[6]);
            const float G = 9.8f;
            stcAccel6.rawAx = ax; stcAccel6.rawAy = ay; stcAccel6.rawAz = az;
            stcAccel6.ax = (float)ax / 32768.0f * 16.0f * G;
            stcAccel6.ay = (float)ay / 32768.0f * 16.0f * G;
            stcAccel6.az = (float)az / 32768.0f * 16.0f * G;
        }
        break;
    case 0xDD: /* 四元数 */
        sum = ucRxBuffer[0]+ucRxBuffer[1]
            + ucRxBuffer[2]+ucRxBuffer[3]
            + ucRxBuffer[4]+ucRxBuffer[5]
            + ucRxBuffer[6]+ucRxBuffer[7]
            + ucRxBuffer[8]+ucRxBuffer[9];
        if (sum != ucRxBuffer[10]) { ucRxCnt = 0; return; }
        {
            short q0 = (short)((ucRxBuffer[3] << 8) | ucRxBuffer[2]);
            short q1 = (short)((ucRxBuffer[5] << 8) | ucRxBuffer[4]);
            short q2 = (short)((ucRxBuffer[7] << 8) | ucRxBuffer[6]);
            short q3 = (short)((ucRxBuffer[9] << 8) | ucRxBuffer[8]);
            stcQuat6.q0 = (float)q0 / 32768.0f;
            stcQuat6.q1 = (float)q1 / 32768.0f;
            stcQuat6.q2 = (float)q2 / 32768.0f;
            stcQuat6.q3 = (float)q3 / 32768.0f;
        }
        break;
    default:
        ucRxCnt = 0;
        return;
    }
    ucRxCnt = 0;
}

/*==== 初始化 ====*/
void Gyro6_Init(void)
{
    memset(&stcAngle6, 0, sizeof(stcAngle6));
    memset(&stcGyro6,  0, sizeof(stcGyro6));
    memset(&stcAccel6, 0, sizeof(stcAccel6));
    memset(&stcQuat6,  0, sizeof(stcQuat6));

    /* 与单轴 Gyro_Init() 对齐: 补做 UART1 寄存器收尾配置。 */
    DL_UART_Main_disableLoopbackMode(UART_1_INST);
    DL_UART_Main_clearInterruptStatus(UART_1_INST,
        DL_UART_INTERRUPT_RX | DL_UART_INTERRUPT_OVERRUN_ERROR);
    DL_UART_Main_enableInterrupt(UART_1_INST,
        DL_UART_INTERRUPT_RX | DL_UART_INTERRUPT_OVERRUN_ERROR);

    /* 关键: RX FIFO threshold 设为 1 字节就触发中断。
     * MSPM0G UART 默认 threshold = 1/2 满 (FIFO 4 字节深, 要攒 2 字节才置 RIS.RXINT),
     * 陀螺仪 11 字节帧 + 115200 波特率下, FIFO 经常不足 2 字节, 导致 RX 中断永远不触发。
     * 设成 ONE_ENTRY 后, 每个字节进 FIFO 就产生 RX 中断。 */
    DL_UART_Main_setRXFIFOThreshold(UART_1_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);

    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);

    delay_ms_g6(100);
    Gyro6_SendCalibrate();
}

/*==== 数据访问函数 ====*/
float Gyro6_X(void)  { return stcGyro6.wx; }
float Gyro6_Y(void)  { return stcGyro6.wy; }
float Gyro6_Z(void)  { return stcGyro6.wz; }
float Gyro6_Roll(void)  { return stcAngle6.Roll; }
float Gyro6_Pitch(void) { return stcAngle6.Pitch; }
float Gyro6_Yaw(void)   { return stcAngle6.Yaw; }
float Gyro6_AccelX(void) { return stcAccel6.ax; }
float Gyro6_AccelY(void) { return stcAccel6.ay; }
float Gyro6_AccelZ(void) { return stcAccel6.az; }
float Gyro6_Q0(void) { return stcQuat6.q0; }
float Gyro6_Q1(void) { return stcQuat6.q1; }
float Gyro6_Q2(void) { return stcQuat6.q2; }
float Gyro6_Q3(void) { return stcQuat6.q3; }
