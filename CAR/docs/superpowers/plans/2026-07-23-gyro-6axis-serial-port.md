# 六轴串口陀螺仪驱动移植 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在封装工程 `25diansai1` 里新增六轴串口陀螺仪驱动 `gyro_6axis.c/.h`（11 字节 0x5A 协议），与现有单轴 `gyro_serial.c` 共存，共用 UART1，靠编译期宏 `USE_GYRO_6AXIS` 二选一。

**Architecture:** 新增独立 .c/.h，类型/函数名一律带 `6` 后缀避免与单轴冲突；解析逻辑移植自资料 `GyroPID -AXIS6/board.c` 的 `CopeSerial2Data`；`main.c` 的 UART1 ISR 与初始化按 `USE_GYRO_6AXIS` 宏分流；syscfg 不动。

**Tech Stack:** MSPM0G3507, TI DriverLib (DL_UART_Main_*), Keil MDK, C99。

## Global Constraints

- 目标 MCU: MSPM0G3507，Keil 工程，DriverLib。
- UART1 引脚: PB4(TX)/PB5(RX)（syscfg 已配，不动）。
- 单轴与六轴是两只不同协议的陀螺仪，同一时刻只启用一套。
- 两者波特率一致，syscfg 不改。
- 类型名/函数名一律加 `6` 后缀，与单轴 `gyro_serial.c/.h` 完全不重名。
- 新 `.c` 必须加入 Keil 工程文件组 `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx` 才会编译。
- 封装工程无自动化测试框架，验证靠编译通过 + 烧录后手动观察。

参考来源（只读）:
- 协议解析: `C:\Users\28442\Desktop\六轴陀螺仪资料\六轴陀螺仪资料\GyroPID -AXIS6\GyroPID -AXIS6\Public\Board\board.c` 的 `CopeSerial2Data`
- UART 关 loopback + 使能 RX 中断范式: `25diansai1/Hardware/gyro_serial.c` 的 `Gyro_Init`

---

### Task 1: 创建 gyro_6axis.h 头文件

**Files:**
- Create: `Hardware/gyro_6axis.h`

**Interfaces:**
- Produces: `USE_GYRO_6AXIS` 宏开关；数据结构 `SAngle6/SGyro6/SAccel6/SQuat6`；全局变量 `stcAngle6/stcGyro6/stcAccel6/stcQuat6`；API `Gyro6_Init/Gyro6_ParseByte/Gyro6_SendCalibrate/Gyro6_SendBiasCal` 及 13 个数据访问函数声明。

- [ ] **Step 1: 写头文件**

```c
/**
 * @file  gyro_6axis.h
 * @brief 六轴串口陀螺仪驱动 (0x5A 协议, 11字节帧, UART1 PB4/PB5)
 *
 * 协议帧: 0x5A TYPE D0L D0H D1L D1H D2L D2H D3L D3H SUM (11字节)
 *   TYPE: 0xAA=角速度 0xBB=角度 0xCC=加速度 0xDD=四元数
 *   SUM : 前10字节累加和低字节
 *   换算: 角速度 raw/32768*2000 deg/s, 角度 raw/32768*180 deg,
 *         加速度 raw/32768*16*9.8 m/s2, 四元数 raw/32768
 *
 * 与单轴 gyro_serial.c 共存, 共用 UART1, 靠本宏二选一:
 *   启用六轴时取消下一行注释, 并确保 main.c ISR 走 Gyro6_ParseByte 分支.
 */
#ifndef __GYRO_6AXIS_H__
#define __GYRO_6AXIS_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

/*==== 宏开关: 启用六轴驱动, 注释掉则用单轴 gyro_serial ====*/
#define USE_GYRO_6AXIS

/*==== 数据结构 ====*/
typedef struct {
    float Roll;   /* 横滚角 deg  -180~+180 */
    float Pitch;  /* 俯仰角 deg  -180~+180 */
    float Yaw;    /* 航向角 deg  -180~+180 */
} SAngle6;

typedef struct {
    float wx;     /* X轴角速度 deg/s  +-2000 */
    float wy;     /* Y轴角速度 deg/s */
    float wz;     /* Z轴角速度 deg/s */
    short rawWx;
    short rawWy;
    short rawWz;
} SGyro6;

typedef struct {
    float ax;     /* X轴加速度 m/s2  +-16g */
    float ay;
    float az;
    short rawAx;
    short rawAy;
    short rawAz;
} SAccel6;

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} SQuat6;

/*==== 全局数据 ====*/
extern SAngle6 stcAngle6;
extern SGyro6  stcGyro6;
extern SAccel6 stcAccel6;
extern SQuat6  stcQuat6;

/*==== API ====*/
void  Gyro6_Init(void);                  /* 初始化串口 + 发Z轴归零 */
void  Gyro6_ParseByte(unsigned char b);  /* 11字节帧状态机, UART ISR 调用 */
void  Gyro6_SendCalibrate(void);         /* Z轴归零 */
void  Gyro6_SendBiasCal(void);           /* 零偏校准 (需静止21秒) */

float Gyro6_X(void);  float Gyro6_Y(void);  float Gyro6_Z(void);
float Gyro6_Roll(void); float Gyro6_Pitch(void); float Gyro6_Yaw(void);
float Gyro6_AccelX(void); float Gyro6_AccelY(void); float Gyro6_AccelZ(void);
float Gyro6_Q0(void); float Gyro6_Q1(void); float Gyro6_Q2(void); float Gyro6_Q3(void);

#endif /* __GYRO_6AXIS_H__ */
```

- [ ] **Step 2: 编译验证头文件可被找到（暂无 .c, 仅确认路径与 include 不报错）**

无需单独编译，Task 2 一起验证。

---

### Task 2: 创建 gyro_6axis.c 实现

**Files:**
- Create: `Hardware/gyro_6axis.c`

**Interfaces:**
- Consumes: Task 1 的所有声明；DriverLib `DL_UART_Main_*`、`DL_Common_delayCycles`、`CPUCLK_FREQ`；syscfg 生成的 `UART_1_INST`、`UART_1_INST_INT_IRQN`、`DL_UART_INTERRUPT_RX`、`DL_UART_INTERRUPT_OVERRUN_ERROR`。
- Produces: Task 1 声明的全部函数定义。

- [ ] **Step 1: 写实现文件**

```c
/**
 * @file  gyro_6axis.c
 * @brief 六轴串口陀螺仪驱动实现 (0x5A 协议, 11字节帧, UART1 PB4=TX/PB5=RX)
 *
 * 移植自资料 GyroPID -AXIS6/board.c 的 CopeSerial2Data.
 * syscfg 未给 UART1 配 RX 中断且开了 loopback, Init 里手动关 loopback + 使能 RX.
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

    /* 关 loopback, 使 PB5(RX) 真正接陀螺仪 TX */
    DL_UART_Main_disableLoopbackMode(UART_1_INST);

    /* 使能 RX + OVERRUN 中断 (syscfg 未配, 这里补) */
    DL_UART_Main_clearInterruptStatus(UART_1_INST,
        DL_UART_INTERRUPT_RX | DL_UART_INTERRUPT_OVERRUN_ERROR);
    DL_UART_Main_enableInterrupt(UART_1_INST,
        DL_UART_INTERRUPT_RX | DL_UART_INTERRUPT_OVERRUN_ERROR);

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
```

- [ ] **Step 2: 确认 gyro_serial.c 里使用的符号在六轴版也合法**

对照单轴 `gyro_serial.c`：`CPUCLK_FREQ`、`DL_Common_delayCycles`、`DL_UART_Main_transmitDataBlocking`、`UART_1_INST`、`UART_1_INST_INT_IRQN`、`DL_UART_INTERRUPT_RX`、`DL_UART_INTERRUPT_OVERRUN_ERROR`、`DL_UART_Main_disableLoopbackMode` 均为同工程已用符号，六轴版沿用即可。无需改动。

---

### Task 3: 把 gyro_6axis.c 加入 Keil 工程

**Files:**
- Modify: `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`（在 gyro_serial.c 注册块后追加 gyro_6axis.c 注册块）

**Interfaces:**
- 无代码接口，仅工程文件配置。

- [ ] **Step 1: 在 .uvprojx 的 gyro_serial.c 块后追加 gyro_6axis.c 块**

定位 `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx` 中：
```xml
            <File>
              <FileName>gyro_serial.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\Hardware\gyro_serial.c</FilePath>
            </File>
```
在其后追加：
```xml
            <File>
              <FileName>gyro_6axis.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\Hardware\gyro_6axis.c</FilePath>
            </File>
```

- [ ] **Step 2: 编译验证 gyro_6axis.c 被纳入编译**

在 Keil 中 Rebuild。预期：`gyro_6axis.c` 出现在编译文件列表，无 "file not found" 错误。此时 main.c 尚未引用它，但只要文件被编译且语法正确即通过。（若 Keil 报头文件找不到，确认 include 路径含 `..\Hardware`，这是工程已有配置。）

---

### Task 4: main.c 按宏分流 ISR 与初始化

**Files:**
- Modify: `25diansai1/main.c`（顶部 include 区、注释块、main() 的 IMU 初始化处、`UART_1_INST_IRQHandler`）

**Interfaces:**
- Consumes: Task 1 的 `USE_GYRO_6AXIS`、`Gyro6_Init`、`Gyro6_ParseByte`；现有单轴 `Gyro_Init`、`Gyro_ParseByte`。

- [ ] **Step 1: include 区加入 gyro_6axis.h**

在 `main.c` 现有 `#include "gyro_serial.h"` 下方加一行：
```c
#include "gyro_serial.h"
#include "gyro_6axis.h"   /* 六轴驱动; 启用时在 gyro_6axis.h 取消 USE_GYRO_6AXIS 注释, 见下方说明 */
```

- [ ] **Step 2: main() 的 IMU 初始化处按宏分流**

定位 `main.c` 中：
```c
    /* IMU 串口初始化 (0x5A 协议, UART1 PB4/PB5) */
    Gyro_Init();
```
替换为：
```c
    /* IMU 串口初始化 (0x5A 协议, UART1 PB4/PB5)
     * USE_GYRO_6AXIS 在 gyro_6axis.h 定义: 启用六轴时走 Gyro6_Init,
     * 否则走单轴 Gyro_Init. 切换宏后需同时换接的陀螺仪硬件. */
#if defined(USE_GYRO_6AXIS)
    Gyro6_Init();
#else
    Gyro_Init();
#endif
```

- [ ] **Step 3: UART1 ISR 按宏分流**

定位 `main.c` 中：
```c
void UART_1_INST_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(UART_1_INST) == DL_UART_IIDX_RX) {
        unsigned char byte = (unsigned char)DL_UART_receiveData(UART_1_INST);
        Gyro_ParseByte(byte);
    }
    if (DL_UART_getPendingInterrupt(UART_1_INST) == DL_UART_IIDX_OVERRUN_ERROR) {
        DL_UART_receiveData(UART_1_INST);  /* 清 OVERRUN 错误 */
    }
}
```
替换 `Gyro_ParseByte(byte);` 那一行为：
```c
#if defined(USE_GYRO_6AXIS)
        Gyro6_ParseByte(byte);
#else
        Gyro_ParseByte(byte);
#endif
```

- [ ] **Step 4: main.c 顶部注释块补一行说明新模块**

在 `main.c` 顶部注释里 `底层驱动:` 行所在段落补充六轴驱动说明，保持现有风格。将：
```
 * 底层驱动: motor/encoder/track/pid/oled(I2C)/gyro_serial/key/bsp
```
改为：
```
 * 底层驱动: motor/encoder/track/pid/oled(I2C)/gyro_serial(单轴)/gyro_6axis(六轴)/key/bsp
 *   注: gyro_serial(单轴5字节帧) 与 gyro_6axis(六轴11字节帧) 共用 UART1,
 *       靠 gyro_6axis.h 的 USE_GYRO_6AXIS 宏二选一, 同时只启用一套并换接对应陀螺仪.
```

- [ ] **Step 5: 编译验证两种配置都通过**

(a) 默认配置（`gyro_6axis.h` 里 `#define USE_GYRO_6AXIS` 启用六轴）：Keil Rebuild，预期 0 error。
(b) 临时把 `gyro_6axis.h` 里 `#define USE_GYRO_6AXIS` 改为 `//#define USE_GYRO_6AXIS`（切回单轴）：Rebuild，预期 0 error。验证后恢复为启用六轴（取消注释）。

两种配置均编译通过即本任务完成。

---

### Task 5: 烧录验证（手动）

**Files:**
- 无文件改动，硬件验证。

**Interfaces:**
- Consumes: Task 1–4 全部成果。

- [ ] **Step 1: 确认六轴陀螺仪硬件接线**

确认六轴陀螺仪模块的 TX 接 MSPM0G3507 的 PB5（UART1 RX），RX 接 PB4（UART1 TX），共地，供电匹配。波特率与单轴一致（syscfg 已配）。

- [ ] **Step 2: 烧录并在 main 循环打印六轴数据**

`gyro_6axis.h` 保持 `USE_GYRO_6AXIS` 启用。在 `main.c` 的 `while(1)` 内临时加入：
```c
        printf("Yaw=%.2f GyroZ=%.2f\r\n", Gyro6_Yaw(), Gyro6_Z());
        delay_ms(100);
```
（`delay_ms` 由 board.c 提供，printf 经 UART0 调试口。）
烧录运行，旋转陀螺仪，预期串口助手看到 Yaw 与 GyroZ 随动变化。

- [ ] **Step 3: 验证 Z 轴归零**

`Gyro6_Init()` 已自动发 Z 轴归零。上电后观察 Yaw 初始应为 0 附近；手动旋转再回到原位，Yaw 应能回到归零点附近。

- [ ] **Step 4: 验证完成后移除临时打印**

将 `while(1)` 内的临时 printf 恢复为空（或保留注释占位），保持 main.c 干净。提交。

```bash
cd "c:/Users/28442/Desktop/fengzhuang/25diansai1"
git add Hardware/gyro_6axis.c Hardware/gyro_6axis.h main.c keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx
git commit -m "feat: 移植六轴串口陀螺仪驱动 gyro_6axis (11字节0x5A协议, 与单轴共用UART1宏切换)"
```

---

## Self-Review

**1. Spec coverage:**
- 新增 gyro_6axis.c/.h → Task 1, 2 ✓
- 11字节解析状态机 → Task 2 ✓
- 4条校准命令（含Z轴归零0A 04）→ Task 2 ✓
- Init 关loopback+使能RX+NVIC → Task 2 ✓
- main.c ISR与Init按宏分流 → Task 4 ✓
- .uvprojx 加文件 → Task 3 ✓
- syscfg 不动 → 未列为任务（正确，无需动作）✓
- 单轴 gyro_serial.c 不动 → 未列为任务（正确）✓
- 验证 → Task 5 ✓
- 类型/函数名带6后缀 → 全文一致 ✓

**2. Placeholder scan:** 无 TBD/TODO/省略，所有代码块完整。✓

**3. Type consistency:** `SAngle6/SGyro6/SAccel6/SQuat6`、`stcAngle6` 等、`Gyro6_*` 函数名在 Task 1（声明）与 Task 2（定义）、Task 4（调用）三处完全一致。`USE_GYRO_6AXIS` 宏名三处一致。✓
