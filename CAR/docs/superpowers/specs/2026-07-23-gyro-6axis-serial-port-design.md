# 六轴串口陀螺仪驱动移植设计

- 日期: 2026-07-23
- 工程目标: `c:\Users\28442\Desktop\fengzhuang\25diansai1` (MSPM0G3507 Keil 封装库)
- 移植来源: `C:\Users\28442\Desktop\六轴陀螺仪资料\六轴陀螺仪资料\GyroPID -AXIS6\GyroPID -AXIS6\Public\Board\board.c`

## 1. 背景与目标

封装工程 `25diansai1` 已有一套**单轴**串口陀螺仪驱动 `Hardware/gyro_serial.c/.h`（5 字节帧，0x5A 协议，只解析 Z 轴角度+角速度，UART1 PB4/PB5）。

用户另有一只**六轴**陀螺仪模块（输出三轴角度/角速度/加速度/四元数，11 字节帧），需要把资料里 `GyroPID -AXIS6/board.c` 的六轴解析逻辑移植进来，与单轴驱动并存。

**目标**：在 `Hardware/` 下新增六轴驱动 `gyro_6axis.c/.h`，与单轴 `gyro_serial.c/.h` 共存，共用 UART1，靠编译期宏 `USE_GYRO_6AXIS` 二选一。单轴驱动保持不动。

## 2. 约束与前提

- 目标 MCU: MSPM0G3507，Keil 工程，DriverLib。
- 单轴与六轴是两只不同协议的陀螺仪，同一时刻只启用一套（宏切换 + 换硬件）。
- 两者波特率一致，故 syscfg 的 UART1 配置不需改动，纯软件切换。
- 封装工程 syscfg 未给 UART1 配 RX 中断，且开了 loopback；`Gyro6_Init()` 必须复用单轴 `Gyro_Init()` 中"关 loopback + 使能 RX 中断 + NVIC"的逻辑。
- PDF 数据手册因 MinerU 超时未能提取；以配套参考代码 `board.c` 的 11 字节协议为权威来源（与实物硬件配套）。

## 3. 协议规范（11 字节帧）

```
帧格式: 0x5A  TYPE  D0L D0H  D1L D1H  D2L D2H  D3L D3H  SUM   (11 字节)
TYPE:   0xAA=角速度  0xBB=角度  0xCC=加速度  0xDD=四元数  0xEE=寄存器读报(可忽略)
SUM:    前 10 字节累加和的低字节
```

数据解析（每组 `(H<<8)|L` 为 `short`）：

| TYPE | 数据 | 换算 |
|---|---|---|
| 0xAA | Wx,Wy,Wz | raw/32768*2000 deg/s |
| 0xBB | Roll,Pitch,Yaw | raw/32768*180 deg |
| 0xCC | Ax,Ay,Az | raw/32768*16*9.8 m/s² |
| 0xDD | q0,q1,q2,q3 | raw/32768 (归一化) |

校准命令（5 字节，**注意 Z 轴归零与单轴不同**）：

| 命令 | 字节 |
|---|---|
| 解锁寄存器 | `55 AA 13 8E 5F` |
| Z 轴归零 | `55 AA 0A 04 00` |
| 保存配置 | `55 AA 00 00 00` |
| 零偏校准 | `55 AA 0A 01 00` |

## 4. 架构与文件布局

```
Hardware/
  gyro_serial.c/.h   ← 现有单轴（5 字节帧），保持不动
  gyro_6axis.c/.h     ← 新增六轴（11 字节帧），移植自 GyroPID/board.c
```

- 类型名/函数名一律加 `6` 后缀（`SAngle6`、`Gyro6_Yaw()` 等），与单轴完全隔离，两套可同处一个编译单元不重名。
- 宏 `USE_GYRO_6AXIS` 定义在 `gyro_6axis.h` 顶部（默认注释掉，启用时取消注释）。

## 5. 数据结构与 API（gyro_6axis.h）

```c
typedef struct { float Roll, Pitch, Yaw; } SAngle6;
typedef struct { float wx, wy, wz; short rawWx, rawWy, rawWz; } SGyro6;
typedef struct { float ax, ay, az; short rawAx, rawAy, rawAz; } SAccel6;
typedef struct { float q0, q1, q2, q3; } SQuat6;

extern SAngle6 stcAngle6;
extern SGyro6  stcGyro6;
extern SAccel6 stcAccel6;
extern SQuat6  stcQuat6;

void  Gyro6_Init(void);                  /* 初始化串口 + 发 Z 轴归零 */
void  Gyro6_ParseByte(unsigned char b);  /* 11 字节帧状态机 (UART ISR 调用) */
void  Gyro6_SendCalibrate(void);         /* Z 轴归零 */
void  Gyro6_SendBiasCal(void);           /* 零偏校准 (需静止 21 秒) */

float Gyro6_X(void); float Gyro6_Y(void); float Gyro6_Z(void);
float Gyro6_Roll(void); float Gyro6_Pitch(void); float Gyro6_Yaw(void);
float Gyro6_AccelX(void); float Gyro6_AccelY(void); float Gyro6_AccelZ(void);
float Gyro6_Q0(void); float Gyro6_Q1(void); float Gyro6_Q2(void); float Gyro6_Q3(void);
```

## 6. gyro_6axis.c 实现要点

- **状态机**：移植 `board.c` 的 `CopeSerial2Data`。11 字节缓冲，帧头校验 `0x5A`，收满 11 字节后按 TYPE 分支计算校验和，失败复位，成功解析对应数据后复位。
- **校准命令发送**：复用单轴的 `DL_Common_delayCycles` 延时 + `DL_UART_Main_transmitDataBlocking(UART_1_INST, ...)` 发送方式。命令字节用六轴版（Z 轴归零 `0A 04`）。
- **Init**：清零数据结构 → `DL_UART_Main_disableLoopbackMode(UART_1_INST)` → 清并使能 RX + OVERRUN 中断 → 清并使能 NVIC → 延时 100ms 稳定 → `Gyro6_SendCalibrate()`。
- **数据访问函数**：返回对应全局结构体字段。

## 7. main.c 改动（按宏分流）

ISR 与 Init 按宏二选一：

```c
void UART_1_INST_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(UART_1_INST) == DL_UART_IIDX_RX) {
        unsigned char byte = (unsigned char)DL_UART_receiveData(UART_1_INST);
    #if defined(USE_GYRO_6AXIS)
        Gyro6_ParseByte(byte);
    #else
        Gyro_ParseByte(byte);
    #endif
    }
    if (DL_UART_getPendingInterrupt(UART_1_INST) == DL_UART_OVERRUN_ERROR) {
        DL_UART_receiveData(UART_1_INST);
    }
}
```

Init 处：
```c
#if defined(USE_GYRO_6AXIS)
    Gyro6_Init();
#else
    Gyro_Init();
#endif
```

`#include` 区补 `#include "gyro_6axis.h"`，并在 main.c 顶部注释块补一行说明该新模块。

## 8. 文件集成与编译

- 新增 `Hardware/gyro_6axis.c`、`Hardware/gyro_6axis.h`
- 把 `gyro_6axis.c` 加入 Keil 工程的 Hardware 文件组（编辑 `.uvprojx`）
- syscfg 不动（UART1 已配，波特率一致）
- 单轴 `gyro_serial.c` 保留不动；用六轴时它仍参与编译但不被调用（无副作用）

## 9. 验证方式

封装工程无自动化测试，手动验证：

- **默认（不定义宏）**：单轴 `Gyro_ParseByte` 照常工作，确认未破坏现有功能。
- **定义 `USE_GYRO_6AXIS` + 接六轴陀螺仪**：OLED 或 printf 打印 `Gyro6_Yaw()/Gyro6_Z()`，旋转看数值随动；`Gyro6_SendCalibrate()` 后看 Yaw 归零。
- 校验和错误帧自动丢弃（状态机复位），观察无数据卡死。

## 10. 移植清单

| 项 | 动作 |
|---|---|
| `gyro_6axis.h` | 新建：数据结构 + API 声明 + `USE_GYRO_6AXIS` 宏 |
| `gyro_6axis.c` | 新建：11 字节解析状态机 + 4 条校准命令 + Init |
| `main.c` | ISR 与 Init 按宏分流；补 include 和注释 |
| `.uvprojx` | gyro_6axis.c 加入 Hardware 组 |
| syscfg | 不动 |
| 单轴 `gyro_serial.c/.h` | 不动，保留 |
