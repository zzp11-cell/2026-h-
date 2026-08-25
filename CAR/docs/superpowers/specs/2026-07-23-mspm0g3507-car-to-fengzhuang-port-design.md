# mspm0g3507_car → fengzhuang 软硬件封装补全移植设计

日期: 2026-07-23
源项目: `C:\Users\28442\Desktop\mspm0g3507_car\mspm0g3507_car` (CCS/Keil 混合, MSPM0G3507 小车全套例程)
目标项目: `C:\Users\28442\Desktop\fengzhuang\25diansai1` (Keil, MSPM0G3507 自建封装库)

## 1. 范围

用户选择「补全缺失模块」: 把源项目里 fengzhuang 还没有的软硬件封装模块移植过来, **保留 fengzhuang 现有引脚映射不变**, 不替换已有模块。

引脚策略: 「逐个问我接线」—— 每个引脚相关模块移植前先问用户实际接线, 代码不写死示例引脚。
Keil 集成策略: 「只复制不集成」—— 移植后只把 .c/.h 放进 fengzhuang 目录, 不改 .uvprojx, 用户自己加文件组。

## 2. 模块分类

### 2.1 不移植 (fengzhuang 已有等价实现, 移植会冲突)
| 源模块 | fengzhuang 等价 | 原因 |
|--------|----------------|------|
| WT_IMU63 | gyro_serial (0x5A 协议, UART1) | 协议和串口实例都不同 |
| UART0 | bsp_printf | fengzhuang 已重定向 printf |
| Motor/Encoder/PID/Track/KEY/OLED | 同名自有实现 | 引脚映射和 API 不同 |

### 2.2 直接可移植 (不依赖引脚的算法/工具)
| 模块 | 适配工作 |
|------|---------|
| Kalman.c/.h | 几乎零改动: 去掉 `#include "main.h"`, 改 `#include <math.h>` 保留。Track_Kalman/Gyro_Kalman 全局量保留 |
| Change_Type.c/.h | 去掉 `#include "main.h"`, 保留 stdlib/string.h。String_to_Int 零逻辑改动 |

### 2.3 需引脚适配的硬件模块 (逐个问接线)
| 模块 | 源依赖 | 适配点 |
|------|--------|--------|
| Buzzer | `Buzzer_PORT/Buzzer_PIN_PIN` (syscfg 宏) | 用 fengzhuang 风格: 用户给 GPIO 引脚后在 .h 里 `#define BUZZER_PORT`/`#define BUZZER_PIN` |
| LED | `GPIO_LED_PORT/GPIO_LED_Light1/2_PIN` | 同上, 两个 LED 引脚 |
| Step_Motor | `Step_Motor_IN1~4_PORT/PIN` + `Step_Motor_BIN1~4_PORT/PIN` (双电机) + `mspm0_delay_ms()` | 8 个 GPIO 引脚, 延时改 `delay_ms()` (fengzhuang board.c 提供) |
| Servo | `PWM_1_INST/PWM_2_INST` + `GPIO_PWM_1/2_C0_IDX` + `DL_TimerG_setCaptureCompareValue` | **TIMA0 已被 fengzhuang 电机 PWM 占用**, 必须换定时器(候选 TIMG7/TIMG8)和新引脚。API 修正: `DL_TimerG_setCaptureCompareValue` 不存在, 改 `DL_Timer_setCaptureCompareValue` |
| K230 | `UART_3_INST` + `K230_ReceiveData` + `Yaw_arr[Yaw_real]` + `Set_Speed` | UART 实例改 fengzhuang 实际串口(默认 UART3 PB2/PB3, 问用户)。调用的 `Yaw_arr[Yaw_real]`→`Yaw()`(gyro_serial.h), `Set_Speed(l,r)`→`Set_PWM(l,r)` 或 `Car_Move(pl,pr)`(motor.h)。K230_Straght_Line_PID 里写死 PID 参数, 保留但加注释 |
| UART3 | 仅一个 `UART3_IRQHandler` 调 `K230_ReceiveData` | 不单独移植文件, K230 的 ISR 直接写在 main.c 或合并进 K230.c |

## 3. 引脚铁律 (遵循 mspm0g-contest 技能)

每次分配引脚必须排除禁用引脚:
- PA10/PA11 — UART0(VOFA)/I2C1(MPU6050) 冲突
- PA2~PA6 — 时钟引脚, 未焊接, 绝对禁用
- PA19/PA20 — SWD 调试, 保留
- fengzhuang 已占用: TIMA0/PA8/PA9 (电机PWM), TIMG6 (PID 10ms), TIMG12 (长计时), I2C0/PA28/PA31 (OLED), UART0/PA10/PA11 (printf), UART1/PB4/PB5 (IMU, 实测 gyro_serial.h 写 PB6/PB7 待核)

每次给硬件引脚必须输出完整表格:
| 外设功能 | 芯片/模块型号 | 天猛星引脚 | IOMUX索引 | 片上复用功能 | 备注 |

## 4. 目录结构

```
25diansai1/
├── Hardware/          (已有, 新增硬件模块放这里)
│   ├── buzzer.c/.h     ← 移植
│   ├── led.c/.h        ← 移植
│   ├── step_motor.c/.h ← 移植
│   ├── servo.c/.h      ← 移植
│   └── k230.c/.h       ← 移植 (含原 UART3 的 ISR 逻辑)
└── Software/          (新建目录)
    ├── kalman.c/.h     ← 移植
    └── change_type.c/.h ← 移植
```
注: `Interrupt_Gather.c` 暂不移植 —— 它定义 TIMG0/TIMG7/TIMG12 ISR, 会和 fengzhuang 的 TIMER_0(TIMG6) ISR 冲突, 且调用的 LED1_ON/Key_GetNum/OLED_ShowSignedNum/Yaw_arr API 全是源项目命名, fengzhuang 对不上。用户后续如需可按 fengzhuang API 重写。

## 5. 命名与风格

- fengzhuang 现有模块用小写文件名 (motor.c, encoder.c), 但模块内函数大小写混用 (Set_PWM, Get_Encoder_countA)。移植模块保留源项目原文件名小写化 (Buzzer.c→buzzer.c), 函数名保留源项目原名 (Buzzer_on, LED1_ON, Servo1_PWM_Set_Angle, Step_Motor_Move, K230_ReceiveData) 以便用户对照源例程, 不强行统一。
- 每个移植文件头部加注释: 来源、移植日期、适配点说明、引脚待确认标记。
- 所有引脚宏在 .h 顶部集中定义, 标 `/* PAxx — 请替换为实际引脚 */`。

## 6. 实施顺序

1. 新建 `Software/` 目录, 移植 Kalman + Change_Type (零引脚, 直接可用)。
2. 移植硬件模块, 每个先问接线再写: Buzzer → LED → Step_Motor → Servo → K230。
3. 不动 fengzhuang 现有任何文件 (main.c 仅在用户要求时加 include 提示)。
4. 不改 .uvprojx (用户自己加文件组)。
5. 移植完成后在 main.c 注释区提示用户如何 include 新模块。

## 7. 成功标准

- 9 个模块 (5 硬件 + 2 软件 + K230 含 UART3 逻辑) 的 .c/.h 文件存在于 fengzhuang 对应目录。
- 所有引脚用 `/* 待确认 */` 占位, 不写死。
- 不破坏 fengzhuang 现有任何已编译模块。
- API 调用全部适配到 fengzhuang 已有 API (Yaw/Set_PWM/Car_Move/delay_ms), 不残留源项目独有符号 (Yaw_arr/Yaw_real/Set_Speed/mspm0_delay_ms)。
