# MSPM0G3507 Keil 封装库改造设计

**日期**: 2026-07-22
**目标工程**: `C:\Users\28442\Desktop\fengzhuang\25diansai1` (已有 Keil .uvprojx)
**实物板**: 用户扩展板 (原理图 `SCH_Schematic1_1-P1_2026-07-22.svg`)

## 1. 目标

把 fengzhuang/25diansai1 从"圈数循迹任务工程"改造为**通用 Keil 封装库**:
- 保留全部底层驱动模块 + PID 封装
- 删除任务逻辑 (圈数循迹/状态机/过弯保护/变速等)
- 引脚按扩展板原理图重新适配 syscfg
- main.c 只留初始化框架 + while(1) 空循环, 用户自己填任务

## 2. 蓝本决策 (三方对比结论)

| 工程 | 模块化 | PID质量 | Keil | 结论 |
|------|--------|---------|------|------|
| fengzhuang(现有) | ✅清晰 | ✅三套PID+结构体+限幅 | ✅已有 | **主体保留** |
| car(开源) | ✅清晰 | ⚠️通用但不如fengzhuang | ❌CCS | 仅参考, 不拷 |
| 24H_4 | ❌motor.c 405行大杂烩 | ❌裸全局变量 | ❌CCS | 不用 |

**方案**: 以 fengzhuang 现有驱动为主, 不引入 car 代码 (因扩展板无舵机/步进/超声波接口, IMU 也用 fengzhuang 自己的 gyro_serial)。

## 3. 模块取舍清单

### 保留 (底层驱动, 全部保留)
| 文件 | 功能 | 备注 |
|------|------|------|
| board.c/h | SysTick_Init + printf 重定向框架 | 保留, 删任务常量引用 |
| bsp_printf.c/h | any_printf 任意串口打印 | 保留 |
| bsp_systick.c/h | delay_ms/us + Systick_getTick | 保留 |
| motor.c/h | Set_PWM + Car_Move + 速度环PI | 保留 |
| encoder.c/h | Get_Encoder_countA/B + RPM | 保留 |
| track.c/h | 8路循迹 Track_GetState | 保留 (但循迹 syscfg 引脚暂不配, 待实物确认) |
| pid.c/h | 位置式转向PID + tPid速度环 + 角度环PID | 保留 (核心) |
| oled.c/h + oledfont.h | **替换为 24H_4 硬件 I2C 版** | fengzhuang 原是 4线SPI, 实物板是 I2C (原理图 PA28/PA31), 从 24H_4 拷 I2C 驱动 (用 DL_I2C API, 地址 0x3C) |
| gyro_serial.c/h | IMU 0x5A 协议串口 | 保留, 改引脚 |
| key.c/h | Key() 按键扫描 | **整体重写**为纯按键返回值 (原 Key() 全是圈数/速度档/状态机任务逻辑, 强耦合 work.h, 且引脚写死 PA18/PB8 未用 syscfg 宏) |

### 删除 (任务层)
| 文件 | 原因 |
|------|------|
| work.c/h | 圈数循迹任务层 (状态机/变速/过弯保护) — 整个删除 |
| config.h | 任务常量 (CROSSINGS_TO_A 等) — 删除任务常量, 保留文件骨架或删除 |
| empty.c | 任务主循环 — 替换为 main.c 框架 |

### 重写
| 文件 | 改动 |
|------|------|
| empty.syscfg | 按扩展板原理图重写所有引脚 |
| empty.c → main.c | 只留 SYSCFG_DL_init() + 各模块 init + while(1) |

## 4. 引脚映射 (扩展板原理图为准)

### 4.1 原理图确认的引脚 (100% 可靠)

| 功能 | 原理图引脚 | syscfg 实例名 | 引脚名 | 状态 |
|------|-----------|--------------|--------|------|
| TB6612 AIN1 | PA8 | Motor (GPIO) | AIN1 | ✅ |
| TB6612 AIN2 | PA9 | Motor | AIN2 | ✅ |
| TB6612 BIN1 | PB2 | Motor | BIN1 | ✅ |
| TB6612 BIN2 | PB3 | Motor | BIN2 | ✅ |
| 电机 PWMA | PA12 | PWMAB (TIMG0) | CCP0 | ✅ |
| 电机 PWMB | PA13 | PWMAB (TIMG0) | CCP1 | ✅ |
| 左编码器 A相 | PA17 | ENCODERA (GPIO中断) | E1A | ✅ |
| 左编码器 B相 | PA24 | ENCODERA | E1B | ✅ |
| 右编码器 A相 | PA22 | ENCODERB (GPIO中断) | E2A | ✅ |
| 右编码器 B相 | 待确认 | ENCODERB | E2B | ⏸ 原理图未明确 |
| 调试串口 UART0 | PA10/PA11 | DEBUG (UART0) | TX/RX | ✅ |
| IMU UART1 | PB4/PB5 | IMU (UART1) | TX/RX | ✅ |
| OLED I2C | PA28/PA31 | `OLED` (I2C0 硬件实例) | SDA/SCL | ✅ |
| LED1 | PA7 | LED (GPIO) | led | ✅ |
| 拨码开关 SW3 | PA26 | KEY (GPIO) | K1 | ✅ |

### 4.2 用户确认的范围决策

- **循迹**: 先不配 syscfg 实例 (原理图通过排针引出, SVG 解析不出连线, 用户暂不使用)
- **按键×3 + 蜂鸣器**: 用户将看实物板后告知引脚, 当前 syscfg 先只配 SW3(PA26), 其余待补

### 4.3 syscfg 实例命名 (保持代码宏不变)

为让现有 motor.c/encoder.c/track.c 代码不改, syscfg 里 GPIO/PWM/UART 实例的 `$name` 必须和原 fengzhuang syscfg 一致:
- 电机方向 GPIO: `Motor` (引脚 AIN1/AIN2/BIN1/BIN2)
- 电机 PWM: `PWMAB` (TIMG0, CCP0/CCP1)
- 编码器: `ENCODERA` / `ENCODERB` (引脚 E1A/E1B/E2A/E2B)
- 循迹: `Track` (PIN_1..PIN_8)
- 按键: `KEY` (BLS/USER 或 K1/K2)
- LED: `LED` (led)
- 调试串口: `DEBUG` (UART0, PA10/PA11)
- IMU: `IMU` (UART1, PB4/PB5) — 原 fengzhuang 用 UART1 但引脚是 PB6/PB7, 改为 PB4/PB5
- OLED: **硬件 I2C0 实例** `OLED` (PA28/PA31, 0x3C, 400kHz), 驱动用 24H_4 的 oled.c (DL_I2C API)

## 5. syscfg 改造细节

### 5.1 需修改的实例 (引脚改为扩展板)

| 实例 | 原引脚(fengzhuang) | 新引脚(扩展板) |
|------|-------------------|---------------|
| Motor.AIN1 | PA14 | PA8 |
| Motor.AIN2 | PA13 | PA9 |
| Motor.BIN1 | PA16 | PB2 |
| Motor.BIN2 | PA17 | PB3 |
| ENCODERA.E1A | PA26 | PA17 |
| ENCODERA.E1B | (原值待查) | PA24 |
| ENCODERB.E2A | (原值待查) | PA22 |
| ENCODERB.E2B | (原值待查) | 待确认 |
| DEBUG TX/RX | PA10/PA11 | PA10/PA11 (不变) |
| IMU TX/RX | PB6/PB7 (UART1) | PB4/PB5 (UART1) |
| Track PIN_1..8 | PA27/26/25/24/14/15/16/17 | 待确认扩展板循迹接口 |
| LED | PB9 | PA7 |
| KEY | PA18/PB8 | PA26 (+待确认) |

### 5.2 需保留的实例名 (代码宏依赖)
所有 `$name` 保持不变, 只改 `pin.$assign` 和定时器实例。代码层零改动 (motor.c 用 `PWM_0_INST`, empty.c/main.c 用 `TIMER_0_INST`)。

### 5.3 定时器分工 (融合 24H_4 方案)

用户认可 24H_4 的定时器中断设置更合理, 新封装库采用其分工:

| syscfg 实例名 | 定时器 | 用途 | 参数 | 原理图引脚 | 代码宏 |
|--------------|--------|------|------|-----------|--------|
| `PWM_0` | **TIMG0** | 电机 PWM | EDGE_ALIGN_UP, timerCount=4000 (80MHz/4000=20kHz) | PA12(CCP0)/PA13(CCP1) | PWM_0_INST, GPIO_PWM_0_C0_IDX/C1_IDX |
| `TIMER_0` | **TIMA0** | PID 控制周期 | PERIODIC_UP, 10ms, LOAD 中断 | — (无引脚) | TIMER_0_INST, TIMER_0_INST_INT_IRQN, TIMER_0_INST_IRQHandler |
| `NTB` (可选) | **TIMG12** | 长计时基准 | PERIODIC_UP, MFCLK 时钟, 6000s 超长周期 | — | NTB_INST |

**改动点 vs fengzhuang 原配置**:
- `PWM_0`: 定时器 TIMA1→**TIMG0**, 引脚 PB2/PB3→**PA12/PA13**, timerCount 8000→4000
- `TIMER_0`: 定时器 TIMG0→**TIMA0** (因 TIMG0 让给 PWM), 中断事件 ZERO→LOAD, 周期保持 10ms
- 新增 `NTB` (TIMG12) 备用长计时 (24H_4 方案, 先配上不强制启用)

> ⚠️ `TIMER_0_INST_IRQHandler` 函数名不变 (由实例名 `TIMER_0` 决定), 但内部 clearInterruptStatus 要从 `DL_TIMERG_INTERRUPT_ZERO_EVENT` 改为 `DL_TIMERA_INTERRUPT_LOAD_EVENT` (TIMA0 用 LOAD 事件)。这是 main.c 里唯一需改的定时器相关代码。

> 可用定时器白名单: TIMG0, TIMG6, TIMG7, TIMG8, TIMG12, TIMA0 — 本方案用的 TIMG0/TIMA0/TIMG12 全部合法。

## 6. main.c 框架

```c
#include "board.h"
#include "ti_msp_dl_config.h"
#include "bsp_systick.h"
#include "bsp_printf.h"
#include "motor.h"
#include "encoder.h"
#include "track.h"
#include "pid.h"
#include "oled.h"
#include "gyro_serial.h"
#include "key.h"

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();

    /* 启动电机 PWM 定时器 */
    DL_Timer_startCounter(PWMAB_INST);

    /* 使能编码器中断 */
    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);

    /* OLED */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t*)"MSPM0G Lib Ready");
    OLED_Refresh_Gram();

    /* PID 初始化 */
    Pid_Init();

    /* IMU 初始化 */
    Gyro_Init();

    printf("=== MSPM0G Keil Lib Ready ===\r\n");

    while (1)
    {
        /* 用户在此填任务逻辑 */
        /* 例: 读编码器、PID 计算、Set_PWM 输出 */
    }
}

/* UART1 中断 (IMU) */
void UART_1_INST_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(IMU_INST) == DL_UART_IIDX_RX) {
        Gyro_ParseByte((unsigned char)DL_UART_receiveData(IMU_INST));
    }
}
```

> ⚠️ 中断处理函数名和实例宏需和 syscfg 生成的一致, 改 syscfg 后要核对 `ti_msp_dl_config.h` 里的 `IMU_INST` / `UART_1_INST_IRQn` 命名。

## 7. 任务删除清单 (具体代码位置)

### work.c/work.h — 整个删除
- st_tick / st_encoder / st_motor 结构
- base_speed / quanshu / keyquan / m0 / kaishi_flag / pause_flag / xunji_flag / baohu_flag / biansu_flag / yizhi_flag 全局
- track_state / lock_yaw / turn_target_yaw / turn_dir 状态机变量
- xunji_pid / Timer_work / renwu / work / Car_Move / Car_setspeed / Track_StateMachine / Track_LockYaw
- 注: Car_Move 在 motor.c 也有, 需确认保留哪个 (倾向 motor.c 的, 删 work.c 的)

### empty.c — 删任务, 保留 ISR 框架
- 删: 圈数/状态机/OLED任务画面/变速/过弯保护逻辑
- 留: UART1 ISR (改名为 main.c 里)、TIMER_0 ISR 框架 (空)、SysTick_Handler (空)
- 重命名 empty.c → main.c

### config.h — 删任务常量
- 删 CROSSINGS_TO_A_T1/T2/T4 等

### key.c — 检查任务耦合
- Key() 函数若引用 quanshu/keyquan 等任务变量, 需改为纯按键返回值

## 8. 验证步骤

1. syscfg 改完后, 在 CCS Theia 或 SysConfig CLI 重新生成 ti_msp_dl_config.h/c
2. Keil 打开 .uvprojx, 把 main.c 加入工程 (替换 empty.c), 删 work.c
3. 编译, 确认无未定义符号 (work.c 的全局变量若被其他文件 extern 引用需清理)
4. 烧录到扩展板, OLED 显示 "Ready"、串口打印、按键、电机空转测试

## 9. 风险点

1. **work.c 全局变量被其他文件 extern**: encoder.c/track.c/key.c 可能 extern 了 work.h 的变量, 删 work.c 后会链接报错 — 需逐个排查 extern 声明
2. **ti_msp_dl_config.h 生成环境**: Keil 不能直接生成 syscfg, 需用 CCS Theia 或 sysconfig CLI 预生成后放入 Debug/ 目录
3. **IMU 引脚 UART1 PB4/PB5**: 原 fengzhuang syscfg 的 UART1 引脚是 PB6/PB7, 改 PB4/PB5 后 `UART_1_INST` 宏不变, 但 IRQHandler 名要核对
4. **编码器 B 相缺引脚**: 原理图右编码器 B 相未明确, 可能影响方向判别
