# MSPM0G3507 Keil 封装库改造 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 fengzhuang/25diansai1 圈数循迹任务工程改造为通用 Keil 封装库, 删任务层保底层驱动+PID, 引脚按扩展板原理图重配, OLED 换 I2C 版, 采用 24H_4 定时器分工。

**Architecture:** 以 fengzhuang 现有驱动为主体 (motor/encoder/pid/gyro_serial/bsp 代码层零改动, 靠 syscfg 宏解耦), 删除 work.c 任务层, key.c 重写为纯按键, oled.c 替换为 24H_4 硬件 I2C 版, empty.c 改为 main.c 框架, syscfg 按扩展板原理图重写引脚+定时器分工 (PWM=TIMG0/PID=TIMA0/长计时=TIMG12)。

**Tech Stack:** MSPM0G3507 (Cortex-M0+, 80MHz), mspm0_sdk 2.10, Keil MDK (uvprojx), SysConfig, DL DriverLib.

## Global Constraints

- 芯片 MSPM0G3507, 主频 80MHz (SYSPLL), SDK mspm0_sdk@2.10.00.04
- syscfg 实例 `$name` 必须保持和代码宏一致 (Motor/PWM_0/TIMER_0/ENCODERA/ENCODERB/DEBUG/IMU/OLED/LED/KEY), 只改 pin.$assign 和定时器实例, 代码层零改动
- 禁用引脚: PA2~PA6 (时钟), PA19/PA20 (SWD) — 本方案启用外部 40MHz 晶振占 PA5/PA6, 故 PA2 可作 GPIO (HUIDU-R2 已用), 但 PA19/PA20 绝对保留调试
- API 遵循 mspm0g-contest skill 白名单: DL_TimerA_setCaptureCompareValue (TIMA), DL_I2C_fillControllerTXFIFO+startControllerTransfer (I2C), DL_WWDT_restart (WWDT)
- 可用定时器: TIMG0/PWM, TIMA0/PID, TIMG12/长计时 (全部白名单内)
- 所有代码带中文注释说明 WHY
- 工程根: C:\Users\28442\Desktop\fengzhuang\25diansai1
- 原理图: C:\Users\28442\Downloads\SCH_Schematic1_1-P1_2026-07-22.svg

## File Structure

### 保留 (零改动)
- `Hardware/motor.c/h` — 电机驱动 (Set_PWM/Car_Move), 依赖 syscfg `PWM_0_INST`/`Motor_*` 宏
- `Hardware/encoder.c/h` — 编码器, 依赖 syscfg `ENCODERA_*`/`ENCODERB_*` 宏
- `Hardware/pid.c/h` — 三套 PID (转向/速度环 tPid/角度环), 无引脚依赖
- `Hardware/gyro_serial.c/h` — IMU 0x5A 协议, 依赖 syscfg `IMU_INST`/`UART_1_INST` 宏
- `Hardware/board.c/h` — SysTick_Init + printf 重定向框架, 删 Buzzer 空实现里的任务注释
- `Hardware/bsp_printf.c/h` — any_printf
- `Hardware/bsp_systick.c/h` — delay_ms/us
- `Hardware/track.c/h` — 8路循迹 (引脚写死代码里, 循迹暂不用, 保留备用)
- `keil/startup_mspm0g350x_uvision.s` + `keil/mspm0g3507.sct` — Keil 启动+链接

### 替换
- `Hardware/oled.c/h` + `Hardware/oledfont.h` — 用 24H_4 硬件 I2C 版替换 fengzhuang SPI 版
  - 源: `C:\Users\28442\Desktop\【真题】24年H题#4，一二三四问全部完成\24H_4\user_driver\oled.c/h` + oledfont.h
  - 依赖 syscfg `OLED_INST` (I2C0)

### 重写
- `Hardware/key.c/h` — 重写为纯按键返回值 (原 Key() 是圈数/速度档任务逻辑)
- `empty.c` → `main.c` — 删任务逻辑, 留 init 框架 + while(1) + IMU ISR + TIMER_0 ISR 骨架

### 删除
- `Hardware/work.c/h` — 圈数循迹任务层 (整个删除)
- `Hardware/config.h` — 任务常量 (CROSSINGS_TO_A 等) 整个删除
- `empty.syscfg` — 重写为扩展板原理图引脚 (同名替换)

### 修改 (syscfg 重写)
- `empty.syscfg` — 按扩展板原理图重写所有引脚 + 24H_4 定时器分工

## 引脚映射总表 (扩展板原理图为准)

| 功能 | 引脚 | syscfg 实例 | 引脚名 | 定时器/外设 |
|------|------|------------|--------|------------|
| TB6612 AIN1 | PA8 | Motor | AIN1 | GPIO Output |
| TB6612 AIN2 | PA9 | Motor | AIN2 | GPIO Output |
| TB6612 BIN1 | PB2 | Motor | BIN1 | GPIO Output |
| TB6612 BIN2 | PB3 | Motor | BIN2 | GPIO Output |
| 电机 PWMA | PA12 | PWM_0 | CCP0 | TIMG0 |
| 电机 PWMB | PA13 | PWM_0 | CCP1 | TIMG0 |
| 左编码器 A | PA17 | ENCODERA | E1A | GPIO Input+RISE中断 |
| 左编码器 B | PA24 | ENCODERA | E1B | GPIO Input+RISE中断 |
| 右编码器 A | PA22 | ENCODERB | E2A | GPIO Input+RISE中断 |
| 右编码器 B | (待确认) | ENCODERB | E2B | GPIO Input+RISE中断 |
| 调试 UART0 | PA10/PA11 | DEBUG | TX/RX | UART0 115200 |
| IMU UART1 | PB4/PB5 | IMU | TX/RX | UART1 115200 |
| OLED I2C | PA28/PA31 | OLED | SDA/SCL | I2C0 400kHz 0x3C |
| LED1 | PA7 | LED | led | GPIO Output |
| 拨码 SW3 | PA26 | KEY | K1 | GPIO Input+PULL_UP |
| 按键2 | (待确认) | KEY | K2 | 待确认 |
| 按键3 | (待确认) | KEY | K3 | 待确认 |
| 蜂鸣器 | (待确认) | BEEP | PIN | 待确认 |

## 定时器分工 (24H_4 方案)

| syscfg 实例 | 定时器 | 用途 | 参数 |
|-------------|--------|------|------|
| PWM_0 | TIMG0 | 电机 PWM | EDGE_ALIGN_UP, timerCount=4000 (20kHz), CCP0=PA12/CCP1=PA13 |
| TIMER_0 | TIMA0 | PID 周期 | PERIODIC_UP, 10ms, LOAD 中断 |
| NTB | TIMG12 | 长计时基准 (备用) | PERIODIC_UP, MFCLK, 6000s |

---

### Task 1: 备份工程并清理任务层文件

**Files:**
- Delete: `Hardware/work.c`
- Delete: `Hardware/work.h`
- Delete: `Hardware/config.h`
- Backup: `25diansai1_backup_<date>` (整个工程复制)

**Interfaces:**
- Consumes: 无
- Produces: 干净的工程目录 (无 work.c/h, config.h), 后续任务在此基础操作

**目的:** 先备份, 再删任务层。work.c/h 是圈数循迹任务层 (状态机/变速/过弯保护), config.h 是任务常量, 这俩整个删。

- [ ] **Step 1: 备份整个工程**

```bash
cp -r "C:/Users/28442/Desktop/fengzhuang/25diansai1" "C:/Users/28442/Desktop/fengzhuang/25diansai1_backup_20260722"
```

验证: `ls "C:/Users/28442/Desktop/fengzhuang/25diansai1_backup_20260722"` 存在。

- [ ] **Step 2: 删除 work.c, work.h, config.h**

```bash
rm "C:/Users/28442/Desktop/fengzhuang/25diansai1/Hardware/work.c"
rm "C:/Users/28442/Desktop/fengzhuang/25diansai1/Hardware/work.h"
rm "C:/Users/28442/Desktop/fengzhuang/25diansai1/Hardware/config.h"
```

验证: `ls Hardware/ | grep -E 'work|config'` 应无输出。

- [ ] **Step 3: 检查是否有其他文件 include 了 work.h 或 config.h**

```bash
cd "C:/Users/28442/Desktop/fengzhuang/25diansai1"
grep -rnE '#include "(work|config)\.h"' Hardware/ empty.c 2>/dev/null
```

Expected: 输出 `empty.c:#include "work.h"` (以及可能 key.c)。记录这些文件, Task 4 (main.c) 和 Task 5 (key.c) 会处理。

- [ ] **Step 4: 检查 Keil .uvprojx 是否把 work.c/config.c 列为源文件**

```bash
grep -E 'work\.c|config\.c' "C:/Users/28442/Desktop/fengzhuang/25diansai1/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx"
```

若有引用, 记录下来, Task 7 (Keil 工程配置) 会从工程文件移除。一般 .uvprojx 用通配符或组包含, 不一定逐个列。

- [ ] **Step 5: Commit (若无 git 则跳过, 记录变更)**

工程非 git 仓库, 本步骤记录变更日志即可:
- 删除 Hardware/work.c, work.h, config.h
- 备份于 25diansai1_backup_20260722

---

### Task 2: 替换 OLED 为 24H_4 硬件 I2C 版

**Files:**
- Replace: `Hardware/oled.c` ← 从 24H_4 拷贝
- Replace: `Hardware/oled.h` ← 从 24H_4 拷贝
- Replace: `Hardware/oledfont.h` ← 从 24H_4 拷贝

**Interfaces:**
- Consumes: syscfg `OLED_INST` (I2C0, 将在 Task 6 配置), DL_I2C API
- Produces: `OLED_Init()`, `OLED_Clear()`, `OLED_ShowString()`, `OLED_ShowNumber()`, `OLED_ShowChar()`, `OLED_Refresh_Gram()`, `OLED_ShowSignedNum()` 等 (签名以 24H_4 oled.h 为准)

**目的:** fengzhuang 原 oled.c 是 4 线 SPI (RST/DC/SCL/SDA), 实物板 OLED 是 I2C (PA28/PA31, 0x3C)。24H_4 的 oled.c 用硬件 I2C (DL_I2C_fillControllerTXFIFO + startControllerTransfer), 符合 API 白名单, 直接替换。

- [ ] **Step 1: 拷贝 24H_4 的 oled 三件套覆盖 fengzhuang**

```bash
cp "C:/Users/28442/Desktop/【真题】24年H题#4，一二三四问全部完成/24H_4/user_driver/oled.c" "C:/Users/28442/Desktop/fengzhuang/25diansai1/Hardware/oled.c"
cp "C:/Users/28442/Desktop/【真题】24年H题#4，一二三四问全部完成/24H_4/user_driver/oled.h" "C:/Users/28442/Desktop/fengzhuang/25diansai1/Hardware/oled.h"
cp "C:/Users/28442/Desktop/【真题】24年H题#4，一二三四问全部完成/24H_4/user_driver/oledfont.h" "C:/Users/28442/Desktop/fengzhuang/25diansai1/Hardware/oledfont.h"
```

- [ ] **Step 2: 检查 24H_4 oled.c 的 include 依赖**

```bash
grep -nE '#include' "C:/Users/28442/Desktop/fengzhuang/25diansai1/Hardware/oled.c"
```

Expected: `#include "oled.h"`, `#include "ti_msp_dl_config.h"` 等。确认无 `#include "work.h"` 或其他已删文件。若 oled.c include 了 delay 之类, 确认 fengzhuang 有对应 (bsp_systick.h 提供 delay_ms)。

- [ ] **Step 3: 检查 oled.h 暴露的 API 签名**

```bash
cat "C:/Users/28442/Desktop/fengzhuang/25diansai1/Hardware/oled.h"
```

记录所有函数签名, Task 4 (main.c) 调用 OLED 时要匹配。常见: `OLED_Init()`, `OLED_Clear()`, `OLED_ShowString(x,y,str,size)`, `OLED_Refresh_Gram()`。

- [ ] **Step 4: 检查 oled.c 是否引用了 24H_4 独有宏 (非 OLED_INST)**

```bash
grep -nE 'DC_MOTOR|NTB|xuanniu|VREF|PWMAB|Motor_PID|MOTOR_PID' "C:/Users/28442/Desktop/fengzhuang/25diansai1/Hardware/oled.c"
```

Expected: 无输出 (oled.c 只应依赖 OLED_INST)。若有 24H_4 特有宏, 需在 Task 6 syscfg 里补同名实例, 或在 oled.c 里改宏名。

---

### Task 3: 重写 empty.syscfg 按扩展板原理图

**Files:**
- Rewrite: `empty.syscfg` (同名替换)

**Interfaces:**
- Consumes: 扩展板原理图引脚映射表 (见上)
- Produces: ti_msp_dl_config.h/c (由 SysConfig 生成) 含所有实例宏: Motor_*, PWM_0_INST, GPIO_PWM_0_C0_IDX/C1_IDX, TIMER_0_INST, TIMER_0_INST_INT_IRQN, ENCODERA_*, ENCODERB_*, ENCODERA_INT_IRQN, ENCODERB_INT_IRQN, DEBUG_INST, IMU_INST, UART_1_INST, OLED_INST, GPIO_LED_*, GPIO_KEY_*

**目的:** syscfg 是核心。所有实例 `$name` 保持不变 (代码宏依赖), 只改 pin.$assign 和定时器实例。采用 24H_4 定时器分工 (PWM=TIMG0, PID=TIMA0, 长计时=TIMG12)。

> ⚠️ SysConfig GUI/CLI 生成 ti_msp_dl_config.h 的步骤在 Task 6 处理。本 Task 只写 empty.syscfg 文件内容。

- [ ] **Step 1: 备份原 syscfg**

```bash
cp "C:/Users/28442/Desktop/fengzhuang/25diansai1/empty.syscfg" "C:/Users/28442/Desktop/fengzhuang/25diansai1/empty.syscfg.bak"
```

- [ ] **Step 2: 用以下完整内容覆盖 empty.syscfg**

> 注意: 待确认引脚 (右编码器E2B/按键2/按键3/蜂鸣器) 先用占位引脚或注释掉, 用户确认后在 Task 8 补。下面先写已确认部分, 待确认项用 `/* 待确认 */` 注释并在 syscfg 里先不分配 (SysConfig 会自动 solve)。

完整 empty.syscfg 内容 (写在文件里, 用 Write 工具):

```javascript
/**
 * MSPM0G3507 Keil 封装库 - 扩展板原理图引脚
 * @cliArgs --device "MSPM0G3507" --package "LQFP-64(PM)" --product "mspm0_sdk@2.10.00.04"
 * @versions {"tool":"1.26.2+4477"}
 */

const GPIO   = scripting.addModule("/ti/driverlib/GPIO", {}, false);
const GPIO1  = GPIO.addInstance();  // Motor
const GPIO2  = GPIO.addInstance();  // ENCODERA
const GPIO3  = GPIO.addInstance();  // ENCODERB
const GPIO4  = GPIO.addInstance();  // LED
const GPIO5  = GPIO.addInstance();  // KEY
const I2C    = scripting.addModule("/ti/driverlib/I2C", {}, false);
const I2C1   = I2C.addInstance();   // OLED
const PWM    = scripting.addModule("/ti/driverlib/PWM", {}, false);
const PWM1   = PWM.addInstance();   // PWM_0
const SYSCTL = scripting.addModule("/ti/driverlib/SYSCTL");
const TIMER  = scripting.addModule("/ti/driverlib/TIMER", {}, false);
const TIMER1 = TIMER.addInstance(); // TIMER_0 (PID)
const TIMER2 = TIMER.addInstance(); // NTB (长计时, 备用)
const UART   = scripting.addModule("/ti/driverlib/UART", {}, false);
const UART1  = UART.addInstance();  // DEBUG
const UART2  = UART.addInstance();  // IMU
const Board  = scripting.addModule("/ti/driverlib/Board", {}, false);

/* === 时钟: 80MHz (SYSPLL) === */
const divider9       = system.clockTree["UDIV"];
divider9.divideValue = 2;
const multiplier2         = system.clockTree["PLL_QDIV"];
multiplier2.multiplyValue = 5;
const mux8       = system.clockTree["HSCLKMUX"];
mux8.inputSelect = "HSCLKMUX_SYSPLL0";

/* === Board SWD === */
Board.peripheral.$assign          = "DEBUGSS";
Board.peripheral.swclkPin.$assign = "PA20";
Board.peripheral.swdioPin.$assign = "PA19";

/* === Motor: TB6612 方向控制 (扩展板 PA8/PA9/PB2/PB3) === */
GPIO1.$name                         = "Motor";
GPIO1.associatedPins.create(4);
GPIO1.associatedPins[0].$name       = "AIN1";
GPIO1.associatedPins[0].pin.$assign = "PA8";
GPIO1.associatedPins[1].$name       = "AIN2";
GPIO1.associatedPins[1].pin.$assign = "PA9";
GPIO1.associatedPins[2].$name       = "BIN1";
GPIO1.associatedPins[2].pin.$assign = "PB2";
GPIO1.associatedPins[3].$name       = "BIN2";
GPIO1.associatedPins[3].pin.$assign = "PB3";

/* === ENCODERA: 左编码器 (扩展板 PA17/PA24, 双边沿中断) === */
GPIO2.$name                               = "ENCODERA";
GPIO2.associatedPins.create(2);
GPIO2.associatedPins[0].$name             = "E1A";
GPIO2.associatedPins[0].direction         = "INPUT";
GPIO2.associatedPins[0].polarity          = "RISE_FALL";
GPIO2.associatedPins[0].interruptPriority = "0";
GPIO2.associatedPins[0].interruptEn       = true;
GPIO2.associatedPins[0].pin.$assign       = "PA17";
GPIO2.associatedPins[1].$name             = "E1B";
GPIO2.associatedPins[1].direction         = "INPUT";
GPIO2.associatedPins[1].polarity          = "RISE_FALL";
GPIO2.associatedPins[1].interruptPriority = "0";
GPIO2.associatedPins[1].interruptEn       = true;
GPIO2.associatedPins[1].pin.$assign       = "PA24";

/* === ENCODERB: 右编码器 (扩展板 E2A=PA22, E2B 待确认) === */
GPIO3.$name                               = "ENCODERB";
GPIO3.associatedPins.create(2);
GPIO3.associatedPins[0].$name             = "E2A";
GPIO3.associatedPins[0].direction         = "INPUT";
GPIO3.associatedPins[0].polarity          = "RISE_FALL";
GPIO3.associatedPins[0].interruptPriority = "0";
GPIO3.associatedPins[0].interruptEn       = true;
GPIO3.associatedPins[0].pin.$assign       = "PA22";
GPIO3.associatedPins[1].$name             = "E2B";
GPIO3.associatedPins[1].direction         = "INPUT";
GPIO3.associatedPins[1].polarity          = "RISE_FALL";
GPIO3.associatedPins[1].interruptPriority = "0";
GPIO3.associatedPins[1].interruptEn       = true;
GPIO3.associatedPins[1].pin.$assign       = "PB27";  /* 占位, 待用户确认实物后改 */

/* === LED: LED1 (扩展板 PA7) === */
GPIO4.$name                         = "LED";
GPIO4.associatedPins.create(1);
GPIO4.associatedPins[0].$name       = "led";
GPIO4.associatedPins[0].pin.$assign = "PA7";

/* === KEY: 拨码 SW3 (PA26) + 2按键待确认 === */
GPIO5.$name                              = "KEY";
GPIO5.associatedPins.create(1);  /* 先只配 SW3, 待确认后 create(3) */
GPIO5.associatedPins[0].$name            = "K1";
GPIO5.associatedPins[0].direction        = "INPUT";
GPIO5.associatedPins[0].internalResistor = "PULL_UP";
GPIO5.associatedPins[0].pin.$assign      = "PA26";

/* === OLED: 硬件 I2C0 (扩展板 PA28/PA31, 0x3C, 400kHz) === */
I2C1.$name                             = "OLED";
I2C1.basicEnableController             = true;
I2C1.basicControllerStandardBusSpeed   = "Fast";  /* 400kHz */
I2C1.peripheral.$assign                = "I2C0";
I2C1.peripheral.sdaPin.$assign         = "PA28";
I2C1.peripheral.sclPin.$assign         = "PA31";
I2C1.sdaPinConfig.$name                = "ti_driverlib_gpio_GPIOPinGeneric0";
I2C1.sclPinConfig.$name                = "ti_driverlib_gpio_GPIOPinGeneric1";

/* === PWM_0: 电机 PWM (扩展板 PA12/PA13, TIMG0, 20kHz) === */
PWM1.$name                              = "PWM_0";
PWM1.timerStartTimer                    = true;
PWM1.pwmMode                            = "EDGE_ALIGN_UP";
PWM1.timerCount                         = 4000;  /* 80MHz/4000=20kHz */
PWM1.peripheral.$assign                 = "TIMG0";
PWM1.peripheral.ccp0Pin.$assign         = "PA12";  /* PWMA */
PWM1.peripheral.ccp1Pin.$assign         = "PA13";  /* PWMB */
PWM1.PWM_CHANNEL_0.$name                = "ti_driverlib_pwm_PWMTimerCC0";
PWM1.PWM_CHANNEL_1.$name                = "ti_driverlib_pwm_PWMTimerCC1";
PWM1.ccp0PinConfig.$name                = "ti_driverlib_gpio_GPIOPinGeneric4";
PWM1.ccp1PinConfig.$name                = "ti_driverlib_gpio_GPIOPinGeneric5";

/* === TIMER_0: PID 控制周期 (TIMA0, 10ms, LOAD 中断) — 24H_4 方案 === */
TIMER1.$name              = "TIMER_0";
TIMER1.timerMode          = "PERIODIC_UP";
TIMER1.timerStartTimer    = true;
TIMER1.timerClkPrescale   = 100;
TIMER1.interrupts         = ["LOAD"];
TIMER1.timerPeriod        = "10 ms";
TIMER1.interruptPriority  = "2";
TIMER1.peripheral.$assign = "TIMA0";

/* === NTB: 长计时基准 (TIMG12, MFCLK, 6000s, 备用) — 24H_4 方案 === */
TIMER2.$name              = "NTB";
TIMER2.timerMode          = "PERIODIC_UP";
TIMER2.timerStartTimer    = true;
TIMER2.timerClkDiv        = 8;
TIMER2.timerClkSrc        = "MFCLK";
TIMER2.timerPeriod        = "6000 s";
TIMER2.peripheral.$assign = "TIMG12";

/* === DEBUG: 调试串口 UART0 (扩展板 PA10/PA11, 115200) === */
UART1.$name                    = "DEBUG";
UART1.targetBaudRate           = 115200;
UART1.enabledInterrupts        = ["RX"];
UART1.peripheral.$assign       = "UART0";
UART1.peripheral.rxPin.$assign = "PA11";
UART1.peripheral.txPin.$assign = "PA10";
UART1.txPinConfig.$name        = "ti_driverlib_gpio_GPIOPinGeneric2";
UART1.rxPinConfig.$name        = "ti_driverlib_gpio_GPIOPinGeneric3";

/* === IMU: UART1 (扩展板 PB4/PB5, 115200, RX中断) === */
UART2.$name                    = "IMU";
UART2.targetBaudRate           = 115200;
UART2.enabledInterrupts        = ["RX"];
UART2.peripheral.$assign       = "UART1";
UART2.peripheral.rxPin.$assign = "PB5";
UART2.peripheral.txPin.$assign = "PB4";
UART2.txPinConfig.$name        = "ti_driverlib_gpio_GPIOPinGeneric6";
UART2.rxPinConfig.$name        = "ti_driverlib_gpio_GPIOPinGeneric7";

SYSCTL.forceDefaultClkConfig = true;
SYSCTL.clockTreeEn           = true;
```

- [ ] **Step 3: 自检 syscfg 无语法错误 (人工核对)**

逐项检查:
- 实例 `$name` 和代码宏一致: Motor/PWM_0/TIMER_0/NTB/ENCODERA/ENCODERB/LED/KEY/OLED/DEBUG/IMU ✓
- 引脚无冲突: PA8/PA9/PB2/PB3(Motor), PA12/PA13(PWM), PA17/PA24(ENCODERA), PA22/PB27(ENCODERB), PA7(LED), PA26(KEY K1), PA28/PA31(OLED), PA10/PA11(DEBUG), PB4/PB5(IMU), PA19/PA20(SWD) — 无重复 ✓
- 禁用引脚: PA2~PA6 未用 (PA5/PA6 是 HFX 晶振, 但本 syscfg 未启用外部晶振, 走内部 PLL, 所以 PA5/PA6 也没配, 安全) ✓
- 定时器白名单: TIMG0/TIMA0/TIMG12 全合法 ✓

- [ ] **Step 4: 记录待确认项 (Task 8 补)**

在工程根建 `PIN_TODO.md`:
```
# 待用户确认实物后补充的引脚
- 右编码器 E2B (现占位 PB27)
- 按键 K2, K3 (现只配 K1=PA26)
- 蜂鸣器 BEEP (未配实例)
- 5路循迹 (用户暂不用, syscfg 不配)
```

---

### Task 4: 重写 main.c (从 empty.c 改造)

**Files:**
- Create: `main.c` (工程根, 替代 empty.c)
- Delete: `empty.c` (内容已迁移到 main.c)

**Interfaces:**
- Consumes: syscfg 宏 (PWM_0_INST, TIMER_0_INST_INT_IRQN, ENCODERA_INT_IRQN, ENCODERB_INT_IRQN, IMU_INST, UART_1_INST), 各驱动 init (OLED_Init/Pid_Init/Gyro_Init)
- Produces: `main()`, `UART_1_INST_IRQHandler()` (IMU RX 中断), `TIMER_0_INST_IRQHandler()` (PID 周期, 空 ISR 骨架), `SysTick_Handler()`

**目的:** empty.c 是圈数循迹任务主循环, 全删, 只留 init + while(1) 空框架 + 两个 ISR 骨架。ISR 函数名必须和 syscfg 生成的一致 (UART_1_INST_IRQHandler / TIMER_0_INST_IRQHandler)。

- [ ] **Step 1: 写 main.c 完整内容**

用 Write 工具创建 `C:/Users/28442/Desktop/fengzhuang/25diansai1/main.c`:

```c
/*
 * ============================================================
 * MSPM0G3507 Keil 封装库 - 主程序框架
 * 扩展板原理图引脚 + 24H_4 定时器分工 (PWM=TIMG0, PID=TIMA0)
 *
 * 底层驱动: motor/encoder/track/pid/oled(I2C)/gyro_serial/key/bsp
 * 任务逻辑: 用户在 while(1) 内自行填写
 * ============================================================
 */
#include "board.h"
#include "ti_msp_dl_config.h"
#include "bsp_systick.h"
#include "bsp_printf.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "oled.h"
#include "gyro_serial.h"
#include "key.h"
#include <stdio.h>
#include <string.h>

/* ======================== 全局变量 ======================== */
volatile uint32_t sys_tick_ms = 0;   /* 10ms 节拍 (TIMER_0 中断累加) */

/* ======================== 初始化 ======================== */
int main(void)
{
    SYSCFG_DL_init();

    /* 启动电机 PWM 定时器 (TIMG0) */
    DL_Timer_startCounter(PWM_0_INST);

    /* 使能编码器中断 (双边沿, 方向判别) */
    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);

    /* 使能 TIMER_0 中断 (10ms PID 节拍, TIMA0) */
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    /* SysTick 1ms 基准 (delay_ms 用) */
    SysTick_Init();

    /* OLED I2C 初始化 */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t*)"MSPM0G Lib");
    OLED_Refresh_Gram();

    /* PID 初始化 (转向/速度环/角度环三套) */
    Pid_Init();

    /* IMU 串口初始化 (0x5A 协议, UART1) */
    Gyro_Init();

    printf("=== MSPM0G Keil Lib Ready ===\r\n");

    /* ======================== Main Loop ======================== */
    while (1)
    {
        /* 用户任务逻辑写在这里:
         *   - 读编码器: Get_Encoder_countA() / Get_Encoder_countB()
         *   - 读 IMU: Yaw() (gyro_serial)
         *   - PID 计算: PID_caculate() / Angle_Calculate()
         *   - 电机输出: Set_PWM(a, b) / Car_Move(pl, pr)
         *   - 按键: Key_GetNum() (key.c 重写后)
         *   - OLED 显示: OLED_ShowString/Number + OLED_Refresh_Gram()
         */
    }
}

/* ======================== UART1 中断 (IMU 串口陀螺仪) ======================== */
/* 函数名必须和 syscfg 生成的 UART_1_INST 一致 */
void UART_1_INST_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(IMU_INST) == DL_UART_IIDX_RX) {
        unsigned char byte = (unsigned char)DL_UART_receiveData(IMU_INST);
        Gyro_ParseByte(byte);
    }
    if (DL_UART_getPendingInterrupt(IMU_INST) == DL_UART_IIDX_OVERRUN_ERROR) {
        DL_UART_receiveData(IMU_INST);  /* 清 OVERRUN */
    }
}

/* ======================== 10ms 定时器中断 (TIMA0, PID 节拍) ======================== */
/* TIMER_0 = TIMA0, 用 LOAD 事件, 故 clearInterruptStatus 用 DL_TIMERA_INTERRUPT_LOAD_EVENT */
void TIMER_0_INST_IRQHandler(void)
{
    DL_TimerA_clearInterruptStatus(TIMER_0_INST, DL_TIMERA_INTERRUPT_LOAD_EVENT);

    sys_tick_ms += 10;

    /* 用户 PID 任务逻辑写在这里 (每 10ms 执行一次):
     *   - 读编码器增量
     *   - 速度环 PID_caculate(&EncoderLPid, actual, target)
     *   - 角度环 Angle_Calculate(target, current, 0.01f)
     *   - Set_PWM() 输出
     */
}

/* ======================== SysTick 中断 (空, 1ms tick 由 board.c 的 tick_ms 维护) ======================== */
void SysTick_Handler(void)
{
}
```

- [ ] **Step 2: 删除 empty.c**

```bash
rm "C:/Users/28442/Desktop/fengzhuang/25diansai1/empty.c"
```

- [ ] **Step 3: 核对 ISR 函数名和宏与 syscfg 一致**

```bash
grep -nE 'UART_1_INST_IRQHandler|TIMER_0_INST_IRQHandler|IMU_INST|TIMER_0_INST|PWM_0_INST|ENCODERA_INT_IRQN|ENCODERB_INT_IRQN' "C:/Users/28442/Desktop/fengzhuang/25diansai1/main.c"
```

Expected: 全部出现在 main.c。这些宏由 Task 6 生成的 ti_msp_dl_config.h 提供, 在 Task 6 完成前编译会报未定义, 正常。

- [ ] **Step 4: 检查 main.c 无引用 work.h/config.h**

```bash
grep -nE '#include "(work|config)\.h"' "C:/Users/28442/Desktop/fengzhuang/25diansai1/main.c"
```

Expected: 无输出。

---

### Task 5: 重写 key.c/h 为纯按键返回值

**Files:**
- Rewrite: `Hardware/key.c`
- Rewrite: `Hardware/key.h`

**Interfaces:**
- Consumes: syscfg `GPIO_KEY_K1_PORT`/`GPIO_KEY_K1_PIN` (等, 由 Task 6 生成), bsp_systick (delay_ms)
- Produces: `Key_GetNum()` 返回 uint8_t (0=无按下, 1=K1, 2=K2, 3=K3), `Key_Scan()` 非阻塞返回当前按键状态

**目的:** 原 Key() 是圈数/速度档/状态机任务逻辑 (强耦合 work.h), 引脚写死 PA18/PB8。重写为纯按键扫描, 用 syscfg 宏, 不依赖任何任务变量。参考 car 工程的 KEY.c 风格 (阻塞去抖返回键值)。

- [ ] **Step 1: 重写 key.h**

用 Write 覆盖 `C:/Users/28442/Desktop/fengzhuang/25diansai1/Hardware/key.h`:

```c
#ifndef __KEY_H
#define __KEY_H

#include "board.h"
#include <stdint.h>

/* ============================================================
 * 按键驱动 (纯扫描, 不含任务逻辑)
 *   K1 = 拨码 SW3 (PA26, 上拉, 按下=低电平)
 *   K2, K3 = 待用户确认引脚后补 (见 PIN_TODO.md)
 *
 * 接口:
 *   Key_GetNum() — 阻塞去抖, 返回当前按下键号 (0=无, 1/2/3=K1/K2/K3)
 *   Key_Scan()   — 非阻塞, 返回当前键号 (不去抖, 供中断态查询)
 * ============================================================ */

uint8_t Key_GetNum(void);   /* 阻塞去抖, 松开后返回键号 */
uint8_t Key_Scan(void);     /* 非阻塞, 即时返回键号 */

#endif /* __KEY_H */
```

- [ ] **Step 2: 重写 key.c**

用 Write 覆盖 `C:/Users/28442/Desktop/fengzhuang/25diansai1/Hardware/key.c`:

```c
#include "key.h"
#include "ti_msp_dl_config.h"
#include "bsp_systick.h"

/* ============================================================
 * 按键扫描 (纯驱动, 不含任务逻辑)
 *   引脚用 syscfg 宏 GPIO_KEY_K1_PORT/PIN (KEY 实例, 引脚名 K1)
 *   按键上拉, 按下读到低电平
 *
 * K2/K3 待用户确认引脚后, 在 syscfg 的 KEY 实例补 K2/K3 引脚,
 * 并在此补 GPIO_KEY_K2/K3 宏的检测 (宏由 syscfg 自动生成)
 * ============================================================ */

uint8_t Key_Scan(void)
{
    /* 非阻塞: 返回当前按下的键号, 0=无 */
    if (DL_GPIO_readPins(GPIO_KEY_K1_PORT, GPIO_KEY_K1_PIN) == 0) return 1;
    /* K2/K3 待补 */
    return 0;
}

uint8_t Key_GetNum(void)
{
    /* 阻塞去抖: 检测到按下 → 20ms 去抖 → 等松开 → 20ms 去抖 → 返回键号 */
    uint8_t KeyNum = 0;

    if (DL_GPIO_readPins(GPIO_KEY_K1_PORT, GPIO_KEY_K1_PIN) == 0) {
        delay_ms(20);
        while (DL_GPIO_readPins(GPIO_KEY_K1_PORT, GPIO_KEY_K1_PIN) == 0);  /* 等松开 */
        delay_ms(20);
        KeyNum = 1;
    }
    /* K2/K3 待用户确认引脚后补:
     *   else if (DL_GPIO_readPins(GPIO_KEY_K2_PORT, GPIO_KEY_K2_PIN) == 0) { ... KeyNum = 2; }
     *   else if (DL_GPIO_readPins(GPIO_KEY_K3_PORT, GPIO_KEY_K3_PIN) == 0) { ... KeyNum = 3; }
     */

    return KeyNum;
}
```

- [ ] **Step 3: 检查 key.c 无引用 work.h/track.h/pid.h (任务耦合已清除)**

```bash
grep -nE '#include "(work|track|pid)\.h"|kaishi_flag|quanshu|keyquan|track_state' "C:/Users/28442/Desktop/fengzhuang/25diansai1/Hardware/key.c"
```

Expected: 无输出。

- [ ] **Step 4: 检查 board.h 是否 include key.h (循环依赖排查)**

```bash
grep -nE '#include "key.h"' "C:/Users/28442/Desktop/fengzhuang/25diansai1/Hardware/board.h"
```

board.h 若 include key.h 且 key.h include board.h 会循环。原 fengzhuang key.h 就 include board.h, 这在原工程能编译说明有 include guard 保护, 保持现状即可。

---

### Task 6: 生成 ti_msp_dl_config.h/c 并核对宏

**Files:**
- Generate: `Debug/ti_msp_dl_config.h` + `Debug/ti_msp_dl_config.c` (由 SysConfig 生成)
- Create/Update: `.vscode/c_cpp_properties.json`

**Interfaces:**
- Consumes: empty.syscfg (Task 3)
- Produces: 所有 syscfg 宏供 main.c/motor.c/encoder.c/key.c/gyro_serial.c/oled.c 编译

**目的:** Keil 不能直接生成 syscfg, 需用 SysConfig CLI 或 CCS Theia 预生成 ti_msp_dl_config.h/c, 放入工程 Debug/ 目录 (或 Keil 工程的 include 路径)。

> ⚠️ 这是手工/工具步骤, 需用户在本机执行 SysConfig。下面给出命令和核对清单。

- [ ] **Step 1: 用 SysConfig CLI 生成 ti_msp_dl_config.h/c**

在用户机器执行 (路径按实际 SDK 安装调整):

```bash
# SysConfig CLI 路径 (随 CCS 或 SysConfig 独立安装)
# 常见: C:/ti/ccs2020/ccs/utils/sysconfig_1.26.x/sysconfig_cli.exe
# 或: C:/ti/sysconfig_1.26.x/sysconfig_cli.exe

SYSCFG_CLI="C:/ti/sysconfig_1.26.2/sysconfig_cli.exe"
SDK="C:/ti/mspm0_sdk_2_10_00_04"
PROJ="C:/Users/28442/Desktop/fengzhuang/25diansai1"

"$SYSCFG_CLI" -s "$SDK/.metadata/product.json" \
    --output "$PROJ/Debug" \
    "$PROJ/empty.syscfg"
```

生成产物: `$PROJ/Debug/ti_msp_dl_config.h` 和 `ti_msp_dl_config.c`。

> 若用户用 CCS Theia: 打开 empty.syscfg → 保存 → 自动生成到 Debug/syscfg/ 目录。

- [ ] **Step 2: 核对生成的 ti_msp_dl_config.h 含所有预期宏**

```bash
grep -oE '#define (Motor_AIN1|Motor_AIN2|Motor_BIN1|Motor_BIN2|PWM_0_INST|GPIO_PWM_0_C0_IDX|GPIO_PWM_0_C1_IDX|TIMER_0_INST|TIMER_0_INST_INT_IRQN|ENCODERA_PORT|ENCODERA_E1A_PIN|ENCODERA_E1B_PIN|ENCODERB_PORT|ENCODERB_E2A_PIN|ENCODERB_E2B_PIN|ENCODERA_INT_IRQN|ENCODERB_INT_IRQN|DEBUG_INST|IMU_INST|UART_1_INST|UART_1_INST_INT_IRQN|OLED_INST|GPIO_LED_led_PIN|GPIO_KEY_K1_PORT|GPIO_KEY_K1_PIN|NTB_INST)' "C:/Users/28442/Desktop/fengzhuang/25diansai1/Debug/ti_msp_dl_config.h"
```

Expected: 全部 28 个宏都出现。若有缺失, 回 Task 3 检查 syscfg 实例名拼写。

- [ ] **Step 3: 核对 IRQHandler 函数名约定**

SysConfig 会在 ti_msp_dl_config.h 顶部注释或 dl_interrupt.h 里声明 IRQHandler。核对:

```bash
grep -nE 'UART_1_INST_IRQHandler|TIMER_0_INST_IRQHandler|ENCODERA_INT_IRQHandler|ENCODERB_INT_IRQHandler' "C:/Users/28442/Desktop/fengzhuang/25diansai1/Debug/ti_msp_dl_config.h" "C:/Users/28442/Desktop/fengzhuang/25diansai1/Debug/ti_msp_dl_config.c"
```

Expected: main.c 里的 `UART_1_INST_IRQHandler` 和 `TIMER_0_INST_IRQHandler` 函数名要和这里一致。若 syscfg 生成的中断函数名不同 (如 `ENCODERA_INT_IRQHandler` vs `GPIO_ENCODERA_INT_IRQHandler`), 调整 main.c 的函数名匹配。

- [ ] **Step 4: 更新 .vscode/c_cpp_properties.json**

用 Write 覆盖 `C:/Users/28442/Desktop/fengzhuang/25diansai1/.vscode/c_cpp_properties.json` (确保 VS Code 不误报):

```json
{
    "configurations": [
        {
            "name": "MSPM0G3507_Config",
            "includePath": [
                "${workspaceFolder}",
                "${workspaceFolder}/Debug",
                "${workspaceFolder}/Hardware",
                "C:/ti/mspm0_sdk_2_10_00_04/source",
                "C:/ti/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include"
            ],
            "defines": [
                "__MSPM0G3507__",
                "__USE_SYSCONFIG__"
            ],
            "compilerPath": "C:/ti/ccs2020/ccs/tools/compiler/ti-cgt-armllvm_4.0.3.LTS/bin/tiarmclang.exe",
            "compilerArgs": [
                "-mcpu=cortex-m0plus",
                "-march=thumbv6m",
                "-mthumb",
                "-mfloat-abi=soft"
            ],
            "cStandard": "c11",
            "cppStandard": "c++11",
            "intelliSenseMode": "windows-clang-arm",
            "browse": {
                "path": [
                    "${workspaceFolder}",
                    "${workspaceFolder}/Debug",
                    "${workspaceFolder}/Hardware",
                    "C:/ti/mspm0_sdk_2_10_00_04/source"
                ],
                "limitSymbolsToIncludedHeaders": true
            }
        }
    ],
    "version": 4
}
```

> 注: 编译器路径 tiarmclang.exe 是 CCS ARMCLANG, Keil 用 AC5/AC6 (armclang), VS Code 配置只影响智能提示不影响 Keil 编译。Keil 编译靠 .uvprojx 设置。

---

### Task 7: Keil 工程配置更新

**Files:**
- Modify: `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`

**Interfaces:**
- Consumes: main.c (Task 4), 各 Hardware/*.c, Debug/ti_msp_dl_config.c (Task 6)
- Produces: 可编译的 Keil 工程

**目的:** Keil 工程文件要: 移除 work.c/config.c 引用, 加入 main.c, 确保 ti_msp_dl_config.c 在编译列表, include 路径含 Debug/ 和 Hardware/。

- [ ] **Step 1: 查看 .uvprojx 当前源文件组和 include 路径**

```bash
grep -nE '<File|<FilePath|<FileName|IncludePath|<Group|<GroupName' "C:/Users/28442/Desktop/fengzhuang/25diansai1/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx" | head -60
```

记录: 现有文件组结构 (Groups), include 路径, 哪些 .c 已列入。

- [ ] **Step 2: 在 .uvprojx 里把 empty.c 替换为 main.c**

用 Edit 工具, 把 `<FileName>empty.c</FileName>` 和对应 `<FilePath>../empty.c</FilePath>` 改为 `main.c` / `../main.c`。

具体 Edit 取决于 Step 1 看到的 XML 结构。典型:
- `<File><FileName>empty.c</FileName><FileType>1</FileType><FilePath>../empty.c</FilePath></File>`
- 改为 `<File><FileName>main.c</FileName><FileType>1</FileType><FilePath>../main.c</FilePath></File>`

- [ ] **Step 3: 移除 work.c, config.c 的 File 节点 (若有)**

```bash
grep -nE '<FileName>(work|config)\.c</FileName>' "C:/Users/28442/Desktop/fengzhuang/25diansai1/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx"
```

若有, 删除对应 `<File>...</File>` 整块 (用 Edit 删除)。

- [ ] **Step 4: 确保 ti_msp_dl_config.c 在编译列表**

```bash
grep -nE 'ti_msp_dl_config\.c' "C:/Users/28442/Desktop/fengzhuang/25diansai1/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx"
```

若无, 添加 `<File><FileName>ti_msp_dl_config.c</FileName><FileType>1</FileType><FilePath>../Debug/ti_msp_dl_config.c</FilePath></File>` 到某个 Group。

- [ ] **Step 5: 确保 IncludePath 含 Debug/ 和 Hardware/**

```bash
grep -oE 'IncludePath>[^<]+' "C:/Users/28442/Desktop/fengzhuang/25diansai1/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx"
```

IncludePath 是分号分隔列表。确认含 `../Debug` 和 `../Hardware` 和 SDK source 路径。若缺, 用 Edit 补到 IncludePath 字符串里。

- [ ] **Step 6: 确认 oled.c (I2C 版) 在编译列表 (替换后路径不变, 应自动生效)**

oled.c 路径 `../Hardware/oled.c` 不变 (只是内容换了), .uvprojx 无需改。确认:
```bash
grep -nE '<FileName>oled\.c</FileName>' "C:/Users/28442/Desktop/fengzhuang/25diansai1/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx"
```
Expected: 存在。

---

### Task 8: 用户确认实物后补全待定引脚

**Files:**
- Modify: `empty.syscfg` (补 E2B/K2/K3/BEEP 引脚)
- Modify: `Hardware/key.c` (补 K2/K3 检测)
- Regenerate: `Debug/ti_msp_dl_config.h/c`

**Interfaces:**
- Consumes: 用户提供的实物引脚 (右编码器B相, 按键2/3, 蜂鸣器)
- Produces: 完整 syscfg + key.c

**目的:** Task 3 的占位引脚 (E2B=PB27, KEY 只配 K1) 在用户确认实物后补全。此 Task 是"等用户反馈"的占位, 不阻塞 Task 1-7。

- [ ] **Step 1: 用户看实物板, 记录引脚**

请用户确认并填入 PIN_TODO.md:
- 右编码器 E2B 接哪个引脚?
- 按键 K2, K3 接哪?
- 蜂鸣器接哪?

- [ ] **Step 2: 更新 empty.syscfg**

改 `GPIO3.associatedPins[1].pin.$assign` 为实物 E2B 引脚。
`GPIO5.associatedPins.create(3)` 加 K2/K3 并配引脚。
新增 `GPIO6` BEEP 实例配蜂鸣器引脚。

- [ ] **Step 3: 更新 key.c 补 K2/K3**

在 key.c 的 `Key_Scan()` 和 `Key_GetNum()` 补 GPIO_KEY_K2/K3 检测分支。

- [ ] **Step 4: 重新生成 ti_msp_dl_config.h/c (同 Task 6 Step 1)**

- [ ] **Step 5: 编译验证 (Keil Ctrl+F7 全编译)**

---

### Task 9: 编译验证与上板测试

**Files:**
- All (编译验证)

**Interfaces:**
- Consumes: 全部前置 Task
- Produces: 可烧录的 .hex/.axf

**目的:** Keil 全编译, 修剩余未定义符号, 烧录上板验证基础功能。

- [ ] **Step 1: Keil 全编译 (Build Target)**

在 Keil 打开 `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx` → F7 (Build)。

Expected: 0 Error。若有 Error, 常见原因:
- `Undefined symbol Kaishi_flag` 等 work.c 变量 → 说明某文件还 extern 了 work.h 变量, 搜索清除
- `OLED_ShowString 参数不匹配` → 24H_4 oled.h 签名和 fengzhuang 调用不同, 调整 main.c 调用
- `UART_1_INST_IRQHandler 重复定义` → syscfg 生成的 ti_msp_dl_config.c 里也有同名 weak 定义, 正常 (main.c 的非 weak 版覆盖)

- [ ] **Step 2: 排查链接错误 (若有 work.h 残留 extern)**

```bash
cd "C:/Users/28442/Desktop/fengzhuang/25diansai1"
grep -rnE 'extern.*(base_speed|quanshu|kaishi_flag|track_state|Tick|Encoder|Motor|sys_tick_ms|time_2s)' Hardware/ main.c 2>/dev/null | grep -v 'pid.h'
```

清除所有对已删 work.h 变量的 extern 引用。`sys_tick_ms` 现定义在 main.c, 若其他文件 extern 它, 保留 (main.c 已定义)。

- [ ] **Step 3: 烧录 (XDS110 SWD)**

CCS Theia 或 UniFlash, SWD (PA19/PA20), 烧录 .axf/.hex。

- [ ] **Step 4: 上板验证基础功能**

逐项验证:
- [ ] OLED 显示 "MSPM0G Lib" (I2C 0x3C 通信 OK)
- [ ] 串口 (PA10/PA11, 115200) 打印 "=== MSPM0G Keil Lib Ready ==="
- [ ] 拨码 SW3 (PA26) 按下 Key_GetNum() 返回 1
- [ ] 电机 PWM: 临时在 while(1) 写 `Set_PWM(2000, 2000)` 验证双轮转动 (方向对不对)
- [ ] 编码器: 临时打印 Get_Encoder_countA/B 验证计数
- [ ] IMU: 临时打印 Yaw() 验证 0x5A 协议解析

- [ ] **Step 4: 更新记忆 (扩展板引脚实测确认后)**

用户确认实物引脚后, 更新 memory `expansion-board-pinout.md`。

---

## Self-Review 记录

**Spec coverage:**
- 保留底层驱动 (motor/encoder/pid/gyro/bsp/track) — Task 1 (只删 work/config) + 各驱动不动 ✓
- 替换 OLED I2C — Task 2 ✓
- 重写 syscfg 扩展板引脚 + 24H_4 定时器分工 — Task 3 ✓
- main.c 框架删任务 — Task 4 ✓
- key.c 重写纯按键 — Task 5 ✓
- ti_msp_dl_config 生成 — Task 6 ✓
- Keil 工程配置 — Task 7 ✓
- 待定引脚补全 — Task 8 ✓
- 编译上板 — Task 9 ✓

**Placeholder scan:** Task 8 是"等用户反馈"占位, 已明确标注不阻塞前置 Task。syscfg 里 E2B=PB27 是占位, 已在 PIN_TODO.md 记录。无其他 TBD。

**Type consistency:** ISR 函数名 (UART_1_INST_IRQHandler / TIMER_0_INST_IRQHandler) 在 Task 4 (main.c 定义) 和 Task 6 (syscfg 生成核对) 一致。Key_GetNum 在 Task 5 定义, Task 4 main.c 注释引用一致。
