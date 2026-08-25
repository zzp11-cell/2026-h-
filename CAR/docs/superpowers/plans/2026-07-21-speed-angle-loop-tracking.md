# 速度环 + 角度环循迹 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 25diansai1 循迹从「速度环 + 位置环(循迹偏差纠偏)」改为「速度环 + 角度环(陀螺仪 yaw 锁航向跑直)+ 过弯定点转 90° 状态机」,复用 2024QuestionH 已验证的角度环跑直逻辑。

**Architecture:** 三态状态机(TRACK_STRAIGHT / TRACK_TURN)在 TIMER_0 10ms ISR 内调用。直线态:角度环 `Angle_Calculate(lock_yaw, yaw)` 输出 `steering`,叠加在速度环输出的 PWM 层做差速(`final_left = base_pwm - STEER_SIGN*steering`)。转向态:按上次方向原地差速转 90°,到位(转满 90° 或重新压线)后重新锁 `lock_yaw` 回直线态。

**Tech Stack:** MSPM0G3507 (Cortex-M0+, 32MHz), TI MSPM0 SDK 2.01.00.03, DriverLib `dl_xxx`, bare-metal C, CCS/Theia 工程结构。

## Global Constraints

- 方向约定沿用 25diansai1 自己的(不照搬 2024H):左轮=A 通道(`Encoder_CountA`/`Velocity_A`/`countL`),右轮=B 通道(`Encoder_CountB`/`Velocity_B`/`countR`),`Set_PWM(pwmA=右, pwmB=左)`,正 PWM=前进。
- 角度环 PID 参数复用 2024H 实测值:`ANGLE_KP=80, ANGLE_KI=0.5, ANGLE_KD=3, MAX_STEERING=4500`。
- 不删除旧循迹代码:`SteeringPID`/`TurnErrorPid`/`xunji_pid()` 保留(可还原)。新增函数并行存在。
- 不改 motor.c / encoder.c / gyro_serial.c / bsp_* / oled.c / ti_msp_dl_config.*。
- 所有新代码带完整中文注释(WHY)。模块化,不堆 main。
- 工程非 git 仓库,无法 commit;每个任务末尾用「静态分析 + 上车验证」替代 git commit。

**验证方式说明(适配嵌入式):** 本工程无单元测试框架(裸机固件,XDS110 烧录)。每个任务的「测试」= MATLAB Code Analyzer 静态检查(若 .m 不适用则跳过)+ CCS/Theia 编译通过 + 上车实测。下面每个任务的「Run/Expected」给出具体的上车验证步骤与预期现象。

---

## File Structure

| 文件 | 责任 | 改动类型 |
|------|------|----------|
| `Hardware/pid.h` | 新增角度环接口声明 | 修改(追加) |
| `Hardware/pid.c` | 新增角度环实现 `Angle_Calculate` / `Angle_PID_Reset` | 修改(追加) |
| `Hardware/track.h` | 新增 `Track_HasLine()` / `Track_GetLastDirection()` 声明 | 修改(追加) |
| `Hardware/track.c` | 新增 `Track_HasLine()` / `Track_GetLastDirection()` 实现 | 修改(追加) |
| `Hardware/work.h` | 新增状态机枚举、全局变量、`Track_StateMachine()` 声明 | 修改(追加) |
| `Hardware/work.c` | 新增状态机实现 `Track_StateMachine()` | 修改(追加) |
| `empty.c` | TIMER_0 ISR 改用状态机;OLED 显示 yaw/状态;`limit_PWM` 辅助 | 修改 |
| `Hardware/key.c` | 启动时复位角度环积分 + 重锁 `lock_yaw` | 修改 |

---

### Task 1: 角度环 PID 模块(pid.c / pid.h)

**Files:**
- Modify: `Hardware/pid.h`(文件末尾追加)
- Modify: `Hardware/pid.c`(文件末尾追加)

**Interfaces:**
- Consumes: 无(纯新增,复用 2024H 参数)
- Produces:
  - `float Angle_Calculate(float target, float current, float dt)` — 角度环计算,返回限幅后的 steering(±MAX_STEERING)。`target`/`current` 单位 deg,`dt` 秒。
  - `void Angle_PID_Reset(void)` — 清角度环积分/上次误差。
  - 宏 `ANGLE_KP 80.0f`, `ANGLE_KI 0.5f`, `ANGLE_KD 3.0f`, `MAX_STEERING 4500`。

- [ ] **Step 1: 在 pid.h 末尾追加角度环声明**

在 `Hardware/pid.h` 的 `#endif /* __PID_H */` 之前追加:

```c
/* ============================================================
 * 角度环 PID (移植自 2024QuestionH pid.c, 实测能跑直)
 *   输入: target/current = 目标/当前 yaw (deg), dt = 控制周期 (秒)
 *   输出: steering 转向量 (PWM 量纲, 限幅 ±MAX_STEERING)
 *   用于直线态锁 lock_yaw 跑直: 消除 yaw 漂移
 * ============================================================ */
#define ANGLE_KP        80.0f
#define ANGLE_KI        0.5f
#define ANGLE_KD        3.0f
#define MAX_STEERING    4500

float Angle_Calculate(float target, float current, float dt);
void  Angle_PID_Reset(void);
```

- [ ] **Step 2: 在 pid.c 末尾追加角度环实现**

在 `Hardware/pid.c` 文件末尾追加:

```c

/* ============================================================
 * 角度环 PID (移植自 2024QuestionH pid.c Angle_Calculate)
 *   实测参数 KP=80 KI=0.5 KD=3, 限幅 4500, 10ms 节拍跑直稳定.
 *   normalize_angle 把误差折回 [-180,180], 避免跨越 ±180 时跳变.
 * ============================================================ */
static float angle_integral    = 0.0f;
static float angle_prev_error  = 0.0f;

static float normalize_angle(float angle)
{
    while (angle >  180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

void Angle_PID_Reset(void)
{
    angle_integral   = 0.0f;
    angle_prev_error = 0.0f;
}

float Angle_Calculate(float target, float current, float dt)
{
    float error = normalize_angle(target - current);

    float P = ANGLE_KP * error;
    angle_integral += error * dt;
    if (angle_integral >  100.0f) angle_integral =  100.0f;
    if (angle_integral < -100.0f) angle_integral = -100.0f;
    float I = ANGLE_KI * angle_integral;
    float derivative = (error - angle_prev_error) / dt;
    float D = ANGLE_KD * derivative;
    angle_prev_error = error;

    float output = P + I + D;
    if (output >  MAX_STEERING) output =  MAX_STEERING;
    if (output < -MAX_STEERING) output = -MAX_STEERING;
    return output;
}
```

- [ ] **Step 3: 编译验证**

在 CCS/Theia 工程里 Build(F11 或 Ctrl+B)。
Expected: 无新增 error/warning。`Angle_Calculate`/`Angle_PID_Reset` 可被引用(此时无调用方,编译应通过)。

- [ ] **Step 4: 静态分析(可选,无 .m 文件则跳过)**

本任务是 .c/.h,无 MATLAB Code Analyzer 适用对象,跳过。改为目检:`normalize_angle` 循环终止条件正确(每次 ±360,必收敛)。

- [ ] **Step 5: 记录完成**

工程非 git,无 commit。在任务清单勾选完成,记录「pid.c 角度环已加,编译通过」。

---

### Task 2: 循迹辅助接口 Track_HasLine / Track_GetLastDirection(track.c / track.h)

**Files:**
- Modify: `Hardware/track.h`(声明区追加)
- Modify: `Hardware/track.c`(文件末尾追加)

**Interfaces:**
- Consumes: `Track_GetState()`(track.c 已有)、`s_last_direction`(track.c 已有 static)
- Produces:
  - `uint8_t Track_HasLine(void)` — 返回 1 表示至少一路压到黑线(state != 0),0 表示全白丢线。
  - `int8_t Track_GetLastDirection(void)` — 返回 `s_last_direction`(-1 线曾在左 / +1 线曾在右 / 0 未知)。

- [ ] **Step 1: 在 track.h 追加声明**

在 `Hardware/track.h` 的 `Track_GetLostCrossed`/`Track_ClearLostCrossed` 声明之后、`#endif` 之前追加:

```c
/* 是否压到任意黑线 (state != 0). 供角度环状态机判断"重新找到线"用 */
uint8_t Track_HasLine(void);

/* 上次有效方向: -1=线曾在左, +1=线曾在右, 0=未知. 供转向态定转向方向 */
int8_t Track_GetLastDirection(void);
```

- [ ] **Step 2: 在 track.c 末尾追加实现**

在 `Hardware/track.c` 文件末尾(现有 `Track_GetLostCrossed`/`Track_ClearLostCrossed` 之后)追加:

```c

/* 是否压到任意黑线: state 非 0 即有路压黑线.
 * 角度环状态机转向态用它判断"是否重新找到线"以双保险回直线态. */
uint8_t Track_HasLine(void)
{
    return (Track_GetState() != 0) ? 1u : 0u;
}

/* 暴露丢线方向记忆, 供转向态决定原地左转/右转 */
int8_t Track_GetLastDirection(void)
{
    return s_last_direction;
}
```

- [ ] **Step 3: 编译验证**

Build 工程。
Expected: 无 error。`Track_HasLine`/`Track_GetLastDirection` 可被引用。

- [ ] **Step 4: 上车验证(单独可验)**

烧录后,车抬起轮子离地,BLS 启动前在 OLED 待机画面(原代码已显示 8 路 state)观察:手挡 CH0~CH7 任一路,对应位应显示 1。此为原有功能,确认未破坏即可。

- [ ] **Step 5: 记录完成**

勾选,记录「track.c 辅助接口已加,OLED 8 路显示正常」。

---

### Task 3: 状态机骨架与全局变量(work.c / work.h)

**Files:**
- Modify: `Hardware/work.h`(追加枚举、变量、函数声明)
- Modify: `Hardware/work.c`(追加状态机实现)

**Interfaces:**
- Consumes: `Angle_Calculate`/`Angle_PID_Reset`(Task 1)、`Track_HasLine`/`Track_GetLastDirection`(Task 2)、`Yaw()`(gyro_serial.h 已有)、`base_speed`(work.c 已有)、`Set_PWM`/`Velocity_A`/`Velocity_B`/`Get_Encoder_countA`/`Get_Encoder_countB`(已有)。
- Produces:
  - 枚举 `TrackState { TRACK_STRAIGHT=0, TRACK_TURN=1 }`。
  - 全局 `volatile uint8_t track_state`、`volatile float lock_yaw`、`volatile float turn_target_yaw`、`volatile int8_t turn_dir`。
  - 宏 `STEER_SIGN (+1)`、`TURN_PWM (2000)`、`TURN_DEADBAND (5.0f)`、`TURN_ANGLE (90.0f)`。
  - `void Track_StateMachine(int countL, int countR)` — 在 TIMER_0 ISR 内调用,读当前 yaw + 编码器计数,直接输出 PWM 到电机。**本任务先实现直线态 + 转向态空壳(转向态先写死原地差速 + 到位判定),后续 Task 4 接入 ISR。**

- [ ] **Step 1: 在 work.h 追加状态机声明**

在 `Hardware/work.h` 的 `lost_flag` extern 声明之后、`/* ---- API ---- */` 之前追加:

```c
/* ============================================================
 * 角度环循迹状态机 (替代旧 xunji_pid 位置环纠偏)
 *   TRACK_STRAIGHT: 角度环锁 lock_yaw 跑直, 速度环两轮同速,
 *                   steering 叠加在 PWM 层做差速.
 *   TRACK_TURN:     按 turn_dir 原地差速转 90 度, 到位后回直线态, m0++.
 * ============================================================ */
typedef enum {
    TRACK_STRAIGHT = 0,   /* 直线态: 角度环锁航向跑直 */
    TRACK_TURN     = 1    /* 转向态: 原地差速过弯 */
} TrackState;

/* 状态机全局变量 (定义在 work.c, empty.c TIMER_0 ISR 读写) */
extern volatile uint8_t track_state;       /* 当前态 (TrackState) */
extern volatile float   lock_yaw;          /* 直线态锁定的目标航向 (deg) */
extern volatile float   turn_target_yaw;   /* 转向态目标 yaw (deg) */
extern volatile int8_t  turn_dir;          /* 转向方向: -1左 / +1右 */

/* 方向/参数宏 (上车试):
 *   STEER_SIGN: 角度环差速符号; 若试车修正方向反了改 -1
 *   TURN_PWM:   转向态原地差速 PWM
 *   TURN_DEADBAND: 转向到位 yaw 误差死区 (度)
 *   TURN_ANGLE: 过弯转角 (矩形赛道 90 度) */
#define STEER_SIGN       (+1)
#define TURN_PWM         (2000)
#define TURN_DEADBAND    (5.0f)
#define TURN_ANGLE       (90.0f)
```

在 `work.h` 的 API 区(`void xunji_pid(void);` 之后)追加:

```c
/* 角度环状态机: 在 TIMER_0 ISR (10ms) 内调用, 直出 PWM.
 *   countL/countR = 本 10ms 编码器增量 (左A/右B) */
void Track_StateMachine(int countL, int countR);

/* 锁定当前航向 (启动/出弯时调用) */
void Track_LockYaw(void);
```

- [ ] **Step 2: 在 work.c 追加状态机实现**

在 `Hardware/work.c` 文件末尾(`void work(void) {}` 之后)追加。需在文件顶部 include 区确认已有 `#include "track.h"`、`#include "gyro_serial.h"`(若缺则补):

```c

/* ============================================================
 * 角度环循迹状态机 (替代 xunji_pid)
 *   直线态: 角度环 lock_yaw → steering, 叠加在速度环 PWM 层做差速.
 *   转向态: 按 turn_dir 原地差速转 90 度.
 * 方向约定 (沿用 25diansai1):
 *   左轮=A (countL, Velocity_A), 右轮=B (countR, Velocity_B).
 *   Set_PWM(pwmA=右, pwmB=左). 正 PWM=前进.
 * ============================================================ */
volatile uint8_t track_state    = TRACK_STRAIGHT;
volatile float   lock_yaw       = 0.0f;
volatile float   turn_target_yaw= 0.0f;
volatile int8_t  turn_dir       = 0;

/* 锁定当前航向 + 复位角度环积分.
 * 启动/出弯回直线态时调用. */
void Track_LockYaw(void)
{
    lock_yaw = Yaw();
    Angle_PID_Reset();
    track_state = TRACK_STRAIGHT;
}

/* 在 TIMER_0 ISR (10ms) 内调用: 读 yaw + 编码器, 直出 PWM. */
void Track_StateMachine(int countL, int countR)
{
    float yaw = Yaw();

    /* ---------- 转向态: 原地差速过弯 ---------- */
    if (track_state == TRACK_TURN) {
        int final_left, final_right;
        if (turn_dir < 0) {
            /* 左转: 左轮退 右轮进 */
            final_left  = -TURN_PWM;
            final_right =  TURN_PWM;
        } else {
            /* 右转: 左轮进 右轮退 */
            final_left  =  TURN_PWM;
            final_right = -TURN_PWM;
        }
        Set_PWM(final_right, final_left);   /* Set_PWM(右, 左) */

        /* 到位双保险: 转满 90 度 或 重新压到黑线 */
        float err = turn_target_yaw - yaw;
        while (err >  180.0f) err -= 360.0f;
        while (err < -180.0f) err += 360.0f;
        if (err < 0.0f) err = -err;          /* fabsf */

        if (err < TURN_DEADBAND || Track_HasLine()) {
            /* 回直线态: 重新锁当前航向, 过弯计数 +1 */
            Track_LockYaw();
            m0++;
            baohu_flag = 1;   /* 防重复计数 (沿用原机制) */
            time_3s = 0;
        }
        return;   /* 转向态不走速度环/角度环 */
    }

    /* ---------- 直线态: 角度环锁 lock_yaw 跑直 ---------- */
    /* 速度环: 两轮同目标 base_speed */
    float pwma = Velocity_A((int)base_speed, countL);   /* A=左轮 */
    float pwmb = Velocity_B((int)base_speed, countR);   /* B=右轮 */
    int  base_pwm = (int)((pwma + pwmb) * 0.5f);

    /* 角度环: 锁 lock_yaw, 输出 steering (PWM 量纲, ±4500) */
    float steering = Angle_Calculate(lock_yaw, yaw, 0.01f);

    int final_left  = base_pwm - STEER_SIGN * (int)steering;
    int final_right = base_pwm + STEER_SIGN * (int)steering;

    /* 限幅 ±7999 */
    if (final_left  >  7999) final_left  =  7999;
    if (final_left  < -7999) final_left  = -7999;
    if (final_right >  7999) final_right =  7999;
    if (final_right < -7999) final_right = -7999;

    Set_PWM(final_right, final_left);   /* Set_PWM(右, 左) */

    /* 丢线检测: 全白 → 进转向态, 按上次方向转 90 度 */
    if (!Track_HasLine()) {
        turn_dir = Track_GetLastDirection();
        if (turn_dir == 0) turn_dir = 1;   /* 未知方向默认右转 */
        /* 目标 yaw = 当前 yaw + 方向*90 (左转 yaw 减, 右转 yaw 加) */
        float tgt = yaw + turn_dir * TURN_ANGLE;
        while (tgt >  180.0f) tgt -= 360.0f;
        while (tgt < -180.0f) tgt += 360.0f;
        turn_target_yaw = tgt;
        track_state = TRACK_TURN;
        Angle_PID_Reset();
    }
}
```

- [ ] **Step 3: 补 include(若缺)**

检查 `Hardware/work.c` 顶部 include 区是否已有 `#include "track.h"` 和 `#include "gyro_serial.h"`。已有 `#include "track.h"`(第2行)。若无 `gyro_serial.h`,在 `#include "track.h"` 后补一行:

```c
#include "gyro_serial.h"
```

- [ ] **Step 4: 编译验证**

Build 工程。
Expected: 无 error。`Track_StateMachine`/`Track_LockYaw` 可被引用(此时 ISR 还没调用,编译应通过)。

- [ ] **Step 5: 记录完成**

勾选,记录「work.c 状态机已加,编译通过」。

---

### Task 4: TIMER_0 ISR 接入状态机(empty.c)

**Files:**
- Modify: `empty.c:151-229`(`TIMER_0_INST_IRQHandler`)

**Interfaces:**
- Consumes: `Track_StateMachine`/`Track_LockYaw`/`track_state`(Task 3)、`Angle_PID_Reset`(Task 1)。原 `xunji_pid`/`track_diff`/`lost_flag` 不再调用(保留定义)。
- Produces: ISR 内循迹走状态机,不再走 `xunji_pid` + `track_diff` 差速。

- [ ] **Step 1: 改 TIMER_0 ISR 的循迹+速度环节**

在 `empty.c` 中,把 `TIMER_0_INST_IRQHandler` 里 `if (kaishi_flag == 1 && pause_flag == 0) { ... }` 块内的「位置环 + 速度环」逻辑替换为状态机调用。定位这段(约 159~191 行):

```c
    if (kaishi_flag == 1 && pause_flag == 0) {
        /* 位置环: 循迹纠偏 */
        if (xunji_flag >= 1) xunji_pid();

        /* 速度环: 增量式 PI + RPM 反馈 */
        {
            int countL = Get_Encoder_countA();
            Encoder.left  = (int16_t)countL;
            int countR = Get_Encoder_countB();
            Encoder.right = (int16_t)countR;

            if (!lost_flag) {
                Motor.run_Lspeed = base_speed - track_diff;
                Motor.run_Rspeed = base_speed + track_diff;
            }

            int PWMA = (int)Velocity_A((int)Motor.run_Lspeed, (int)countL);
            int PWMB = (int)Velocity_B((int)Motor.run_Rspeed, (int)countR);

            if (PWMA > 7999) PWMA = 7999;
            if (PWMA < -7999) PWMA = -7999;
            if (PWMB > 7999) PWMB = 7999;
            if (PWMB < -7999) PWMB = -7999;

            Set_PWM(PWMA, PWMB);
        }
```

替换为:

```c
    if (kaishi_flag == 1 && pause_flag == 0) {
        /* 读编码器 (左A/右B), 供状态机速度环用 */
        int countL = Get_Encoder_countA();
        Encoder.left  = (int16_t)countL;
        int countR = Get_Encoder_countB();
        Encoder.right = (int16_t)countR;

        /* 角度环状态机: 直线态锁 yaw 跑直 + 转向态过弯.
         * 状态机内部完成速度环 + 角度环 + 差速 + Set_PWM. */
        if (xunji_flag >= 1) {
            Track_StateMachine(countL, countR);
        } else {
            Set_PWM(0, 0);
        }
```

**注意:** `Set_PWM(PWMA, PWMB)` 参数顺序在状态机内已按 `Set_PWM(右, 左)` 正确处理(见 Task 3),ISR 这里不再调 `Set_PWM`。

- [ ] **Step 2: 确认过弯保护/变速逻辑不动**

ISR 后半段(`baohu_flag`/`biansu_flag`/`yizhi_flag`,约 193~228 行)保持不变。状态机里 `m0++` 与 `baohu_flag=1` 已与原 `Track_GetLostCrossed` 机制兼容。

- [ ] **Step 3: 确认 main loop 过弯检测不重复计 m0**

`empty.c` main loop 里(约 119~125 行)原有:
```c
        if (baohu_flag == 0 && Track_GetLostCrossed()) {
            Track_ClearLostCrossed();
            m0++;
            baohu_flag = 1;
            if (keyquan != 0) biansu_flag = 1;
        }
```
状态机转向态已自行 `m0++` 并置 `baohu_flag=1`,此处会因 `baohu_flag==1` 而跳过,不重复。但 `Track_GetLostCrossed` 仍可能被 track.c 内部置位。为避免双重计数,把这段改为只触发变速,不再 `m0++`:

```c
        /* 过弯后变速 (m0 已由状态机转向态递增, 这里只触发变速) */
        if (baohu_flag == 0 && Track_GetLostCrossed()) {
            Track_ClearLostCrossed();
            baohu_flag = 1;
            if (keyquan != 0) biansu_flag = 1;
        }
```

- [ ] **Step 4: 编译验证**

Build 工程。
Expected: 无 error。`xunji_pid`/`track_diff`/`lost_flag` 不再被 ISR 引用(但定义保留,可能有 unused warning,可接受或加 `(void)` 抑制)。

- [ ] **Step 5: 上车验证 — 跑直**

烧录。车放直线赛道(或平地有一直线黑线)。BLS 启动。
Expected:
- 车应沿当前航向**走直**(不画龙、不偏)。
- 若车向一侧持续偏或修正方向反 → 把 `STEER_SIGN` 改 `-1`(work.h)重烧。
- 若走直但抖 → 调小 `ANGLE_KD`(pid.h)。
- OLED 显示 yaw 应稳定(漂移 < 1°/s)。

- [ ] **Step 6: 上车验证 — 过弯**

车放矩形赛道直线段末端,前方有 90° 弯。
Expected:
- 走到丢线(全白)→ 进转向态原地转 → 转满 90° 或重新压线 → 回直线态继续走直。
- `m0` 每过一个弯 +1,OLED `Cross:` 计数递增。
- 若转向方向反(应左转却右转)→ 检查 `Track_GetLastDirection` 与 `turn_dir` 符号;`turn_dir*TURN_ANGLE` 符号需匹配陀螺仪 yaw 正向(上车试,必要时把 `turn_dir * TURN_ANGLE` 改为 `-turn_dir * TURN_ANGLE`)。

- [ ] **Step 7: 记录完成**

勾选,记录「ISR 已接入状态机,跑直 + 过弯上车验证通过(或记录待调参数)」。

---

### Task 5: 启动时锁航向 + OLED 显示状态(key.c / empty.c)

**Files:**
- Modify: `Hardware/key.c:80-97`(BLS 启动分支)
- Modify: `empty.c` OLED 运行画面(约 100~115 行)

**Interfaces:**
- Consumes: `Track_LockYaw`/`Angle_PID_Reset`(Task 1/3)、`track_state`(Task 3)。
- Produces: 启动瞬间锁定当前 yaw 为 `lock_yaw`;OLED 显示 yaw/状态。

- [ ] **Step 1: key.c 启动分支加锁航向**

在 `Hardware/key.c` 的「待机 → 启动」分支(约 81~97 行),在 `kaishi_flag = 1;` 之前、`base_speed = 5;` 之前追加锁航向。定位:

```c
                Vel_PI_Reset_A();
                Vel_PI_Reset_B();
                m0 = 0;
```

在其后(`baohu_flag = 0; ...` 那行之前或之后均可,确保在 `kaishi_flag=1` 之前)追加:

```c
                /* 角度环: 锁定当前航向, 进入直线态跑直 */
                Angle_PID_Reset();
                Track_LockYaw();
                xunji_flag = 1;   /* 使能状态机 (renwu 也会设, 这里提前确保) */
```

需在 `key.c` 顶部 include 区确认有 `#include "pid.h"`(已有,第7行)和 `#include "work.h"`(已有,第5行)。`Track_LockYaw` 声明在 work.h,已包含。

- [ ] **Step 2: empty.c 停止分支复位状态机**

在 `empty.c` 的 `Timer_work()`(work.c)已处理 `Motor.button==0` 清零。但状态机变量需在停止时复位。在 `key.c`「运行 → 停止」分支(约 71~79 行)的 `Car_Move(0,0);` 之后追加:

```c
                /* 停车: 状态机回直线态, 清转向目标 */
                track_state = TRACK_STRAIGHT;
                turn_dir = 0;
                Angle_PID_Reset();
```

`track_state`/`turn_dir` 声明在 work.h(已 include)。

- [ ] **Step 3: empty.c OLED 运行画面显示 yaw + 状态**

在 `empty.c` 运行画面(约 100~115 行,`cur_mode == 1` 分支)已显示 Yaw。在 `Cross:` 行下方加一行状态。定位:

```c
                OLED_ShowString(0, 32, (uint8_t*)"Cross: /        ");
                OLED_ShowNumber(40, 32, m0, 2, 16);
                OLED_ShowNumber(72, 32, quanshu * 4, 2, 16);
                OLED_ShowString(0, 48, (uint8_t*)"                ");
```

把第 4 行(48 行)改为显示状态:

```c
                OLED_ShowString(0, 32, (uint8_t*)"Cross: /        ");
                OLED_ShowNumber(40, 32, m0, 2, 16);
                OLED_ShowNumber(72, 32, quanshu * 4, 2, 16);
                /* 第4行: 状态机当前态 (S=直线 T=转向) + lock_yaw */
                OLED_ShowString(0, 48, (uint8_t*)"St:   Lk:       ");
                OLED_ShowChar(24, 48, (track_state == TRACK_TURN) ? 'T' : 'S', 12, 1);
                {
                    int lk = (int)lock_yaw;
                    OLED_ShowNumber(56, 48, (uint32_t)(lk < 0 ? -lk : lk), 3, 16);
                    if (lk < 0) OLED_ShowChar(48, 48, '-', 12, 1);
                }
```

- [ ] **Step 4: 编译验证**

Build 工程。
Expected: 无 error。

- [ ] **Step 5: 上车验证 — 启动锁航向 + 显示**

烧录。待机时手把车摆正。BLS 启动。
Expected:
- OLED 第 4 行显示 `St:S Lk:xxx`(xxx = 启动瞬间 yaw)。
- 启动后车按该 yaw 走直。
- 过弯时 `St:` 变 `T`,回直线态变 `S`,`Lk:` 更新为新航向。

- [ ] **Step 6: 记录完成**

勾选,记录「启动锁航向 + OLED 状态显示正常」。

---

### Task 6: 全量上车验证与参数整定

**Files:** 无代码改动(仅参数微调,改 work.h / pid.h 宏)

- [ ] **Step 1: 完整圈数测试**

设 `quanshu=1`(USER 短按),中速档(`keyquan=2`)。BLS 启动,放矩形赛道。
Expected: 跑完 4 个弯(4×90°)后自动停车,`m0` 到 4。

- [ ] **Step 2: 参数整定清单(按现象调)**

| 现象 | 调整 |
|------|------|
| 走直但画龙/抖 | `ANGLE_KD` 调小(80→3 改为 80→1),或 `ANGLE_KP` 调小 |
| 走直但稳态偏一侧 | `STEER_SIGN` 取反(work.h) |
| 转向转过头/跳过 | `TURN_PWM` 调小(2000→1500),`TURN_DEADBAND` 调大(5→8) |
| 转向方向反 | `turn_dir * TURN_ANGLE` 改 `-turn_dir * TURN_ANGLE`(work.c `Track_StateMachine` 转向态目标计算) |
| yaw 漂移大带偏 | 陀螺仪静止校零(Gyro_SendBiasCal),或 `ANGLE_KI` 调小(0.5→0.1) |
| 速度环跟不上 | `base_speed` 档位或 `VEL_PI_KP/KI`(encoder.c)调,但本计划不动 encoder.c |

- [ ] **Step 3: 多圈测试**

`quanshu=2` 或 `3`,FAST 档(`keyquan=0`)。验证多圈稳定性 + 渐进加速(`yizhi_flag`)仍生效。

- [ ] **Step 4: 记录最终参数**

把上车定下的 `STEER_SIGN`/`TURN_PWM`/`TURN_DEADBAND`/`ANGLE_KP/KI/KD` 写回 work.h / pid.h,在 spec 末尾记录实测值。

- [ ] **Step 5: 记录完成**

勾选,记录「全量圈数测试通过,参数已整定」。

---

## Self-Review

**1. Spec coverage:**
- 角度环 PID(KP=80/KI=0.5/KD=3/MAX=4500):Task 1 ✓
- `Track_HasLine`/方向记忆暴露:Task 2 ✓
- 三态状态机(直线态/转向态):Task 3 ✓
- 角度环叠加 PWM 层融合(`final_left = base_pwm - STEER_SIGN*steering`):Task 3 Step 2 ✓
- 转向态原地差速 + 双保险到位(90°/压线):Task 3 Step 2 ✓
- `m0++` 过弯计数:Task 3 Step 2 + Task 4 Step 3(去重)✓
- 启动锁 `lock_yaw`:Task 5 Step 1 ✓
- 出弯重锁 `lock_yaw`:Task 3 Step 2(`Track_LockYaw`)✓
- 圈数任务 `renwu()` 不变:Task 4 未动 renwu ✓
- OLED 显示 yaw/状态:Task 5 Step 3 ✓
- 不删旧代码:Task 1/3 保留 `SteeringPID`/`TurnErrorPid`/`xunji_pid` ✓
- 沿用 25diansai1 方向约定:Global Constraints + Task 3 注释 ✓

**2. Placeholder scan:** 无 TBD/TODO。所有代码块完整。参数有默认值 + Task 6 整定清单。✓

**3. Type consistency:**
- `Angle_Calculate(float, float, float)` → Task 3 调用 `Angle_Calculate(lock_yaw, yaw, 0.01f)` ✓
- `Track_HasLine(void)` 返回 uint8_t → Task 3 `if (!Track_HasLine())` ✓
- `Track_GetLastDirection(void)` 返回 int8_t → Task 3 `turn_dir = Track_GetLastDirection()` ✓
- `Track_StateMachine(int countL, int countR)` → Task 4 `Track_StateMachine(countL, countR)` ✓
- `Track_LockYaw(void)` → Task 5 `Track_LockYaw()` ✓
- `track_state`/`lock_yaw`/`turn_target_yaw`/`turn_dir` 类型在 work.h 声明与 work.c 定义一致(volatile uint8_t/float/float/int8_t)✓
- `Set_PWM(右, 左)`:Task 3 `Set_PWM(final_right, final_left)`,Task 3 转向态同,与 motor.c `Set_PWM(pwmA=右, pwmB=左)` 一致 ✓

无问题。
