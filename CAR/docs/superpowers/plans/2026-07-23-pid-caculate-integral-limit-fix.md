# PID_caculate 积分限幅修复 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修复 `PID_caculate` 积分限幅限错全局实例的 bug,给 `tPid` 加 `I_min/I_max` 字段,使速度环明天可用。

**Architecture:** `tPid` 结构体加 `I_min/I_max` 两字段;`PID_caculate` 把 `I_limit` 的对象从写死的 `EncoderLPid/RPid` 改成传入的 `pid` 自身,范围取 `pid->I_min/I_max`;`Pid_Init` 给三个实例分别赋限幅值(速度环 0~5000,转向环 ±3000)。

**Tech Stack:** MSPM0G3507, Keil MDK, C99, 无自动化测试(编译通过 + 明天实测)。

## Global Constraints

- 目标 MCU: MSPM0G3507, Keil 工程, DriverLib。
- `tPid` 结构体定义在 `pid.h`,实例 `EncoderLPid/EncoderRPid/TurnErrorPid` 声明在 `pid.h`、定义在 `pid.c`。
- `PID_caculate` 当前 0 实际调用点(main.c 里是注释示例),改签名/字段不影响既有调用。
- 封装工程无自动化测试框架,验证靠 Rebuild 通过 + 明天烧录实测速度环。
- 不动 `I_limit` 函数本身(逻辑正确)、不动 `Velocity_A/B`(encoder.c 备用实现)、不动 main.c 注释示例。

---

### Task 1: tPid 结构体加 I_min/I_max 字段

**Files:**
- Modify: `Hardware/pid.h:10-18`(`tPid` 结构体定义)

**Interfaces:**
- Produces: `tPid` 结构体新增 `double I_min, I_max` 字段,供 Task 2 的 `PID_caculate` 和 Task 3 的 `Pid_Init` 使用。

- [ ] **Step 1: 在 tPid 结构体末尾加 I_min/I_max 字段**

定位 `Hardware/pid.h` 中:
```c
typedef struct {
    double Kp, Ki, Kd;      /* 比例, 积分, 微分系数 */
    double target_val;     /* 目标值 */
    double actual_val;     /* 实际值 */
    double err;            /* 当前偏差 */
    double err_last;       /* 上次偏差 */
    double err_sum;        /* 误差累计值 */
    double output;         /* 输出 */
} tPid;
```
替换为:
```c
typedef struct {
    double Kp, Ki, Kd;      /* 比例, 积分, 微分系数 */
    double target_val;     /* 目标值 */
    double actual_val;     /* 实际值 */
    double err;            /* 当前偏差 */
    double err_last;       /* 上次偏差 */
    double err_sum;        /* 误差累计值 */
    double output;         /* 输出 */
    double I_min, I_max;   /* 积分限幅范围 */
} tPid;
```

- [ ] **Step 2: Rebuild 验证编译通过**

在 Keil 中 Rebuild。预期:0 error(加字段不影响既有代码,因 PID_caculate 之前无实际调用,新字段未初始化也暂不报错)。

---

### Task 2: PID_caculate 限传入实例自身

**Files:**
- Modify: `Hardware/pid.c:27-46`(`PID_caculate` 函数)

**Interfaces:**
- Consumes: Task 1 的 `tPid.I_min/I_max` 字段;现有 `I_limit(tPid *pid, double low, double high)`(pid.c 第 48 行)。
- Produces: 修正后的 `PID_caculate`,限传入 `pid` 自身积分,范围取 `pid->I_min/I_max`。

- [ ] **Step 1: 替换 PID_caculate 中限幅写死的两行**

定位 `Hardware/pid.c` 中:
```c
void PID_caculate(tPid *pid, double actual_val, double target_val)
{
    pid->actual_val = actual_val;
    pid->target_val = target_val;

    pid->err = pid->target_val - pid->actual_val;   /* 当前误差 = 目标 - 实际 */

    pid->err_sum += pid->err;                        /* 误差累计 */

    /* 速度环积分限幅 0~5000 (禁止负向积分, 降速只靠比例项) */
    I_limit(&EncoderLPid, 0, 5000);
    I_limit(&EncoderRPid, 0, 5000);

    /* 输出 = Kp*当前误差 + Ki*误差累计 + Kd*(当前误差 - 上次误差) */
    pid->output = pid->Kp * pid->err
                + pid->Ki * pid->err_sum
                + pid->Kd * (pid->err - pid->err_last);

    pid->err_last = pid->err;   /* 保存上次误差 */
}
```
替换为:
```c
void PID_caculate(tPid *pid, double actual_val, double target_val)
{
    pid->actual_val = actual_val;
    pid->target_val = target_val;

    pid->err = pid->target_val - pid->actual_val;   /* 当前误差 = 目标 - 实际 */

    pid->err_sum += pid->err;                        /* 误差累计 */

    /* 积分限幅: 限传入实例自身, 范围由实例的 I_min/I_max 决定 */
    I_limit(pid, pid->I_min, pid->I_max);

    /* 输出 = Kp*当前误差 + Ki*误差累计 + Kd*(当前误差 - 上次误差) */
    pid->output = pid->Kp * pid->err
                + pid->Ki * pid->err_sum
                + pid->Kd * (pid->err - pid->err_last);

    pid->err_last = pid->err;   /* 保存上次误差 */
}
```

- [ ] **Step 2: Rebuild 验证编译通过**

在 Keil 中 Rebuild。预期:0 error。

---

### Task 3: Pid_Init 给实例设 I_min/I_max

**Files:**
- Modify: `Hardware/pid.c:19-25`(`Pid_Init` 函数)

**Interfaces:**
- Consumes: Task 1 的 `tPid.I_min/I_max` 字段;全局实例 `EncoderLPid/EncoderRPid/TurnErrorPid`(pid.c 第 12 行附近定义)。
- Produces: 三个实例初始化时带积分限幅值,供 Task 2 的 `PID_caculate` 使用。

- [ ] **Step 1: 替换 Pid_Init, 给每个实例赋 I_min/I_max**

定位 `Hardware/pid.c` 中:
```c
void Pid_Init(void)
{
    EncoderLPid.Kp = 5;  EncoderLPid.Ki = 10; EncoderLPid.Kd = 0;
    EncoderRPid.Kp = 5;  EncoderRPid.Ki = 10; EncoderRPid.Kd = 0;

    TurnErrorPid.Kp = 5.2; TurnErrorPid.Ki = 0;  TurnErrorPid.Kd = 10;
}
```
替换为:
```c
void Pid_Init(void)
{
    EncoderLPid.Kp = 5;  EncoderLPid.Ki = 10; EncoderLPid.Kd = 0;
    EncoderLPid.I_min = 0;    EncoderLPid.I_max = 5000;   /* 速度环: 禁止负向积分 */

    EncoderRPid.Kp = 5;  EncoderRPid.Ki = 10; EncoderRPid.Kd = 0;
    EncoderRPid.I_min = 0;    EncoderRPid.I_max = 5000;

    TurnErrorPid.Kp = 5.2; TurnErrorPid.Ki = 0;  TurnErrorPid.Kd = 10;
    TurnErrorPid.I_min = -3000; TurnErrorPid.I_max = 3000;  /* 转向环: 双向积分 */
}
```

- [ ] **Step 2: Rebuild 验证编译通过**

在 Keil 中 Rebuild。预期:0 error。

---

### Task 4: 烧录实测速度环（明天手动）

**Files:**
- 无文件改动,硬件验证。

**Interfaces:**
- Consumes: Task 1–3 全部成果;`Get_Encoder_countA/B`(encoder.h)、`Set_PWM`(motor.h)。

- [ ] **Step 1: 在 main.c 或定时器中断里写速度环调用**

在 `TIMER_0_INST_IRQHandler`(10ms 节拍)或 main while(1) 内,对左右轮分别闭环:
```c
int cntL = Get_Encoder_countA();   /* 左轮脉冲增量 */
int cntR = Get_Encoder_countB();   /* 右轮脉冲增量 */
PID_caculate(&EncoderLPid, cntL, targetL);   /* targetL = 目标脉冲数, 需空载实测整定 */
PID_caculate(&EncoderRPid, cntR, targetR);
Set_PWM((int)EncoderLPid.output, (int)EncoderRPid.output);  /* A=左, B=右 */
```
注:`targetL/targetR` 是编码器计数目标,首次上电需空载实测 50ms(或本工程 ENCODER_SAMPLE_MS)脉冲数后整定。

- [ ] **Step 2: 烧录观察左右轮稳速**

烧录运行,预期:左右轮按目标速度稳定转动,积分不饱和(长时间运行不会越来越快或反向),左右轮互不干扰(调左轮不影响右轮积分)。

- [ ] **Step 3: 若左右轮方向反, 按 motor.c 修复指南排查**

若 `Set_PWM(outL, outR)` 导致左右轮转向异常,回顾 motor.c 的 `Car_Move`/`Set_PWM` 修复(A=左/B=右),确认参数顺序。若编码器计数一正一负(物理镜像),在 encoder.c 中断里对调该轮 `++`/`--`。

---

## Self-Review

**1. Spec coverage:**
- tPid 加 I_min/I_max 字段 → Task 1 ✓
- PID_caculate 限传入 pid 自身 → Task 2 ✓
- Pid_Init 给三实例赋限幅(速度环 0~5000,转向环 ±3000)→ Task 3 ✓
- I_limit 不动 → 未列为任务(正确,无需动作)✓
- Velocity_A/B 不动 → 未列为任务(正确)✓
- main.c 注释示例可照用 → Task 4 Step 1 照用 ✓
- 验证 Rebuild + 明天实测 → Task 1-3 各 Step 2 + Task 4 ✓

**2. Placeholder scan:** 无 TBD/TODO/省略,所有代码块完整,targetL/targetR 在 Task 4 注明需实测整定(非占位,是硬件事实)。✓

**3. Type consistency:** `I_min/I_max` 为 `double`(Task 1 定义),Task 2 用 `pid->I_min/I_max` 传给 `I_limit(tPid*, double, double)`(pid.c 第 48 行,参数 double)类型一致;Task 3 赋值 `0/5000/-3000/3000` 为 int 字面量隐式转 double,一致。`EncoderLPid/RPid/TurnErrorPid` 三个实例名在 Task 3(赋值)与 Task 4(调用)一致。✓
