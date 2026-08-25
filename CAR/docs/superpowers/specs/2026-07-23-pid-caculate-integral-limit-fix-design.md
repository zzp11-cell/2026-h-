# PID_caculate 积分限幅修复 Design

## 背景

`pid.c` 的 `PID_caculate(tPid *pid, double actual_val, double target_val)` 是通用位置式 PID 函数,明天用于做速度环(左右轮编码器闭环)。当前实现有 bug:积分限幅对象错误。

## 问题

第 37-38 行:

```c
pid->err_sum += pid->err;
I_limit(&EncoderLPid, 0, 5000);   /* 限的是全局 EncoderLPid, 不是传入的 pid */
I_limit(&EncoderRPid, 0, 5000);
```

- 函数签名接收 `tPid *pid`,但内部去限幅全局的 `EncoderLPid/RPid` 实例,而非传入的 `pid`。语义错乱。
- 调用 `PID_caculate(&EncoderRPid, ...)` 时,限的是 EncoderLPid(无关实例),EncoderRPid 自身积分反而不限。
- 调用 `PID_caculate(&TurnErrorPid, ...)` 时,TurnErrorPid 的积分完全不限幅(会饱和),且无端限了 EncoderL/R。
- 限幅范围 `0~5000` 写死,只适合速度环(目标恒正),转向环需双向积分时无法复用。

## 方案

### 1. tPid 结构体加积分限幅字段

`pid.h` 的 `tPid` 新增 `I_min`/`I_max` 两个字段:

```c
typedef struct {
    double Kp, Ki, Kd;
    double target_val, actual_val;
    double err, err_last, err_sum;
    double output;
    double I_min, I_max;   /* 积分限幅范围 (新增) */
} tPid;
```

### 2. PID_caculate 限传入实例自身

```c
void PID_caculate(tPid *pid, double actual_val, double target_val)
{
    pid->actual_val = actual_val;
    pid->target_val = target_val;
    pid->err = pid->target_val - pid->actual_val;
    pid->err_sum += pid->err;
    I_limit(pid, pid->I_min, pid->I_max);   /* 限传入实例自身 */
    pid->output = pid->Kp * pid->err
                + pid->Ki * pid->err_sum
                + pid->Kd * (pid->err - pid->err_last);
    pid->err_last = pid->err;
}
```

### 3. Pid_Init 给每个实例设限幅

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

## 不改的部分

- `I_limit` 函数本身逻辑正确(只是之前调用方式错),不动。
- main.c 注释里的调用示例 `PID_caculate(&EncoderLPid, cntL, targetL)` 现在语义正确,可照用。
- `Velocity_A/B`(encoder.c 里的纯 P 速度环)不动,那是另一套备用实现。

## 验证

- Rebuild 通过(tPid 加字段不影响其他,因为 PID_caculate 之前是死代码,无既有调用依赖旧签名)。
- 明天实测:左右轮分别用 `PID_caculate(&EncoderLPid, cntL, targetL)` 闭环,目标速度恒定时轮速稳定、积分不饱和、左右不互相干扰。

## 影响范围

- 改 `pid.h`:`tPid` 结构体加 2 字段。
- 改 `pid.c`:`PID_caculate` 2 行(限幅对象)、`Pid_Init` 加 3 实例的 I_min/I_max 赋值。
- 无调用点需改(之前 0 实际调用)。
