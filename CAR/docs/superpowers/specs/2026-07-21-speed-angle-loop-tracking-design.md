# 25diansai1 循迹改为速度环 + 角度环 设计文档

日期: 2026-07-21
工程: `C:\Users\28442\Desktop\25diansai1\25diansai1`
参考: `C:\Users\28442\Desktop\2024QuestionH\2024QuestionH`(角度环实测能跑直)

## 1. 背景与目标

25diansai1 原循迹架构为「速度环 + 位置环」:
- 速度环 `Velocity_A/B`(encoder.c)输出 PWM
- 位置环 `xunji_pid()`(work.c)读 8 路循迹偏差 → `TurnErrorPid` → `track_diff`,作为速度环 target 的差速偏移(`run_L = base_speed - track_diff`,`run_R = base_speed + track_diff`)

问题:位置环用循迹偏差纠偏,丢线时靠 `±1000` 标记原地找线,跑直稳定性依赖循迹传感器,无航向锁。

目标:改为「速度环 + 角度环」,循迹时用陀螺仪 yaw 锁定航向跑直(复用 2024QuestionH 已验证跑直逻辑),过弯用按上次方向定点转 90° 的状态机。

## 2. 总体架构

三态状态机(定义在 work.c,在 TIMER_0 ISR 10ms 节拍内调用):

```
TRACK_STRAIGHT  →  (丢线)  →  TRACK_TURN  →  (转满90° 或 重新压线)  →  TRACK_STRAIGHT
   角度环锁 lock_yaw 跑直       原地差速按上次方向转 90°               重新锁定新 lock_yaw, m0++
```

- **TRACK_STRAIGHT(直线态)**:角度环锁 `lock_yaw`,速度环两轮同目标 `base_speed`,角度环输出 `steering` 叠加在 PWM 层做差速。循迹传感器只读不参与转向,仅用于检测丢线进入转向态。
- **TRACK_TURN(转向态)**:按 `s_last_direction`(-1 左/+1 右,来自 track.c 方向记忆)原地差速,直到 yaw 转满 90° 或重新压到黑线。到位后清 yaw 目标、重新锁 `lock_yaw`,回直线态,`m0++` 过弯计数。

## 3. 方向约定(沿用 25diansai1 自己的,不照搬 2024H)

25diansai1 实测约定:
- 左轮 = A 通道(`Encoder_CountA`、`Velocity_A`、`countL`、`Set_PWM` 的 `pwmB`)
- 右轮 = B 通道(`Encoder_CountB`、`Velocity_B`、`countR`、`Set_PWM` 的 `pwmA`)
- `Set_PWM(pwmA=右, pwmB=左)`
- 正 PWM = 前进
- 丢线左转:`run_L=-6, run_R=+6`(左轮反、右轮正 = 原地左转)✓ 与正 PWM 前进一致
- `Yaw()` 正向 = 陀螺仪本身约定(上车试方向时定 `STEER_SIGN`)

2024QuestionH 是 A=右、B=左且目标负值前进,与 25diansai1 相反,故 sign 必须用 25diansai1 自己的。

## 4. 模块改动清单

| 文件 | 改动 |
|------|------|
| `pid.c` / `pid.h` | 新增角度环 `Angle_Calculate(target, current, dt)` + `Angle_PID_Reset()`。参数复用 2024H 实测值 `KP=80, KI=0.5, KD=3, MAX_STEERING=4500`。旧 `SteeringPID` / `TurnErrorPid` 保留不删(可还原旧循迹) |
| `work.c` / `work.h` | 新增循迹状态机 `Track_StateMachine()`,替代原 `xunji_pid()` 的纠偏逻辑。新增全局:`track_state`、`lock_yaw`、`turn_target_yaw`、`turn_dir`。保留 `xunji_pid()` 旧函数体不删 |
| `track.c` / `track.h` | 不动核心加权逻辑。新增 `Track_HasLine()`(是否压到任意黑线)。保留 `s_last_direction` 方向记忆(供转向态用,新增 getter `Track_GetLastDirection()`) |
| `empty.c` | TIMER_0 ISR:直线态用角度环 `steering` 在 PWM 层做差速;转向态用固定小差速。移除 `track_diff` 参与速度环 target 的逻辑。OLED 显示新增 yaw/状态 |
| `key.c` | 启动时复位角度环积分 + 重锁 `lock_yaw`,其余不动 |

不动:motor.c、encoder.c、gyro_serial.c、bsp_*、oled.c、ti_msp_dl_config.*、pid.c 旧 `SteeringPID`/`TurnErrorPid`。

## 5. 角度环 + 速度环融合(角度环叠加在 PWM 层)

直线态每 10ms(在 TIMER_0 ISR 内):

```c
yaw = Yaw();
steering = Angle_Calculate(lock_yaw, yaw, 0.01f);  // 角度环输出, 限幅 ±4500
PWMA = Velocity_A(base_speed, countL);   // A=左轮, target=base_speed(两轮同速)
PWMB = Velocity_B(base_speed, countR);   // B=右轮
base_pwm = (PWMA + PWMB) / 2;            // 速度环输出的平均 PWM
final_left  = base_pwm - STEER_SIGN * steering;
final_right = base_pwm + STEER_SIGN * steering;
final_left  = limit_PWM(final_left,  -7999, 7999);
final_right = limit_PWM(final_right, -7999, 7999);
Set_PWM(final_right, final_left);       // Set_PWM(右,左)
```

转向态(原地差速,按上次方向):

```c
if (turn_dir < 0) { /* 左转: 左退右进 */ final_left = -TURN_PWM; final_right = +TURN_PWM; }
else              { /* 右转: 左进右退 */ final_left = +TURN_PWM; final_right = -TURN_PWM; }
Set_PWM(final_right, final_left);
// 到位: |normalize_angle(turn_target_yaw - yaw)| < TURN_DEADBAND  或  Track_HasLine()
```

## 6. 上车可调参数(默认值写好,实车调)

| 参数 | 默认 | 说明 |
|------|------|------|
| `STEER_SIGN` | +1 | 角度环差速符号;实车若修正方向反了改 -1 |
| `ANGLE_KP/KI/KD` | 80 / 0.5 / 3 | 角度环 PID(复用 2024H 实测值) |
| `MAX_STEERING` | 4500 | 角度环输出限幅 |
| `TURN_PWM` | 2000 | 转向态原地差速 PWM |
| `TURN_DEADBAND` | 5.0 | 转向到位 yaw 误差死区(度) |
| `base_speed` | 5/10/20 | 速度环目标(快/中/慢档,沿用原 keyquan) |

## 7. 过弯计数与圈数任务

- 进入转向态:`turn_dir = Track_GetLastDirection()`,`turn_target_yaw = normalize_angle(lock_yaw + turn_dir*90)`,`Angle_PID_Reset()`。
- 转向态到位:`|normalize_angle(turn_target_yaw - yaw)| < TURN_DEADBAND` **或** `Track_HasLine()`(双保险,后者防转过头)。
- 到位后:`lock_yaw = yaw`,`track_state = TRACK_STRAIGHT`,`m0++`,`Angle_PID_Reset()`。
- 圈数任务 `renwu()` 不变:`m0 <= quanshu*4` 继续跑,到圈数停车。

## 8. 锁定航向时机

- 启动(BLS 按下):`lock_yaw = Yaw()`,`track_state = TRACK_STRAIGHT`,`Angle_PID_Reset()`。
- 每次从转向态回直线态:`lock_yaw = Yaw()`(重新锁当前航向)。
- 直线态全程不再更新 `lock_yaw`(除非进/出转向态),保证跑直稳定。

## 9. 待确认/风险

- `STEER_SIGN` 方向需上车试(唯一方向参数)。
- 陀螺仪 `Yaw()` 漂移:跑直靠 yaw 锁定,若陀螺仪漂移大会带偏。2024H 用 MPU6050 DMP,25diansai1 用串口单轴陀螺仪(0x5A),需确认静止时 yaw 稳定。若漂移大,角度环 KI 可调小或加静止校零。
- 转向态原地差速 `TURN_PWM` 过大可能跳过 90°,用 `TURN_DEADBAND` + `Track_HasLine()` 双保险。
