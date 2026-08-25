#ifndef _TRACK_H
#define _TRACK_H

/*
 * ============================================================
 * 7 路循迹传感器驱动 (active high, 黑=1高电平, 白=0低电平)
 * ============================================================
 *  通道布局 (从左到右, 7 路对称):
 *    CH0(L3)  CH1(L2)  CH2(L1)  CH3(M0)  CH4(R1)  CH5(R2)  CH6(R3)
 *    -3       -2       -1        0        +1       +2       +3
 *
 *  引脚 (用户暂未接线, 接硬件时只改这里; CH7 为 8 路时代扩展位, 7 路不接不读):
 *    CH0 = PA27  (待确认)
 *    CH1 = PA12  (待确认)
 *    CH2 = PB16  (待确认)
 *    CH3 = PB17  (待确认)
 *    CH4 = PA24  (待确认)
 *    CH5 = PA9   (待确认)
 *    CH6 = PA22  (待确认)
 *    CH7 = 保留占位, 7 路模式不读取 (bit7 恒 0)
 *
 *  返回掩码: bit0=CH0(L3最左) ... bit6=CH6(R3最右), bit7 恒 0
 *  active high: 传感器读到 1(高电平=黑线) → 该位置 1
 *
 *  注意: 7 路时 Track_GetState() 不读 CH7, bit7 始终为 0。
 *        car 的查表 case 值仍可直接用 (bit7=0 的状态天然落在查表覆盖范围内)。
 * ============================================================
 */
#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ---- 7 路引脚定义 (syscfg TRACK 组, 直接用 ti_msp_dl_config.h 生成的宏) ----
 * track.c 里用 TRACK_CHx_PORT + TRACK_CHx_PIN 读取 (config.h 已生成, 勿在此重定义).
 * 实际接线 (syscfg):
 *   CH0: PB24 (L3 最左)    CH1: PB25 (L2)    CH2: PB20 (L1)
 *   CH3: PA14 (M0 中)      CH4: PA16 (R1)    CH5: PB17 (R2)
 *   CH6: PB19 (R3 最右)
 * active-high: 黑=1(高电平), 白=0. PULL_DOWN 配置, 若传感器 active-low 改 PULL_UP + 翻 readPins 判定. */



/* CH7: 保留占位, 7 路模式不读取 (bit7 恒 0) */
/* 如以后扩到 8 路, 在此定义 TRACK_CH7_PORT/PIN 并在 Track_GetState() 取消注释 */


/* ---- 丢线特殊标记 (移植 work.c get_sensor_actual) ---- */
#define TRACK_LOST_LEFT   (-1000.0f)   /* 丢线且线曾在左 → 原地左转 */
#define TRACK_LOST_RIGHT  ( 1000.0f)   /* 丢线且线曾在右 → 原地右转 */

/*
 * 读取 8 路传感器, 返回 8 位掩码
 *   bit0 = CH0(L3), bit1 = CH1(L2), ... bit7 = CH7(扩展)
 *   active high: 读到 1(压黑线) → 对应 bit 置 1
 */
uint8_t Track_GetState(void);

/*
 * 计算 8 路加权偏差
 *   正值 = 线在右(需右转), 负值 = 线在左(需左转), 0 = 正中
 *   权重: CH0=-3, CH1=-2, CH2=-1, CH3=0, CH4=+1, CH5=+2, CH6=+3, CH7=扩展
 *   全白丢线: 返回 TRACK_LOST_LEFT / TRACK_LOST_RIGHT (按上次方向)
 *   十字(左4路或右4路全黑): 返回 0 直行通过
 */
float Track_GetError(void);

/* 丢线过弯标志访问 (供任务层过弯计数用) */
uint8_t Track_GetLostCrossed(void);
void    Track_ClearLostCrossed(void);

/* 是否压到任意黑线 (state != 0). 供角度环状态机判断"重新找到线"用 */
uint8_t Track_HasLine(void);

/* 上次有效方向: -1=线曾在左, +1=线曾在右, 0=未知. 供转向态定转向方向 */
int8_t Track_GetLastDirection(void);


/* ============================================================
 *  循迹转向环 (位置式 PID, 输出 base+diff 脉冲, 叠到速度环 target)
 *  替换角度环位置: error=Track_GetError() → base/diff
 * ============================================================
 *  量纲: error±3 (7路加权), 输出 base/diff 脉冲数/10ms
 *  error>0 线在右 → 右转 → 左快右慢 (target_L=base-diff, target_R=base+diff)
 *  丢线(TRACK_LOST_LEFT/RIGHT): base=0, diff=±rotate → 原地转
 *  十字(error=0): base=base_pulse, diff=0 直行
 *  参数: Kp≈1.0, Kd≈0.5, base_pulse=5, diff限幅±3, rotate_pulse=3
 * ============================================================ */
#define TRACK_BASE_PULSE   8       /* 循迹基础速度 (脉冲/10ms) — 保留兼容 */
#define TRACK_BASE_FAST   15       /* 直线高速 */
#define TRACK_BASE_SLOW   8        /* 弯道低速 */
#define TRACK_DIFF_MAX    15       /* 差速限幅 (脉冲) */
#define TRACK_ROTATE_PULSE 5      /* 丢线/过弯原地转 速度 */

extern float TRACK_KP;   /* 循迹转向环 P (ISR 按任务切换) */
extern float TRACK_KD;   /* 循迹转向环 D (ISR 按任务切换) */
extern float TRACK_KP_T1; extern float TRACK_KD_T1;  /* 任务1 专用 */
extern float TRACK_KP_T2; extern float TRACK_KD_T2;  /* 任务2/3 共用 */
void Track_Steering_Compute(float error, int *base, int *diff);
/*   error=Track_GetError() 返回值 (含 TRACK_LOST_LEFT/RIGHT)
 *   *base=基础速度, *diff=差速 (正=右转左快右慢) */

/* ============================================================
 *  弯道检测 + 过弯 (active-high, 参考 WHEELTEC 例程 TurnDetector/HandleTurn)
 * ============================================================
 *  Track_DetectTurn: 7路 state → 弯道类型
 *    十字: state==0x7F (全亮)
 *    左直角: (state&0x07)==0x07 && (state&0x78)!=0 (CH0~2全亮+右侧有亮)
 *    右直角: (state&0x78)==0x78 && (state&0x07)!=0 (CH4~6全亮+左侧有亮)
 *    否则 TURN_NONE
 *  Track_HandleTurn: 非阻塞两阶段过弯 (直行200ms+旋转500ms), 后驱对称±rotate
 *    返回1=过弯接管 target_L/R 已设; 返回0=交给转向环
 * ============================================================ */
typedef enum {
    TURN_NONE = 0, TURN_LEFT_90, TURN_RIGHT_90, TURN_CROSS
} TurnType;

TurnType Track_DetectTurn(uint8_t state);
uint8_t  Track_HandleTurn(int *target_L, int *target_R);   /* 返回1=接管 */
TurnType  Track_GetCurrentTurn(void);   /* 供 OLED 显示当前弯道类型 */
void Track_ResetTurnState(void);

/* ============================================================
 * 查表法循迹 (备用, 与加权法 Track_GetError 共存, 按场景选用)
 *   7 路: Track_Adjust / Track_err  (bit0=CH0最左...bit6=CH6最右)
 *   4 路: FourTrack_Adjust / FourTrack_err (只用中间 4 路 CH2~CH5)
 *   偏差约定: 偏左为负(左轮减速), 偏右为正(右轮减速)
 * ============================================================ */
extern int track_state;
extern int track_stop_state;       /* 巡线没亮多少 ms 就跳出, 默认 250 */

void Track_Adjust(void);           /* 7 路查表直出左右轮速度 (Car_Move) */
int  Track_err(void);              /* 7 路查表偏差: 偏左-, 偏右+ */
void FourTrack_Adjust(void);       /* 4 路查表直出速度 */
int  FourTrack_err(void);          /* 4 路查表偏差 */


#endif /* _TRACK_H */
