#include "track.h"

/*
 * 7 路循迹: 读取 + 加权偏差 + 丢线原地转向标记
 
 * 约定:
 *   active high — 传感器黑=1(高电平), 白=0(低电平), 压到黑线对应 bit 置 1
 *   正偏差 = 线偏右(需右转), 负偏差 = 线偏左(需左转)
 */

/* 7 路权重, 与 track.h 通道顺序一致: CH0..CH6
 * 7 路对称布局: CH0=-3, CH1=-2, CH2=-1, CH3=0(中间), CH4=+1, CH5=+2, CH6=+3
 * 直线时 CH3(中)亮 或 CH2/CH4 对称亮 → bias≈0 直行 */
static const float track_weight[7] = {
    -3.0f, -2.0f, -0.5f,  0.0f,   // CH0~CH3  左半 + 中
     0.5f,  2.0f,  3.0f            // CH4~CH6  右半
};

/* 丢线方向记忆: -1=线曾在左, +1=线曾在右, 0=未知 */
static int8_t  s_last_direction = 0;
/* 上次有效偏差 (未知方向时保持) */
static float   s_last_valid_bias = 0.0f;
/* 连续丢线计数 */
static uint16_t s_lost_cnt = 0;
/* 丢线过弯标记: 持续丢线超过阈值置 1, 供任务层过弯计数 */
static uint8_t  s_lost_crossed = 0;

uint8_t Track_GetState(void)
{
    uint8_t state = 0;

    /* active high: 传感器黑=1(高电平), 白=0(低电平)
     * readPins 返回非 0 表示高电平(黑线) → 置 1
     * 7 路只读 CH0~CH6, bit7(CH7) 恒为 0 */
    if ((DL_GPIO_readPins(TRACK_CH0_PORT, TRACK_CH0_PIN)) != 0) state |= (1 << 0);
    if ((DL_GPIO_readPins(TRACK_CH1_PORT, TRACK_CH1_PIN)) != 0) state |= (1 << 1);
    if ((DL_GPIO_readPins(TRACK_CH2_PORT, TRACK_CH2_PIN)) != 0) state |= (1 << 2);
    if ((DL_GPIO_readPins(TRACK_CH3_PORT, TRACK_CH3_PIN)) != 0) state |= (1 << 3);
    if ((DL_GPIO_readPins(TRACK_CH4_PORT, TRACK_CH4_PIN)) != 0) state |= (1 << 4);
    if ((DL_GPIO_readPins(TRACK_CH5_PORT, TRACK_CH5_PIN)) != 0) state |= (1 << 5);
    if ((DL_GPIO_readPins(TRACK_CH6_PORT, TRACK_CH6_PIN)) != 0) state |= (1 << 6);
    /* CH7 不接, 不读取, bit7 恒 0 */

    return state;
}

float Track_GetError(void)
{
    uint8_t state = Track_GetState();
    float numerator = 0.0f;
    float denominator = 0.0f;
    uint8_t i;

    /* 遍历 7 路 (bit0~bit6) */
    for (i = 0; i < 7; i++) {
        if (state & (1 << i)) {
            numerator += track_weight[i];
            denominator += 1.0f;
        }
    }

    /*
     * 十字路口 / 直角弯检测 (7 路):
     *   左转直角: CH0~CH4 全黑 (state & 0x1F == 0x1F)
     *   右转直角: CH2~CH6 全黑 (state & 0x7C == 0x7C)
     *   居中十字: CH1~CH5 全黑 (state & 0x3E == 0x3E)
     *   → bias=0 直行通过, 直行后短暂全白(丢线)按上次方向原地找线
     */
    if ((state & 0x1F) == 0x1F ||   /* CH0~CH4 全黑: 左转直角 */
        (state & 0x7C) == 0x7C ||   /* CH2~CH6 全黑: 右转直角 */
        (state & 0x3E) == 0x3E)     /* CH1~CH5 全黑: 居中十字 */
    {
        s_last_valid_bias = 0.0f;
        s_lost_cnt = 0;
        return 0.0f;
    }

    /* 有信号: 记录方向, 重置丢线计数, 返回加权偏差
     * bias>0 = 黑线偏右(CH4/5/6 权重正), bias<0 = 黑线偏左(CH0/1/2 权重负) */
    if (denominator > 0.0f) {
        s_lost_cnt = 0;
        float bias = numerator / denominator;
        if (bias > 0.0f)      s_last_direction = 1;   /* 正=线在右, 丢线时原地右转 */
        else if (bias < 0.0f) s_last_direction = -1;  /* 负=线在左, 丢线时原地左转 */
        s_last_valid_bias = bias;
        return bias;
    }

    /* 全白丢线 */
    s_lost_cnt++;
    if (s_lost_cnt > 10) s_lost_crossed = 1;   /* 持续丢线, 标记过弯 */

    /* 按上次方向原地旋转找线 */
    if (s_last_direction == -1) return TRACK_LOST_LEFT;    /* 线在左, 原地左转 */
    if (s_last_direction ==  1) return TRACK_LOST_RIGHT;    /* 线在右, 原地右转 */
    return s_last_valid_bias;                               /* 未知方向, 保持 */
}

uint8_t Track_GetLostCrossed(void)  { return s_lost_crossed; }
void    Track_ClearLostCrossed(void){ s_lost_crossed = 0; }

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


/* ============================================================ */
/*  循迹转向环 + 弯道检测 + 过弯 (2026-07-25 新增)              */
/* ============================================================ */

/* 转向 PID 参数 (error±3 → diff±3, Kp≈1起; 实测调) */
float TRACK_KP    = 2.6f;   /* 循迹 P (ISR 按任务切换写入) */
float TRACK_KD    = 4.0f;   /* 循迹 D */
float TRACK_KP_T1 = 3.7f;   /* T2 专用 P (VOFA P1 调) */
float TRACK_KD_T1 = 3.3f;   /* T2 专用 D (VOFA D1 调) */
float TRACK_KP_T2 = 1.1f;   /* T4/T5 共用 P (VOFA P3 调) */
float TRACK_KD_T2 = 2.0f;   /* T4/T5 共用 D (VOFA D3 调) */
static float s_track_err_last = 0.0f;   /* 微分用 */

/* 弯道状态 (过弯期间保持, hold_count 倒计时) */
static TurnType s_current_turn = TURN_NONE;
static uint16_t s_turn_hold = 0;        /* 过弯保持周期数 (10ms/周期) */
static uint32_t s_turn_start_ms = 0;    /* 过弯起始时间 (sys_tick_ms) */
static uint8_t  s_turn_active = 0;      /* 1=正在过弯 */
static uint8_t  s_turn_armed = 1;       /* 1=允许检测新弯道 */

#define TRACK_TURN_CLEAR_MASK 0x77u

/* 暴露当前弯道类型给 OLED */
TurnType Track_GetCurrentTurn(void) { return s_current_turn; }

static uint8_t Track_TurnPatternCleared(uint8_t state)
{
    if (state == 0u) return 1u;
    if ((state & TRACK_TURN_CLEAR_MASK) == 0u) return 1u;
    return (Track_DetectTurn(state) == TURN_NONE) ? 1u : 0u;
}

void Track_ResetTurnState(void)
{
    s_current_turn = TURN_NONE;
    s_turn_hold = 0;
    s_turn_start_ms = 0;
    s_turn_active = 0;
    s_turn_armed = 1;
}

/* 转向环: error → base + diff
 *   error>0 线在右 → 右转 → diff>0 (target_L=base-diff, target_R=base+diff, 左快右慢)
 *   丢线 (TRACK_LOST_LEFT/RIGHT): base=0, diff=±rotate 原地转
 *   十字/正常 (error 有值): base=BASE, diff=PID(error) 限幅 */
void Track_Steering_Compute(float error, int *base, int *diff)
{
    /* 丢线: 原地旋转找线 */
    if (error == TRACK_LOST_LEFT) {
        *base = 0;
        *diff = -TRACK_ROTATE_PULSE;   /* 线在左 → 原地左转 (左轮负右轮正) */
        return;
    }
    if (error == TRACK_LOST_RIGHT) {
        *base = 0;
        *diff = TRACK_ROTATE_PULSE;    /* 线在右 → 原地右转 */
        return;
    }

    /* 位置式 PD → diff. base 由调用方按里程计调速 */
    *base = TRACK_BASE_PULSE;
    float d = (error - s_track_err_last) * TRACK_KD;
    s_track_err_last = error;
    float out = TRACK_KP * error + d;
    int idiff = (int)out;
    if (idiff >  TRACK_DIFF_MAX) idiff =  TRACK_DIFF_MAX;
    if (idiff < -TRACK_DIFF_MAX) idiff = -TRACK_DIFF_MAX;
    *diff = idiff;
}

/* 弯道检测 (active-high: 黑=1, 全亮0x7F=十字) */
TurnType Track_DetectTurn(uint8_t state)
{
    /* 十字路口: 全亮 */
    if (state == 0x7F) return TURN_CROSS;
    /* 左直角: CH0~CH2 全亮 (0x07) 且右侧(CH3~CH6)有亮 → 线在左,要左转 */
    if ((state & 0x07) == 0x07 && (state & 0x78) != 0) return TURN_LEFT_90;
    /* 右直角: CH4~CH6 全亮 (0x70) 且左侧(CH0~CH3)有亮 → 线在右,要右转 */
    if ((state & 0x70) == 0x70 && (state & 0x0F) != 0) return TURN_RIGHT_90;
    return TURN_NONE;
}

/* 过弯执行 (非阻塞两阶段). 返回1=接管 target_L/R, 0=交给转向环.
 * 两阶段 (后驱对称 ±rotate):
 *   0~200ms: 直行 target_L=R=base (冲过弯道口)
 *   200~700ms: 旋转 左转 target_L=-rot, target_R=+rot; 右转反之
 *   到 700ms: 结束, 交回转向环 */
uint8_t Track_HandleTurn(int *target_L, int *target_R)
{
    extern volatile uint32_t sys_tick_ms;   /* main.c 10ms 节拍 */

    /* 不在过弯: 检测新弯道 */
    if (!s_turn_active) {
        uint8_t st = Track_GetState();
        if (!s_turn_armed) {
            if (Track_TurnPatternCleared(st)) {
                s_turn_armed = 1;
            } else {
                return 0;
            }
        }
        TurnType t = Track_DetectTurn(st);
        if (t == TURN_NONE) return 0;        /* 无弯道, 交回转向环 */
        s_current_turn = t;
        s_turn_active = 1;
        s_turn_armed = 0;
        s_turn_start_ms = sys_tick_ms;
        /* 十字不旋转, 直行通过即可 */
    }

    /* 过弯中: 按时间分阶段 */
    uint32_t elapsed = sys_tick_ms - s_turn_start_ms;

    if (s_current_turn == TURN_CROSS) {
        /* 十字: 直行 300ms 通过, 不旋转 */
        *target_L = TRACK_BASE_PULSE;
        *target_R = TRACK_BASE_PULSE;
        if (elapsed >= 300) {
            s_turn_active = 0;
            s_current_turn = TURN_NONE;
            return 0;
        }
        return 1;
    }

    /* 直角弯: 0~200ms 直行, 200~700ms 旋转 */
    if (elapsed < 200) {
        *target_L = TRACK_BASE_PULSE;
        *target_R = TRACK_BASE_PULSE;
    } else if (elapsed < 700) {
        int rot = TRACK_ROTATE_PULSE;
        if (s_current_turn == TURN_LEFT_90) {
            *target_L = -rot;   /* 左转: 左轮反右轮正 */
            *target_R =  rot;
        } else { /* TURN_RIGHT_90 */
            *target_L =  rot;   /* 右转: 左轮正右轮反 */
            *target_R = -rot;
        }
    } else {
        s_turn_active = 0;
        s_current_turn = TURN_NONE;
        return 0;   /* 过弯结束, 交回转向环 */
    }
    return 1;
}


/* ====================================================================== */
/*  以下为 2026-07-23 从 mspm0g3507_car/Track.c 补全的查表法循迹             */
/*  适配:                                                                  */
/*   - car 用 Set_Speed(左,右) → CAR 用 Car_Move(左,右) (方向一致)   */
/*   - 复用 CAR 的 Track_GetState() (bit=1=压黑线), case 数值与 car  */
/*     完全一致 (car 的 Track_Get_State 也是 bit=1=黑线, 高低电平差异在     */
/*     Track_GetState 内部已统一, 这里无需关心)                              */
/* ====================================================================== */

#include "motor.h"   /* Car_Move(左,右) */

int track_state;
int track_stop_state = 250;       /* 巡线没亮多少 ms 就跳出 */

/* ====================================================================== */
/*  7 路查表法循迹 (2026-07-23 按 7 路布局重写, 非 car 原 8 路表)          */
/*                                                                        */
/*  7 路布局 (LSB 在左, 与加权法 Track_GetState() 一致):                   */
/*    bit0=CH0(L3最左) bit1=CH1(L2) bit2=CH2(L1) bit3=CH3(M0中)           */
/*    bit4=CH4(R1) bit5=CH5(R2) bit6=CH6(R3最右)  bit7 恒 0               */
/*                                                                        */
/*  偏差约定: 偏左为负(左轮减速/右轮加速), 偏右为正(右轮减速/左轮加速)      */
/*  注意: car 原 8 路表是 MSB 在左(bit7=最左), 与本布局方向相反,           */
/*        故此处按 7 路布局重新列 case, 不直接搬 car 的表。                 */
/* ====================================================================== */

/* 7 路查表直出左右轮速度 (走黑线)
 * Car_Move(左轮, 右轮): 偏左→左轮减速右轮加速; 偏右→反之 */
void Track_Adjust(void)
{
    uint8_t state = Track_GetState();

    switch (state)
    {
        /* 居中直行: 中间 CH3 亮, 或 CH2+CH4 对称, 或 CH2/CH3/CH4 组合 */
        case 0x08:   /* 0001 000 中间单亮 */
        case 0x0C:   /* 0011 00 CH2+CH3 */
        case 0x18:   /* 011 000 CH3+CH4 */
        case 0x1C:   /* 0111 00 CH2+CH3+CH4 */
            Car_Move(20, 20);   break;

        /* 偏左一级: CH2 亮 */
        case 0x04:   /* 001 00 CH2 */
            Car_Move(15, 25);   break;

        /* 偏右一级: CH4 亮 */
        case 0x10:   /* 010 000 CH4 */
            Car_Move(25, 15);   break;

        /* 偏左二级: CH1 亮 / CH1+CH2 */
        case 0x02:   /* 000 0 10 CH1 */
        case 0x06:   /* 000 11 0 CH1+CH2 */
            Car_Move(10, 30);   break;

        /* 偏右二级: CH5 亮 / CH4+CH5 */
        case 0x20:   /* 100 000 CH5 */
        case 0x30:   /* 110 000 CH4+CH5 */
            Car_Move(30, 10);   break;

        /* 偏左三级: CH0 亮 / CH0+CH1 */
        case 0x01:   /* 000 000 1 CH0 */
        case 0x03:   /* 000 001 1 CH0+CH1 */
            Car_Move(0, 35);    break;

        /* 偏右三级: CH6 亮 / CH5+CH6 */
        case 0x40:   /* 1 000 000 CH6 */
        case 0x60:   /* 11 00 000 CH5+CH6 */
            Car_Move(35, 0);    break;

        /* 直角/十字: 左半全黑 或 右半全黑 → 原地转 */
        case 0x07:   /* 000 0111 CH0+CH1+CH2 左直角 */
            Car_Move(-30, 35);  break;
        case 0x70:   /* 111 0000 CH4+CH5+CH6 右直角 */
            Car_Move(35, -30);  break;

        /* 全丢或未覆盖状态: 慢速直行 (实际应配合 Track_GetError 的丢线处理) */
        case 0x00:
        default:
            Car_Move(15, 15);   break;
    }
}

/* 7 路查表偏差: 偏左为-, 偏右为+, 0~3 级 (供 PID 用) */
int Track_err(void)
{
    uint8_t state = Track_GetState();
    int err = 0;

    switch (state)
    {
        /* 偏差 0 级 (居中) */
        case 0x08:   /* CH3 中 */
        case 0x0C:   /* CH2+CH3 */
        case 0x18:   /* CH3+CH4 */
        case 0x1C:   /* CH2+CH3+CH4 */
        case 0x14:   /* CH2+CH4 对称 */
            err = 0;  break;

        /* 偏差 -1 级 (偏左) */
        case 0x04:   /* CH2 */
            err = -1;  break;

        /* 偏差 +1 级 (偏右) */
        case 0x10:   /* CH4 */
            err = 1;   break;

        /* 偏差 -2 级 */
        case 0x02:   /* CH1 */
        case 0x06:   /* CH1+CH2 */
            err = -2;  break;

        /* 偏差 +2 级 */
        case 0x20:   /* CH5 */
        case 0x30:   /* CH4+CH5 */
            err = 2;   break;

        /* 偏差 -3 级 */
        case 0x01:   /* CH0 */
        case 0x03:   /* CH0+CH1 */
            err = -3;  break;

        /* 偏差 +3 级 */
        case 0x40:   /* CH6 */
        case 0x60:   /* CH5+CH6 */
            err = 3;   break;

        /* 直角: 左/右半全黑 → 大偏差 */
        case 0x07:   /* CH0+CH1+CH2 左直角 */
            err = -4;  break;
        case 0x70:   /* CH4+CH5+CH6 右直角 */
            err = 4;   break;

        default:
            break;   /* 全丢等, 不改 err */
    }
    return err;
}

/* 4 路查表 (只用中间 4 路 CH2~CH5, 即 bit2~bit5)
 * 4 路布局: bit2=CH2(L1) bit3=CH3(M0) bit4=CH4(R1) bit5=CH5(R2) */
void FourTrack_Adjust(void)
{
    /* 只取中间 4 路 (bit2~bit5), 屏蔽其他 */
    uint8_t state = Track_GetState() & 0x3C;

    switch (state)
    {
        case 0x00:   /* 0000 全丢 */
            Car_Move(30, 30);   break;

        /* 居中: CH3 单亮 或 CH2+CH4 对称 */
        case 0x08:   /* 0010 CH3 */
        case 0x14:   /* 0101 CH2+CH4 对称 */
        case 0x1C:   /* 0111 CH2+CH3+CH4 */
            Car_Move(30, 30);   break;

        /* 偏左: CH2 亮 */
        case 0x04:   /* 0001 CH2 */
            Car_Move(20, 30);   break;

        /* 偏右: CH4 亮 */
        case 0x10:   /* 0100 CH4 */
            Car_Move(30, 20);   break;

        /* 偏左大: CH2+CH3 亮 */
        case 0x0C:   /* 0011 CH2+CH3 */
            Car_Move(0, 30);    break;

        /* 偏右大: CH3+CH4 亮 */
        case 0x18:   /* 0110 CH3+CH4 */
            Car_Move(30, 0);    break;

        /* 直角: 左两路全黑 或 右两路全黑 → 原地转 */
        case 0x24:   /* 1001 CH2+CH5 (跨中) - 罕见, 原地左转 */
            Car_Move(-40, 40);  break;

        default:
            Car_Move(15, 15);   break;
    }
}

/* 4 路查表偏差: 偏左-, 偏右+, 0~4 级 */
int FourTrack_err(void)
{
    uint8_t state = Track_GetState() & 0x3C;   /* 只看中间 4 路 */
    int err = 0;

    switch (state)
    {
        /* 偏差 0 (居中) */
        case 0x08:   /* CH3 */
        case 0x14:   /* CH2+CH4 对称 */
        case 0x1C:   /* CH2+CH3+CH4 */
            err = 0;  break;

        /* 偏差 1 */
        case 0x04:   /* CH2 偏左 */
            err = 1;  break;
        case 0x10:   /* CH4 偏右 */
            err = -1;  break;

        /* 偏差 2 */
        case 0x0C:   /* CH2+CH3 偏左 */
            err = 2;  break;
        case 0x18:   /* CH3+CH4 偏右 */
            err = -2;  break;

        /* 偏差 4 (直角) */
        case 0x24:   /* CH2+CH5 跨中 */
            err = 4;  break;

        default:
            break;
    }
    return err;
}
