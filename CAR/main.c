/*
 * ============================================================
 * MSPM0G3507 Keil 封装库 - 主程序框架
 * 扩展板原理图引脚 + 定时器分工 (PWM=TIMA0/PA8-9, PID=TIMG6/10ms, 长计时=TIMG12/NTB)
 *
 * 底层驱动: motor/encoder/track/pid/oled(I2C)/gyro_serial(单轴)/gyro_6axis(六轴)/key/bsp
 *   注: gyro_serial(单轴5字节帧) 与 gyro_6axis(六轴11字节帧) 共用 UART1,
 *       靠 gyro_6axis.h 的 USE_GYRO_6AXIS 宏二选一, 同时只启用一套并换接对应陀螺仪.
 * 任务逻辑: 用户在 while(1) 与 TIMER_0_INST_IRQHandler 内自行填写
 * ============================================================
 *
 * 2026-07-23 从 mspm0g3507_car 补全移植的模块 (Hardware/ + Software/):
 *   Hardware: buzzer(PB17,低电平响) led(PA7,高电平点亮)
 *             step_motor(双4相8拍,引脚待接) servo(TIMG7/50Hz,引脚待SysConfig)
 *             k230(FF FE帧,UART待接,含原UART3的ISR逻辑)
 *   Software: kalman(一维标量滤波) change_type(二进制字符串转int)
 *   使用方法: 在下方 #include 区加入对应头文件即可, 例:
 *     #include "buzzer.h"
 *     #include "led.h"
 *     #include "step_motor.h"
 *     #include "servo.h"
 *     #include "k230.h"
 *     #include "kalman.h"
 *     #include "change_type.h"
 *   注意: 新 .c 文件需手动加入 Keil 工程 (.uvprojx) 文件组才会编译。
 *        servo/k230/step_motor 引脚为占位, 接硬件前先在 SysConfig 配置并替换 .h 顶部宏。
 * ============================================================
 */
#include "board.h"
#include "ti_msp_dl_config.h"
#include "bsp_systick.h"
#include "bsp_printf.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "track.h"       /* 7路循迹 + 转向环 + 弯道过弯 */
#include "oled.h"
#include "gyro_serial.h"
#include "gyro_6axis.h"   /* 六轴驱动; 启用时在 gyro_6axis.h 取消 USE_GYRO_6AXIS 注释, 见下方说明 */
#include "led.h"          /* LED1(PA7) + LED2(PB22) 状态慢闪 */
#include "key.h"
#include "vofa.h"       /* VOFA+ 上位机: JustFloat 波形上传 + 文本命令 PID 调参 */
#include <stdio.h>
#include <string.h>

/* ======================== 全局变量 ======================== */
volatile uint32_t sys_tick_ms = 0;   /* 10ms 节拍 (TIMER_0 中断累加, 供主循环节拍用) */

/* ======================== 开环符号标定开关 ========================
 * CALIB_OPEN_LOOP=1 时, K1 按下后 ISR 不跑 PID, 固定输出 Car_Move(+2000,+2000)
 *   跑 CALIB_RUN_MS 后停, 用来一次性测出:
 *   (a) Car_Move(+) 物理方向 (前进/后退) = 电机/驱动层符号
 *   (b) OLED EL 的正负 = 编码器层符号 (raw = Get_Encoder_countA() 不取反)
 *   闭环稳定的充要条件: Car_Move(+) 的物理方向 == cntL 增大的物理方向.
 *   标定完改回 0 即恢复 PID 速度环.
 * 用法: 烧录后按 K1, 看车走哪 + OLED EL 正负, 报给我定修正点. */
#define CALIB_OPEN_LOOP 0
#define CALIB_RUN_MS     1000     /* 固定输出持续 1 秒 */
volatile uint8_t  calib_running = 0;        /* K1 触发, 1=正在输出固定值 */
volatile uint32_t calib_start_ms = 0;       /* 开始时刻 */
volatile int      calib_el_raw = 0;         /* 给 OLED 显示的 raw (不取反) */
volatile int      calib_total_L = 0;        /* 累计总脉冲 (不清零), 诊断编码器是否匀速计数 */
volatile int      calib_total_R = 0;

/* ---- 速度环测试全局参数 ----
 * target_pulse : 速度环目标 (脉冲数/10ms), VOFA 命令 T1=N! 调 (K3 已改切页)
 * speed_running: 1=电机运转, 0=停 (K1切换)
 * enc_L/R      : 最近一次10ms编码器脉冲数 (ISR写, OLED读)
 * 例程速度环直接用脉冲数目标, 不换算 m/s.
 */
volatile int    target_pulse  = 5;      /* 速度环目标(脉冲数/10ms). 先 5 低速测角度环 (防甩尾) */
volatile float  target_yaw    = 0.0f;   /* 角度环目标航向(deg, -180~180). 默认 0=锁当前方向跑直 */
volatile uint8_t task_sel     = 0;       /* 0=T2(≥3路解锁,回正刹停), 1=T4(偏航>15°减速停), 2=T5(暂不停车). K3 切换 */
volatile float   start_yaw    = 0.0f;    /* 起跑时刻航向, 任务2/3 停车判据 */
volatile uint8_t  t3_armed    = 0;       /* 任务3: 已偏离>30°, 允许回归检测 */
/* T2(第二问)停车: 扫到≥3路不立即刹, 等车身回正(yaw≈start)再刹停 (车身尽量直) */
#define T2_RECOVER_DEG    5.0f          /* 回正判定: |yaw-start|<此值视为车身已直 (调小更直但易卡, 靠超时兜底) */
#define T2_ARM_TIMEOUT_MS 2000u         /* 解锁后最多等这么久, 超时强制刹 (防丢线/未回正失控) */
volatile uint8_t  t2_armed    = 0;       /* T2: 已扫到≥3路, 解锁等回正 */
volatile uint32_t t2_arm_ms   = 0;       /* T2 解锁时刻 (超时兜底) */
volatile uint8_t speed_running  = 0;       /* 默认停止, K1 启动 */
volatile int    enc_L = 0, enc_R = 0;
volatile float  spd_L_ms = 0.0f, spd_R_ms = 0.0f;
volatile int    cur_pwmL = 0, cur_pwmR = 0;   /* ISR 写, OLED 读: 显示用 PWM(取反,前进为正) */
volatile float  track_err_disp = 0.0f;          /* ISR 写, VOFA 波形: 循迹偏差 */
volatile int    track_diff_disp = 0;            /* ISR 写, VOFA 波形: 差速输出 */
volatile uint8_t speed_test_mode = 0;           /* 页面2 K1: 1=纯速度环测试, 0=循迹 */

/* OLED 当前显示页 (VOFA 命令 V1=N! 在线切换: 0=运行 1=调试 2=参数) */
volatile uint8_t oled_page = 0;

/* 角度环→速度环 量纲缩放: steer(PWM,±4500) → 脉冲差速. 1脉冲target≈700PWM(实测反推).
 * steer/STEER_TO_PULSE = diff_pulse. 700 能跑但P涨4000; 1500 启动不了(待查).
 * 太猛→加大, 不纠偏→减小. */
#define STEER_TO_PULSE 700.0f

/* 里程计调速: 赛道四段 (2直道+2半圆)
 *   780 脉冲/轮转, 轮周长=π×0.065=0.2042m, 脉冲/米=780/0.2042≈3820 */
#define PULSE_PER_M      3820.0f
#define LAP_TOTAL_M      6.142f   /* 1.5+π×0.5+1.5+π×0.5 */
#define STRAIGHT1_END    1.500f   /* 直道1 0→1.5m */
#define CURVE1_END       3.071f   /* 半圆1 1.5→3.071m (π×0.5≈1.571) */
#define STRAIGHT2_END    4.571f   /* 直道2 3.071→4.571m */
volatile float lap_dist_m = 0.0f;  /* 里程计累计 (起跑清零) */
volatile float lap_speed_base = 0; /* 当前速度档 base, OLED 显示用 */
volatile uint32_t run_start_ms   = 0; /* 起跑时刻 (sys_tick_ms) */
volatile uint32_t run_elapsed_ms = 0; /* 最终成绩 (停车冻结, K1清零) */
volatile uint8_t  decel_active = 0;   /* T4/T5 减速停车: 0=正常, 1=减速中 */
volatile uint8_t  decel_count  = 0;   /* 减速步数 (ISR 累加, 到阈值停车) */

/* ======================== OLED 显示工具 ========================
 * oled_putline: 在第 y 行写一行文本, 右侧补空格填满整行宽度.
 * 作用: 覆盖旧像素, 取代 OLED_Clear() 全清 -> 消除刷新闪烁/拖影.
 * 字号与行容量:
 *   size=12 -> 高12px, 5行(y=0/12/24/36/48), 21字符/行 (128/6)
 *   size=16 -> 高16px, 4行(y=0/16/32/48),    16字符/行 (128/8) */
static void oled_putline(uint8_t y, const char *s, uint8_t size)
{
    uint8_t cols = (size == 12) ? 21 : 16;   /* 每行字符数 */
    char line[24];
    uint8_t i = 0;
    while (s[i] && i < cols) { line[i] = s[i]; i++; }
    while (i < cols)         { line[i] = ' '; i++; }   /* 尾部补空格覆盖旧内容 */
    line[cols] = '\0';
    OLED_ShowString(0, y, (uint8_t*)line, size);
}

/* ======================== 初始化 ======================== */
int main(void)
{
    SYSCFG_DL_init();

    /* 启动电机 PWM 定时器 (TIMA0, PA8/PA9, 20kHz) */
    DL_Timer_startCounter(PWM_0_INST);

    /* 使能编码器中断 (ENCODERA/ENCODERB 全在 GPIOA, 共用 GROUP1 中断, 一次使能)
     * 宏 GPIO_MULTIPLE_GPIOA_INT_IRQN 由 syscfg 生成; ISR = GROUP1_IRQHandler (encoder.c) */
    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);

    /* 使能 TIMER_0 中断 (10ms PID 节拍, TIMG6, LOAD 事件) */
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    /* SysTick 1ms 基准 (delay_ms/delay_us 用, board.c 提供) */
    SysTick_Init();

    /* OLED 4线SPI 初始化 (SCL=PB9/SDA=PB8/RES=PB10/DC=PB11, 软件 GPIO) */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t*)"MSPM0G Lib", 16);   /* 24H_4 oled: 4参数(x,y,str,size) */
    OLED_Refresh();   /* 24H_4 oled.c 用 OLED_Refresh, 非 Refresh_Gram */

    /* IMU 串口初始化 (0x5A 协议, UART1 PB4/PB5)
     * USE_GYRO_6AXIS 在 gyro_6axis.h 定义: 启用六轴时走 Gyro6_Init,
     * 否则走单轴 Gyro_Init. 切换宏后需同时换接的陀螺仪硬件. */
#if defined(USE_GYRO_6AXIS)
    Gyro6_Init();
#else
    Gyro_Init();
#endif
    DL_UART_Main_clearInterruptStatus(VOFA_UART_INST,
        DL_UART_INTERRUPT_RX | DL_UART_INTERRUPT_OVERRUN_ERROR);
    DL_UART_Main_enableInterrupt(VOFA_UART_INST,
        DL_UART_INTERRUPT_RX | DL_UART_INTERRUPT_OVERRUN_ERROR);
    DL_UART_Main_setRXFIFOThreshold(VOFA_UART_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    NVIC_ClearPendingIRQ(VOFA_UART_IRQ);
    NVIC_EnableIRQ(VOFA_UART_IRQ);

    printf("=== MSPM0G Keil Lib Ready ===\r\n");

    /* ======================== Main Loop ======================== */
    while (1)
    {
        /* 用户任务逻辑写在这里:
         *   - 读编码器: Get_Encoder_countA() / Get_Encoder_countB() (encoder.h)
         *   - 读 IMU: Yaw() (gyro_serial.h)
         *   - PID 计算: Angle_Calculate (pid.h) / Velocity_A/B (encoder.h)
         *   - 电机输出: Set_PWM(a, b) / Car_Move(pl, pr) (motor.h)
         *   - 按键: Key() (key.h, Task5 重写后用 syscfg 宏读 K1/K2/K3)
         *   - OLED 显示: OLED_ShowAngle6() 100ms 刷一次六轴三角度 (oled.h)
         */

        /* ---- 按键: 边沿检测 (按下瞬间只触发一次)
         * Key_Scan 非阻塞无去抖, 主循环快, 按住期间每次扫描都返回键值 ->
         * 不做边沿检测会一次按下触发多次 (切页跳多页/启停来回toggle).
         * 只在 0->按下 跳变时处理, 加 20ms 去抖确认. */
        static uint8_t key_prev = 0;
        uint8_t key = Key_Scan();
        if (key != 0 && key_prev == 0) {
            delay_ms(20);                   /* 去抖 */
            if (Key_Scan() == key) {        /* 确认还按着同一键 */
                if (key == 1) {            /* K1: 页0=循迹启停, 页1=减速, 页2=速度环测试 */
                    if (oled_page == 1) {
                        target_pulse -= 1;
                        if (target_pulse < 0) target_pulse = 0;
                    } else if (oled_page == 2) {
                        /* 页面2: 纯速度环测试, 两轮同 target */
                        speed_test_mode = !speed_test_mode;
                        if (speed_test_mode) {
                            speed_running = 1;
                            lap_dist_m = 0.0f; run_start_ms = sys_tick_ms;
                            run_elapsed_ms = 0;
                        } else {
                            speed_running = 0;
                            Motor_Brake();
                            Vel_PI_Reset_A();
                            Vel_PI_Reset_B();
                        }
                    } else {
#if CALIB_OPEN_LOOP
                        calib_running = 1;
                        calib_start_ms = sys_tick_ms;
#else
                        speed_running = !speed_running;
                        if (speed_running) { lap_dist_m = 0.0f; run_start_ms = sys_tick_ms; run_elapsed_ms = 0; start_yaw = Gyro6_Yaw(); t3_armed = 0; t2_armed = 0; decel_active = 0; decel_count = 0; }  /* 起跑清零 */                        if (!speed_running) {
                            Motor_Brake();            /* 短接制动锁0 */
                            Vel_PI_Reset_A();
                            Vel_PI_Reset_B();
                            Angle_PID_Reset();
                        }
#endif
                    }
                } else if (key == 2) {     /* K2: 速度环测试=减速, 其他=切页 */
                    if (oled_page == 2 && speed_test_mode) {
                        target_pulse -= 1;
                        if (target_pulse < 0) target_pulse = 0;
                    } else {
                        oled_page = (oled_page + 1) % 3;
                    }
                } else if (key == 3) {     /* K3: 页1/页2测试=加速, 其他=T2→T4→T5 */
                    if (oled_page == 1 || (oled_page == 2 && speed_test_mode)) {
                        target_pulse += 1;
                    } else {
                        task_sel = (task_sel + 1) % 3;
                        Vel_PI_Reset_A();
                        Vel_PI_Reset_B();
                    }
                }
            }
        }
        key_prev = key;

        /* ---- VOFA+ 命令轮询: 检查 "P1=12.5!" 类 PID 调参命令 ---- */
        Vofa_PollCommand();
        /* 运行时参数同步: 停车调参时 ISR 不跑, 主循环里同步 TRACK_KP/KD */
        if (task_sel == 0) { TRACK_KP = TRACK_KP_T1; TRACK_KD = TRACK_KD_T1; }
        else              { TRACK_KP = TRACK_KP_T2; TRACK_KD = TRACK_KD_T2; }

        /* ---- OLED 显示 (100ms 刷新, 12字号, 每页5行) ----
         * K3 切页: 0=运行 1=调试 2=参数. 切页时 OLED_Clear() 消旧残.
         * 注: 12字号 y 间距=12, 5行用 y=0/12/24/36/48 (12×5=60<64). */
        static uint32_t last_oled_ms = 0;
        static uint8_t  disp_page_last = 0xFF;
        if (sys_tick_ms - last_oled_ms >= 100) {
            last_oled_ms = sys_tick_ms;
            if (oled_page != disp_page_last) {
                OLED_Clear();
                disp_page_last = oled_page;
            }
            char buf[24];
#if CALIB_OPEN_LOOP
            OLED_ShowString(0, 0,  (uint8_t*)"CALIB  K1=run",12);
            sprintf(buf,"EL:%d  ER:%d",(int)calib_el_raw,(int)enc_R);
            OLED_ShowString(0,12,(uint8_t*)buf,12);
            sprintf(buf,"TL:%d  TR:%d",(int)calib_total_L,(int)calib_total_R);
            OLED_ShowString(0,24,(uint8_t*)buf,12);
            OLED_ShowString(0,48,(uint8_t*)"OPEN LOOP",12);
#else
            #define PULSE_TO_MS 0.03423f
            uint32_t elapsed_ms = speed_running ? (sys_tick_ms - run_start_ms) : run_elapsed_ms;
            switch (oled_page) {
            case 0:
                /* 运行: 任务/状态/计时/航向/KPKD/编码器/PWM (5行) */
                sprintf(buf,"T%d %s  T:%.1fs",
                    (task_sel==0)?2:(task_sel==1)?4:5,speed_running?"RUN":"STP",
                    (double)elapsed_ms / 1000.0);
                OLED_ShowString(0,0,(uint8_t*)buf,12);
                {   /* 直接读任务源参数, 停车调参时也能看到变化 */
                    float kp = (task_sel == 0) ? TRACK_KP_T1 : TRACK_KP_T2;
                    float kd = (task_sel == 0) ? TRACK_KD_T1 : TRACK_KD_T2;
                    sprintf(buf,"Y:%+.0f KP:%.1f KD:%.1f",
                        (double)Gyro6_Yaw(),(double)kp,(double)kd);
                }
                OLED_ShowString(0,12,(uint8_t*)buf,12);
                sprintf(buf,"EL:%+d ER:%+d V:%.2f",
                    enc_L,enc_R,(double)((enc_L+enc_R)/2*PULSE_TO_MS));
                OLED_ShowString(0,24,(uint8_t*)buf,12);
                sprintf(buf,"PWM:%d %d B:%d",
                    cur_pwmL,cur_pwmR,(int)lap_speed_base);
                OLED_ShowString(0,36,(uint8_t*)buf,12);
                sprintf(buf,"D:%.2fm",(double)lap_dist_m);
                OLED_ShowString(0,48,(uint8_t*)buf,12);
                break;
            case 1:
                /* 调试: 7路传感器/状态hex/脉冲/速度/航向+档+PWM (5行) */
                {
                    uint8_t ts = Track_GetState();
                    sprintf(buf,"S:%d%d%d%d%d%d%d ST:%02X",
                        (ts>>0)&1,(ts>>1)&1,(ts>>2)&1,(ts>>3)&1,
                        (ts>>4)&1,(ts>>5)&1,(ts>>6)&1,ts);
                }
                OLED_ShowString(0,0,(uint8_t*)buf,12);
                sprintf(buf,"EL:%+d ER:%+d",enc_L,enc_R);
                OLED_ShowString(0,12,(uint8_t*)buf,12);
                sprintf(buf,"V:%+.2f %+.2f m/s",
                    (double)(enc_L*PULSE_TO_MS),(double)(enc_R*PULSE_TO_MS));
                OLED_ShowString(0,24,(uint8_t*)buf,12);
                sprintf(buf,"T:%.1fs D:%.2fm S:%d",
                    (double)elapsed_ms / 1000.0,
                    (double)lap_dist_m, target_pulse);
                OLED_ShowString(0,36,(uint8_t*)buf,12);
                sprintf(buf,"PWM:%d %d %s",
                    cur_pwmL,cur_pwmR,speed_running?"RUN":"STP");
                OLED_ShowString(0,48,(uint8_t*)buf,12);
                break;
            case 2:
                /* 参数页: T2 PD / T4&T5 PD / 速度环 / VOFA提示 */
                sprintf(buf,"T2 KP:%.1f KD:%.1f",
                    (double)TRACK_KP_T1,(double)TRACK_KD_T1);
                OLED_ShowString(0,0,(uint8_t*)buf,12);
                sprintf(buf,"T4 KP:%.1f KD:%.1f",
                    (double)TRACK_KP_T2,(double)TRACK_KD_T2);
                OLED_ShowString(0,12,(uint8_t*)buf,12);
                sprintf(buf,"VP:%.0f VI:%.1f VF:%.0f",
                    (double)Velcity_Kp,(double)Velcity_Ki,(double)Velcity_Kff);
                OLED_ShowString(0,24,(uint8_t*)buf,12);
                if (speed_test_mode) {
                    sprintf(buf,"TEST T:%d EL:%d ER:%d",
                        target_pulse, enc_L, enc_R);
                    OLED_ShowString(0,36,(uint8_t*)buf,12);
                    sprintf(buf,"PWM L:%d R:%d", cur_pwmL, cur_pwmR);
                    OLED_ShowString(0,48,(uint8_t*)buf,12);
                } else {
                    sprintf(buf,"P1/D1=T2 P3/D3=T4");
                    OLED_ShowString(0,36,(uint8_t*)buf,12);
                    sprintf(buf,"P2=VP I2=VI D2=VF");
                    OLED_ShowString(0,48,(uint8_t*)buf,12);
                }
                break;
            }
#endif
            /* JustFloat 波形: 3通道 (左脉冲/右脉冲/航向), 100ms 送一次 */
            {
                float wf[5] = {
                    (float)enc_L, (float)enc_R,
                    track_err_disp, (float)track_diff_disp,
                    Gyro6_Yaw()
                };
                Vofa_JustFloat_Send(wf, 5);
            }
            OLED_Refresh();
        }
    }
}

/* ======================== UART0 中断 (调试串口 printf, 仅清错误) ======================== */
void UART_0_INST_IRQHandler(void)
{
    uint32_t iidx = DL_UART_getPendingInterrupt(UART_0_INST);

    if (iidx == DL_UART_IIDX_RX) {
        DL_UART_receiveData(UART_0_INST);   /* 读走数据防止 OVERRUN, 不处理 */
    }
    else if (iidx == DL_UART_IIDX_OVERRUN_ERROR) {
        DL_UART_receiveData(UART_0_INST);
    }
}

/* ======================== UART1 中断 (JDY-31 蓝牙 VOFA+ + IMU 陀螺仪) ========================
 * UART1 物理接 JDY-31 蓝牙从机 (VOFA+) 或 IMU 陀螺仪, 二选一接线.
 * 代码层双协议并行解析: VOFA 文本命令 → Vofa_RX_ISR, IMU 0x5A 协议 → Gyro6_ParseByte.
 * 接 IMU 时 Vofa_RX_ISR 收到二进制垃圾 (无 "=...!" 模式, 自动忽略).
 * 接蓝牙时 Gyro6_ParseByte 收到文本乱码 (无 0x5A 帧头, 自动忽略).
 * 函数名必须和 syscfg 实例 UART_1 生成的 UART_1_INST_IRQHandler 一致
 * 关键: DL_UART_getPendingInterrupt 读 IIDX 寄存器是"读即清" (hw_uart.h:
 *   "A read clears the corresponding interrupt flag in RIS and MIS registers").
 * 必须只读一次存到局部变量, 再用变量判断分支。原版调两次 getPendingInterrupt,
 * 第二次读会把已清的 RX 标志或下一个挂起中断清掉, 导致 RX 字节丢失/中断异常。 */
void VOFA_UART_INST_IRQHandler(void)
{
    uint32_t iidx = DL_UART_getPendingInterrupt(VOFA_UART_INST);   /* 只读一次, 读即清 */

    if (iidx == DL_UART_IIDX_RX) {
        unsigned char byte = (unsigned char)DL_UART_receiveData(VOFA_UART_INST);
        Vofa_RX_ISR(byte);   /* VOFA+ 文本命令 (蓝牙 HC-05 下发的 "P1=12.5!") */
#if !defined(UART_2_INST)
#if defined(USE_GYRO_6AXIS)
        Gyro6_ParseByte(byte);
#else
        Gyro_ParseByte(byte);
#endif
#endif
    }
    else if (iidx == DL_UART_IIDX_OVERRUN_ERROR) {
        DL_UART_receiveData(VOFA_UART_INST);  /* 清 OVERRUN 错误 */
    }
}

#if defined(UART_2_INST)
void UART_1_INST_IRQHandler(void)
{
    uint32_t iidx = DL_UART_getPendingInterrupt(UART_1_INST);

    if (iidx == DL_UART_IIDX_RX) {
        unsigned char byte = (unsigned char)DL_UART_receiveData(UART_1_INST);
#if defined(USE_GYRO_6AXIS)
        Gyro6_ParseByte(byte);
#else
        Gyro_ParseByte(byte);
#endif
    }
    else if (iidx == DL_UART_IIDX_OVERRUN_ERROR) {
        DL_UART_receiveData(UART_1_INST);
    }
}
#endif

/* ======================== 10ms 定时器中断 (TIMG6, PID 节拍) ======================== */
/* TIMER_0 = TIMG6, 中断事件为 LOAD (syscfg 配 interrupts=["LOAD"])。
 * 注意: 事件宏用通用版 DL_TIMER_INTERRUPT_LOAD_EVENT; clear 用通用版 DL_Timer_clearInterruptStatus。
 *       (原方案 TIMA0 让给 PWM, PID 定时器改用 TIMG6) */
void TIMER_0_INST_IRQHandler(void)
{
    DL_Timer_clearInterruptStatus(TIMER_0_INST, DL_TIMER_INTERRUPT_LOAD_EVENT);

    sys_tick_ms += 10;

    /* ======================== 速度环 (每 10ms) ========================
     * 1. 读编码器脉冲 (Get_Encoder_countA/B 内部清零, 10ms 累计)
     * 2. 换算实际速度 m/s (供 OLED 显示)
     * 3. 目标 m/s → 目标脉冲数, 喂 Velocity_A/B (纯P, Kp=5) 得 PWM
     * 4. Car_Move 输出 (限幅±7999), 停止时输出0
     * 换算常数: pulses_per_rev_wheel=1560, circumference=0.2670m, sample=0.01s
     *   目标脉冲数 = target_ms * 1560 * 0.01 / 0.2670 = target_ms * 58.43
     *   实际速度   = pulses * 0.2670 / (1560 * 0.01) = pulses * 0.01712
     */
    /* 速度环照搬 WHEELTEC 例程 (empty.c ISR):
     *   Get_Encoder_countA/B 是变量 (encoder.c), ISR 累加, 这里取值后清零.
     *   encoderB 取反 (例程符号, 后续标定可调).
     *   目标 -15 (脉冲数, 例程值), Velocity_A/B 增量式 PI, 结果取反, Set_PWM 直接输出.
     * 闭环方向判据: Set_PWM(+PWMA) 物理方向 必须 == encoderA_cnt 增大方向.
     *   发散→在取值处加/去取反或翻目标/结果符号(成对翻一处); 倒跑→翻 motor.c Set_PWM 方向条件.
     */
    int encoderA_cnt;
    int encoderB_cnt;
    Encoder_GetAndClear(&encoderA_cnt, &encoderB_cnt);
    /* 编码器方向适配前驱 (Set_PWM正负极交换后, encoder符号同步翻) */
    encoderB_cnt = -encoderB_cnt;                    /* B 取反, 闭环方向配对 */

    enc_L = encoderA_cnt;    /* 给 OLED 显示 */
    enc_R = encoderB_cnt;

    /* 里程计: 左右轮平均脉冲 → 累计米 */
    lap_dist_m += (float)(encoderA_cnt + encoderB_cnt) / 2.0f / PULSE_PER_M;

#if CALIB_OPEN_LOOP
    calib_el_raw = encoderA_cnt;          /* raw (不取反), 给 OLED 显示编码器层符号 */
    calib_total_L += encoderA_cnt;        /* 累计 (不清零), 诊断匀速计数 */
    calib_total_R += encoderB_cnt;

    /* 标定: K1 触发后固定输出正值, 跑 CALIB_RUN_MS 后停 (直接 Set_PWM, 不走 Car_Move) */
    if (calib_running) {
        if (sys_tick_ms - calib_start_ms < CALIB_RUN_MS) {
            Set_PWM(2000, 2000);          /* 固定正命令, 跳过 PID. PWMA=右, PWMB=左 */
        } else {
            calib_running = 0;
            Set_PWM(0, 0);
        }
    } else {
        Set_PWM(0, 0);
    }
#else
    if (speed_running) {
        int target_L, target_R;

        /* 页面2 纯速度环测试: 两轮同 target, 不走循迹 */
        if (speed_test_mode) {
            target_L = target_pulse;
            target_R = target_pulse;
            track_err_disp = 0.0f;
            track_diff_disp = 0;
            lap_speed_base = (float)target_pulse;
        } else
        /* 循迹差速转向 */
        {
            uint8_t stop_now = 0;
            if (task_sel == 0) {
                /* T2(第二问): 扫到≥3路先解锁(不立即刹), 等车身回正(yaw≈start)再刹停
                 *   扫3路 = 进十字/终点, 此时往往带偏. 直接刹车身斜.
                 *   解锁后继续循迹纠偏, 直到 |yaw-start| < T2_RECOVER_DEG 视为回正,
                 *   或超时 T2_ARM_TIMEOUT_MS 强制刹 (防丢线/未回正失控). */
                uint8_t ts = Track_GetState();
                uint8_t cnt = 0;
                for (uint8_t i = 0; i < 7; i++)
                    if (ts & (1 << i)) cnt++;
                if (cnt >= 3 && !t2_armed) {        /* 首次扫到≥3路 → 解锁等回正 */
                    t2_armed = 1;
                    t2_arm_ms = sys_tick_ms;
                }
                if (t2_armed) {
                    float dy = Gyro6_Yaw() - start_yaw;
                    while (dy >  180.0f) dy -= 360.0f;
                    while (dy < -180.0f) dy += 360.0f;
                    if (dy < 0.0f) dy = -dy;
                    uint8_t timeout = (sys_tick_ms - t2_arm_ms) > T2_ARM_TIMEOUT_MS;
                    if (dy < T2_RECOVER_DEG || timeout) stop_now = 1;   /* 回正或超时 → 刹停 */
                }
            } else if (task_sel == 1) {
                /* 任务4: 偏航角变化 >15° → 减速停车 */
                float dy = Gyro6_Yaw() - start_yaw;
                while (dy >  180.0f) dy -= 360.0f;
                while (dy < -180.0f) dy += 360.0f;
                if (dy < 0.0f) dy = -dy;
                if (dy > 15.0f && !decel_active) decel_active = 1;
            } else {
                /* T5: 暂不停车, 一直转 */
            }
            if (stop_now) {
                /* T2 回正后刹停 (短接制动锁0, 车身保持直) */
                run_elapsed_ms = sys_tick_ms - run_start_ms;
                speed_running = 0;
                Motor_Brake();
                Vel_PI_Reset_A();
                Vel_PI_Reset_B();
                target_L = 0; target_R = 0;
                t2_armed = 0;
                decel_active = 0;
            } else {
                int base, diff;
                /* 按任务加载 PD: T2独享, T4/T5共用 */
                if (task_sel == 0) { TRACK_KP = TRACK_KP_T1; TRACK_KD = TRACK_KD_T1; }
                else              { TRACK_KP = TRACK_KP_T2; TRACK_KD = TRACK_KD_T2; }
                float err = Track_GetError();
                Track_Steering_Compute(err, &base, &diff);
                track_err_disp = err;
                track_diff_disp = diff;
                /* 任务定速: T2=15, T4/T5=10 */
                base = (task_sel == 0) ? TRACK_BASE_FAST : 10;
                lap_speed_base = (float)base;
                /* T4/T5 减速停车: base 线性降至 0, ~100周期(1s) */
                if (decel_active) {
                    decel_count++;
                    if (decel_count >= 100) {
                        /* 减速完成, 自然停车 */
                        run_elapsed_ms = sys_tick_ms - run_start_ms;
                        speed_running = 0;
                        Set_PWM(0, 0);
                        Vel_PI_Reset_A();
                        Vel_PI_Reset_B();
                        target_L = 0; target_R = 0;
                        decel_active = 0;
                        decel_count  = 0;
                    } else {
                        base = base * (100 - decel_count) / 100;
                        if (base < 1) base = 1;
                    }
                }
                target_L = base - diff;   /* error>0线在右→右转→左快右慢 */
                target_R = base + diff;
                if (target_L < -15) target_L = -15;  if (target_L > 15) target_L = 15;
                if (target_R < -15) target_R = -15;  if (target_R > 15) target_R = 15;
            }
        }

        /* 速度环(内环): 命令+反馈同轮. PWMA→左轮(encoderA), PWMB→右轮(encoderB) */
        int PWMA = -Velocity_A(target_L, encoderA_cnt);   /* 左轮命令+左轮反馈 */
        int PWMB = -Velocity_B(target_R, encoderB_cnt);   /* 右轮命令+右轮反馈 */
        PWMA = limit_PWM(PWMA, -7999, 7999);
        PWMB = limit_PWM(PWMB, -7999, 7999);
        Set_PWM(PWMA, PWMB);   /* PWMA→左, PWMB→右 (实物) */
        cur_pwmL = -PWMA;      /* 显示取反: 前进=正PWM, 后退=负 (实际输出PWMA不动) */
        cur_pwmR = -PWMB;      /* 右轮同理取反 */
    } else {
        cur_pwmL = 0;
        cur_pwmR = 0;
        Motor_Brake();           /* 停车时短接制动, 非滑行 */
    }
#endif
}

/* ======================== SysTick 中断 (1ms tick) ========================
 * 照搬 WHEELTEC 例程 LED_Flash 思路: 1ms 节拍里调 LED2_StatusFlash(500)
 * → PB22 用户灯 500ms 亮 / 500ms 灭慢闪, 用来看程序是否在待机跑
 * 不占用 TIMER_0 (10ms PID 节拍) */
void SysTick_Handler(void)
{
    LED2_StatusFlash(500);
}
