#ifndef CODE_VOFA_H_
#define CODE_VOFA_H_

#include "ti_msp_dl_config.h"
#include <stdint.h>

#if defined(UART_2_INST)
#define VOFA_UART_INST            UART_2_INST
#define VOFA_UART_IRQ             UART_2_INST_INT_IRQN
#define VOFA_UART_INST_IRQHandler UART_2_INST_IRQHandler
#else
#define VOFA_UART_INST            UART_1_INST
#define VOFA_UART_IRQ             UART_1_INST_INT_IRQN
#define VOFA_UART_INST_IRQHandler UART_1_INST_IRQHandler
#endif

/* ============================================================
 * VOFA+ 上位机通信协议 (移植自 VOFA 参考工程)
 *
 * 硬件接线: UART2 = PB15(TX) / PB18(RX), 9600 8N1 (syscfg 配置)
 *   PC ──USB── HC-05(主机) ──蓝牙── JDY-31(从机,小车上) ──UART2── MSPM0G3507
 *   IMU 走 UART1 (PB4/PB5), 蓝牙走 UART2, 物理分开不冲突
 *
 * 1. JustFloat 协议 (MCU → VOFA+): 二进制浮点数据上传
 *    帧格式: [N×float(4B 小端)][帧尾 0x00 0x00 0x80 0x7F]
 *    帧尾 = IEEE 754 +Inf, VOFA+ 靠它实现无帧头同步
 *
 * 2. 文本命令协议 (VOFA+ → MCU): PID 参数下发
 *    帧格式: XX=value!
 *    XX ∈ {P1,P2,P3, I1,I2,I3, D1,D2,D3}
 *
 *    VOFA 命令 → PID 参数映射 (由 Vofa_PollCommand 自动关联):
 *    P1/I1/D1 → ANGLE_KP / ANGLE_KI / ANGLE_KD (pid.h extern, 角度环)
 *    P2/I2/D2 -> Velcity_Kp / Velcity_Ki / Velcity_Kff (速度环, encoder.c)
 *    P3/-/D3 -> TRACK_KP / (无积分) / TRACK_KD (循迹转向环, track.c)
 *    直接改上述全局变量即生效, 无需额外调用.
 * ============================================================ */

/* ---- JustFloat: 发送 N 通道 float 数据 ---- */

/* 通用 N 通道发送 (data 数组长度 = count) */
void Vofa_JustFloat_Send(float *data, uint8_t count);

/* 快捷发送 1/2/3 通道 */
void Vofa_JustFloat_One(float a);
void Vofa_JustFloat_Two(float a, float b);
void Vofa_JustFloat_Three(float a, float b, float c);

/* ---- 文本命令: 接收解析 ---- */

/* UART1 RX 中断回调 (由 UART_1_INST_IRQHandler 调用, 每收到 1 字节调一次) */
void Vofa_RX_ISR(uint8_t byte);

/* 主循环中轮询: 检查缓冲区是否有完整命令, 有则解析并更新 PID 参数 */
void Vofa_PollCommand(void);

/* OLED 当前显示页 (VOFA 命令 V1=N! 切换: 0=运行 1=调试 2=参数) */
extern volatile uint8_t oled_page;

/* 速度档目标脉冲数 (VOFA 命令 T1=N! 调整, K3 已改切页故速度档改由 VOFA 调) */
extern volatile int target_pulse;

#endif /* CODE_VOFA_H_ */
