/**
 * @file  k230.h
 * @brief K230 视觉板通信驱动 (UART + FF FE 帧协议)
 *
 * 来源: mspm0g3507_car/Hardware/K230.h + Hardware/UART3.c
 * 移植: 2026-07-23 → CAR/Hardware/
 * 适配:
 *   - 源项目 UART3.c 仅一个 UART3_IRQHandler 调 K230_ReceiveData, 已合并进本文件
 *   - 源项目调用 Yaw_arr[Yaw_real] (int) → CAR 用 Yaw() (float, gyro_serial.h)
 *   - 源项目调用 Set_Speed(l,r) → CAR 用 Set_PWM(pwmA,pwmB) (motor.h, 签名一致)
 *
 * 引脚/串口状态: 用户暂不接 K230, UART 实例宏占位。
 *
 * ⚠️ 接硬件前必须在 SysConfig (empty.syscfg) 中:
 *    1. 添加一个 UART 实例 (如 UART3, PB2=TX/PB3=RX, 9600 或 115200 8N1)
 *    2. 生成代码后, 用 SysConfig 产出的宏替换下方的 K230_UART_INST / K230_UART_IRQ
 *    3. 在 main.c 中把 K230_UART 的 IRQHandler 内调 K230_ReceiveData(byte)
 *
 * 帧协议 (源项目实测):
 *   FF FE ... FE  — 起始 FF, 终止 FE, 中间为数据
 *   本驱动只做帧解析状态机, 具体数据字段含义由调用方自定
 */
#ifndef __K230_H__
#define __K230_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

#define K230_FRAME_MAX_PAYLOAD  18u

/* ============ UART 实例宏 (待用户在 SysConfig 配置后替换) ============
 * 占位: 假定用 UART3。SysConfig 配好后改为生成的宏, 例如:
 *   #define K230_UART_INST       UART_3_INST          (syscfg 生成)
 *   #define K230_UART_IRQ        UART_3_INST_INT_IRQN (syscfg 生成)
 */
#define K230_UART_INST          (UART3)             /* 待确认: SysConfig 实例 */
#define K230_UART_IRQ           (UART3_INT_IRQn)    /* 待确认: IRQ 号 */

/* ============ API ============ */
void K230_Init(void);                        /* 初始化: 使能 UART RX 中断 */
void K230_ReceiveData(uint8_t RxData);        /* 帧解析状态机 (在 UART ISR 中逐字节调用) */
void K230_Straght_Line_PID(int target);       /* 直线行走 PID (用 Yaw() 闭环, 写死参数, 按需调) */
uint8_t K230_FrameAvailable(void);
uint8_t K230_ReadFrame(uint8_t *dst, uint8_t max_len);

extern int detect_count;   /* 检测到停止位的次数 */

#endif /* __K230_H__ */
