/**
 * @file  gyro_6axis.h
 * @brief 六轴串口陀螺仪驱动 (0x5A 协议, 11字节帧, UART1 PB4/PB5)
 *
 * 协议帧: 0x5A TYPE D0L D0H D1L D1H D2L D2H D3L D3H SUM (11字节)
 *   TYPE: 0xAA=角速度 0xBB=角度 0xCC=加速度 0xDD=四元数
 *   SUM : 前10字节累加和低字节
 *   换算: 角速度 raw/32768*2000 deg/s, 角度 raw/32768*180 deg,
 *         加速度 raw/32768*16*9.8 m/s2, 四元数 raw/32768
 *
 * 与单轴 gyro_serial.c 共存, 共用 UART1, 靠本宏二选一:
 *   启用六轴时取消下一行注释, 并确保 main.c ISR 走 Gyro6_ParseByte 分支.
 */
#ifndef __GYRO_6AXIS_H__
#define __GYRO_6AXIS_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

/*==== 宏开关: 启用六轴驱动, 注释掉则用单轴 gyro_serial ====*/
#define USE_GYRO_6AXIS

/*==== 数据结构 ====*/
typedef struct {
    float Roll;   /* 横滚角 deg  -180~+180 */
    float Pitch;  /* 俯仰角 deg  -180~+180 */
    float Yaw;    /* 航向角 deg  -180~+180 */
} SAngle6;

typedef struct {
    float wx;     /* X轴角速度 deg/s  +-2000 */
    float wy;     /* Y轴角速度 deg/s */
    float wz;     /* Z轴角速度 deg/s */
    short rawWx;
    short rawWy;
    short rawWz;
} SGyro6;

typedef struct {
    float ax;     /* X轴加速度 m/s2  +-16g */
    float ay;
    float az;
    short rawAx;
    short rawAy;
    short rawAz;
} SAccel6;

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} SQuat6;

/*==== 全局数据 ====*/
extern SAngle6 stcAngle6;
extern SGyro6  stcGyro6;
extern SAccel6 stcAccel6;
extern SQuat6  stcQuat6;

/*==== API ====*/
void  Gyro6_Init(void);                  /* 初始化串口 + 发Z轴归零 */
void  Gyro6_ParseByte(unsigned char b);  /* 11字节帧状态机, UART ISR 调用 */
void  Gyro6_SendCalibrate(void);         /* Z轴归零 */
void  Gyro6_SendBiasCal(void);           /* 零偏校准 (需静止21秒) */

float Gyro6_X(void);  float Gyro6_Y(void);  float Gyro6_Z(void);
float Gyro6_Roll(void); float Gyro6_Pitch(void); float Gyro6_Yaw(void);
float Gyro6_AccelX(void); float Gyro6_AccelY(void); float Gyro6_AccelZ(void);
float Gyro6_Q0(void); float Gyro6_Q1(void); float Gyro6_Q2(void); float Gyro6_Q3(void);

#endif /* __GYRO_6AXIS_H__ */
