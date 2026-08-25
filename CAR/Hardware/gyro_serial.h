/**
 * @file  gyro_serial.h
 * @brief 串口单轴陀螺仪驱动 (0x5A 协议, 115200bps, UART1 PB6/PB7)
 *
 * 协议说明:
 *   角速度帧: 0x5A 0xAA AzL AzH Checksum  -> 5字节, raw/32768*2000 deg/s
 *   角度帧:   0x5A 0xBB YawL YawH Checksum -> 5字节, raw/32768*180 deg
 *
 * 校准指令 (5字节, 通过UART发送):
 *   解锁:    {0x55, 0xAA, 0x13, 0x8E, 0x5F}
 *   Z轴归零: {0x55, 0xAA, 0x15, 0x00, 0x00}
 *   保存:    {0x55, 0xAA, 0x00, 0x00, 0x00}
 *   零偏校准:{0x55, 0xAA, 0x0A, 0x01, 0x00}
 */
#ifndef __GYRO_SERIAL_H__
#define __GYRO_SERIAL_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* 陀螺仪数据结构 */
typedef struct {
    float Yaw;   /* 偏航角, 单位: deg, 范围 -180 ~ +180 */
} SAngle;

typedef struct {
    short rawWz; /* Z轴角速度原始值 */
    float wz;    /* Z轴角速度, 单位: deg/s, 范围 +-2000 */
} SGyro;

/* 全局数据访问 */
extern SAngle stcAngle;
extern SGyro  stcGyro;

/* API 函数 */
void  Gyro_Init(void);                          /* 初始化串口 + 发送校准命令 */
void  Gyro_ParseByte(unsigned char ucData);      /* 协议解析状态机 (在UART ISR中调用) */
void  Gyro_SendCalibrate(void);                  /* 发送Z轴归零 + 保存 */
void  Gyro_SendBiasCal(void);                    /* 发送零偏校准 (需保持静止21秒) */

float Yaw(void);                                 /* 获取当前Yaw角 (deg) */
float GyroZ(void);                               /* 获取Z轴角速度 (deg/s) */

#endif /* __GYRO_SERIAL_H__ */
