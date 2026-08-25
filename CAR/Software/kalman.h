/**
 * @file  kalman.h
 * @brief 卡尔曼滤波器 (一维标量滤波, 适用于轨迹误差/陀螺角度等单变量平滑)
 *
 * 来源: mspm0g3507_car/Software/Kalman.h
 * 移植: 2026-07-23 → CAR/Software/
 * 适配: 无引脚依赖, 仅去掉 #include "main.h"
 *
 * 调参参考:
 *   - 轨迹多变 (系统自身变化快) → 增大 procNoise (Q), 让滤波器更信任新测量
 *   - 传感器噪声大 → 增大 measNoise (R), 让滤波器更平滑
 */
#ifndef __KALMAN_H__
#define __KALMAN_H__

#include <stdint.h>

/* 卡尔曼滤波器结构体 (一维) */
typedef struct KalmanFilter_TypeDef
{
    float procNoise;     /* 过程噪声方差 Q: 系统自身不确定性 */
    float measNoise;     /* 测量噪声方差 R: 传感器不确定性 */
    float estVal;        /* 估计值 x: 当前最优估计 */
    float estErrCov;     /* 估计误差协方差 P: 估计的不确定度 */
    float kalGain;       /* 卡尔曼增益 K: 测量 vs 估计的信任比例 */
} KalmanFilter_TypeDef;

/* 预定义的两个滤波器实例 (可选使用, 也可自建) */
extern struct KalmanFilter_TypeDef Track_Kalman;  /* 轨迹误差滤波 */
extern struct KalmanFilter_TypeDef Gyro_Kalman;   /* 陀螺角度滤波 */

/* 初始化滤波器: pNoise=Q, mNoise=R, initVal=初始估计值 */
void  Kalman_Init(KalmanFilter_TypeDef *kf, float pNoise, float mNoise, float initVal);

/* 输入新测量值, 返回滤波后的估计值 */
float Kalman_Update(KalmanFilter_TypeDef *kf, float measurement);

/* 便捷封装: 对轨迹误差 (int) 做卡尔曼滤波, 返回 int */
int   Track_Error_Filter(int err);

/* 便捷封装: 对陀螺角度 (float) 做卡尔曼滤波, 返回 float */
float Gyro_Angle_Filter(float angle);

#endif /* __KALMAN_H__ */
