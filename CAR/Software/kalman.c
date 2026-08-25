/**
 * @file  kalman.c
 * @brief 卡尔曼滤波器实现 (一维标量)
 *
 * 来源: mspm0g3507_car/Software/Kalman.c
 * 移植: 2026-07-23 → CAR/Software/
 * 适配: 去掉 #include "main.h", 保留 math.h
 */
#include "kalman.h"
#include <math.h>

/* 初始化卡尔曼滤波器 */
void Kalman_Init(KalmanFilter_TypeDef *kf, float pNoise, float mNoise, float initVal)
{
    kf->procNoise = pNoise;
    kf->measNoise = mNoise;
    kf->estVal    = initVal;   /* 初始估计值 */
    kf->estErrCov = 1.0f;      /* 初始估计误差协方差 */
}

/* 卡尔曼滤波更新: 输入测量值, 返回滤波后估计值 */
float Kalman_Update(KalmanFilter_TypeDef *kf, float measurement)
{
    /* 预测步骤: 误差协方差加上过程噪声 (系统不确定性增加) */
    kf->estErrCov += kf->procNoise;

    /* 更新步骤: 计算卡尔曼增益 K = P / (P + R) */
    kf->kalGain = kf->estErrCov / (kf->estErrCov + kf->measNoise);

    /* 用增益修正估计值: x = x + K * (测量值 - x) */
    kf->estVal += kf->kalGain * (measurement - kf->estVal);

    /* 更新误差协方差: P = (1 - K) * P */
    kf->estErrCov = (1.0f - kf->kalGain) * kf->estErrCov;

    return kf->estVal;
}

/* 轨迹误差滤波实例: Q=0.1, R=1.0 (平滑优先, 适合循迹误差) */
struct KalmanFilter_TypeDef Track_Kalman = {0.1f, 1.0f, 0.0f, 1.0f, 0};

int Track_Error_Filter(int err)
{
    return (int)Kalman_Update(&Track_Kalman, (float)err);
}

/* 陀螺角度滤波实例: Q=0.01, R=0.5 (高度平滑, 抑制 yaw 抖动) */
struct KalmanFilter_TypeDef Gyro_Kalman = {0.01f, 0.5f, 0.0f, 1.0f, 0};

float Gyro_Angle_Filter(float angle)
{
    return Kalman_Update(&Gyro_Kalman, angle);
}
