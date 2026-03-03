#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

/* --- 【关键修复】添加标准整数类型头文件 --- */
#include <stdint.h>  

/* 如果你的项目中 float 也有问题，确保编译器支持 float (Keil V5 默认支持) */

typedef struct {
    float Q_angle;      // 过程噪声协方差 (角度)
    float Q_gyro;       // 过程噪声协方差 (角速度偏置)
    float R_angle;      // 测量噪声协方差 (加速度计)
    float dt;           // 滤波周期 (秒)

    float angle;        // 滤波后的角度
    float angle_dot;    // 滤波后的角速度
    float Q_bias;       // 陀螺仪零偏估计

    float P[2][2];      // 误差协方差矩阵
} Kalman_Filter_t;

// 函数声明
void Kalman_Filter_Init(Kalman_Filter_t *kf, float Q_angle, float Q_gyro, float R_angle, float dt);
void Kalman_Filter_Update(Kalman_Filter_t *kf, float acc_angle_deg, float gyro_rate_dps);

#endif

