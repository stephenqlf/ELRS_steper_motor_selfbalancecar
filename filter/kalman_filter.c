#include "kalman_filter.h"
#include "math.h"


Kalman_Filter_t balance_kf; // 全局变量


/**
 * @brief 初始化卡尔曼滤波器
 * @param kf: 指向滤波器结构体的指针
 * @param Q_angle: 过程噪声协方差 (角度)
 * @param Q_gyro: 过程噪声协方差 (角速度偏置)
 * @param R_angle: 测量噪声协方差 (加速度计)
 * @param dt: 滤波周期 (秒)
 */
void Kalman_Filter_Init(Kalman_Filter_t *kf, float Q_angle, float Q_gyro, float R_angle, float dt)
{
    kf->Q_angle = Q_angle;
    kf->Q_gyro  = Q_gyro;
    kf->R_angle = R_angle;
    kf->dt      = dt;

    // 初始状态
    kf->angle   = 0.0f;
    kf->angle_dot = 0.0f;
    kf->Q_bias  = 0.0f;

    // 初始化协方差矩阵 P
    kf->P[0][0] = 1.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;
}

/**
 * @brief 执行一次卡尔曼滤波更新
 * @param kf: 指向滤波器结构体的指针
 * @param acc_angle: 由加速度计计算出的角度 (度)
 * @param gyro_rate: 陀螺仪原始角速度 (度/秒)
 */
void Kalman_Filter_Update(Kalman_Filter_t *kf, float acc_angle, float gyro_rate)
{
    float dt = kf->dt;
    float Pdot[4];

    // --- 预测阶段 (Predict) ---
    // 更新角度预测值
    kf->angle += (gyro_rate - kf->Q_bias) * dt;

    // 更新协方差矩阵 P
    Pdot[0] = kf->Q_angle - kf->P[0][1] - kf->P[1][0];
    Pdot[1] = -kf->P[1][1];
    Pdot[2] = -kf->P[1][1];
    Pdot[3] = kf->Q_gyro;

    kf->P[0][0] += Pdot[0] * dt;
    kf->P[0][1] += Pdot[1] * dt;
    kf->P[1][0] += Pdot[2] * dt;
    kf->P[1][1] += Pdot[3] * dt;

    // --- 更新阶段 (Update) ---
    // 计算残差
    float Angle_err = acc_angle - kf->angle;

    // 计算卡尔曼增益
    float PCt_0 = kf->P[0][0];
    float PCt_1 = kf->P[1][0];
    float E = kf->R_angle + PCt_0;
    float K_0 = PCt_0 / E;
    float K_1 = PCt_1 / E;

    // 更新协方差矩阵 P
    kf->P[0][0] -= K_0 * PCt_0;
    kf->P[0][1] -= K_0 * kf->P[0][1];
    kf->P[1][0] -= K_1 * PCt_0;
    kf->P[1][1] -= K_1 * kf->P[0][1];

    // 更新状态
    kf->angle  += K_0 * Angle_err;
    kf->Q_bias += K_1 * Angle_err;
    kf->angle_dot = gyro_rate - kf->Q_bias;
}
