#ifndef __BALANCE_H__
#define __BALANCE_H__

#include <stdint.h>

void Balance_Init(void);

// 速度环: 目标速度，左/右反馈(开环可传0), 时间间隔dt, 清零标志
float Vel_Loop(int32_t speed, int32_t L, int32_t R, float dt);

// 角度环: 目标角度，时间间隔dt, 清零标志
int32_t Ang_Loop(float target_ang,float measure_ang, float dt);

// 角速度环: 目标角速度，时间间隔dt, 清零标志
int32_t Pal_Loop(int32_t target_pal, int32_t measure_pal, float dt);

// 转向环: 时间间隔dt, 清零标志 (目标值从全局变量 Omega_Turn 读取)
int32_t Turn_Loop(float gyrozixl,float dt);

#endif /* __BALANCE_H__ */
