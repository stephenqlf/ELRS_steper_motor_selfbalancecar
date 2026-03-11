#include <math.h>
#include <stdint.h>
#include "balance.h"
#include "MPU6050.h"

/* ================= 全局/外部变量映射 ================= */
extern float pitch, roll, yaw;       
extern short gyrox, gyroy, gyroz;    
extern float Zhongzhi;               
extern uint8_t Flag_Stop ;            

// 模仿示例程序的全局 Trim 和目标变量
// 需要在 control.c 中定义并赋值
extern int32_t Speed_Trim;   
extern int32_t Turn_Trim;    
extern int32_t Omega_Turn;   // 目标转向角速度

/* ================= PID 参数 (参考示例值，可根据 800Hz 调整) ================= */
// 注意：示例参数是基于 400Hz/200Hz 的。
// 如果 dt 传入正确 (0.0025 或 0.005)，这些参数可以直接使用或微调。
// 速度环 (400Hz 逻辑)
#define Kp_Vel      (-0.6f)    // 示例: 2.4e-4 (你的系统量纲不同，需大幅调整，此处为经验值)
#define Ti_Vel      (0.4f)     
#define Td_Vel      (0.0f)     

// 角度环 (400Hz 逻辑)
#define Kp_Ang      (45.0f)    // 示例: 1190
#define Ti_Ang      (0.12f)    // 示例: 0.115
#define Td_Ang      (0.0f)     

// 角速度环 (800Hz 逻辑)
#define Kp_Pal      (-20.0f)    // 示例: -2.85
#define Ti_Pal      (1000.0f)     // 示例: 0.385
#define Td_Pal      (0.0f)     
//    Ki = Kp_Pal * dt / Ti_Pal;
//    Kd = Kp_Pal * Td_Pal / dt;
// 转向环 (200Hz 逻辑)
#define Kp_Turn     (-0.8f)    // 示例: -0.365
#define Ti_Turn     (400.0f)   // 示例: 520
#define Td_Turn     (0.0f)     

/* ================= 内部静态变量 ================= */
// 速度环
static float vel_e_k_1 = 0, vel_e_k_2 = 0;
// 角度环
static float ang_e_k_1 = 0, ang_e_k_2 = 0;
// 角速度环
static float pal_e_k_1 = 0, pal_e_k_2 = 0;
// 转向环
static float turn_e_k_1 = 0;
static float turn_integral = 0;

void Balance_Init(void)
{
    vel_e_k_1 = 0; vel_e_k_2 = 0;
    ang_e_k_1 = 0; ang_e_k_2 = 0;
    pal_e_k_1 = 0; pal_e_k_2 = 0;
    turn_e_k_1 = 0; turn_integral = 0;
}

/**
 * @brief 速度环 (模仿示例签名)
 * @param speed 目标速度
 * @param L, R 电机反馈 (开环步进电机可传 0 或使用 last_pwm)
 * @param dt 实际时间间隔
 * @param clr 清零标志
 */
float Vel_Loop(int32_t speed, int32_t L, int32_t R, float dt)
{
    float output, e_k;
    float Ki, Kd;

    if (Flag_Stop == 1) {
        vel_e_k_1 = 0; vel_e_k_2 = 0;
        return 0;
    }

    

    // 示例逻辑：误差 = 目标 + Trim - 估算速度
    // 开环系统中，(L+R)/2 近似为速度，或者直接用 0 (纯积分控制)
    // 这里保留示例公式，但如果你没有编码器，L 和 R 传 0 即可，此时变成纯 I 控制
    float estimated_speed = (float)(L + R) / 2.0f; 
    // 如果没有编码器反馈，可以改用上一轮 output 滤波作为估算 (如之前代码)
    // 为了严格匹配示例接口，我们先用示例公式。若效果不好，可在 control.c 传伪值。
    
    e_k = (float)speed + (float)Speed_Trim - estimated_speed;

    Ki = Kp_Vel * dt / Ti_Vel;
    Kd = Kp_Vel * Td_Vel / dt;

    output = Kp_Vel * (e_k - vel_e_k_1) + Ki * e_k + Kd * (e_k - 2 * vel_e_k_1 + vel_e_k_2);

    vel_e_k_2 = vel_e_k_1;
    vel_e_k_1 = e_k;

    return output;
}

/**
 * @brief 角度环
 */
int32_t Ang_Loop(float target_ang,float measure_ang, float dt)
{
    float e_k, output;
    float Ki, Kd;

    if (Flag_Stop == 1) {
        ang_e_k_1 = 0; ang_e_k_2 = 0;
        return 0;
    }

    

    // 示例逻辑：误差 = 目标角度 - 实际 Pitch
    e_k = target_ang - measure_ang;

    Ki = Kp_Ang * dt / Ti_Ang;
    Kd = Kp_Ang * Td_Ang / dt;

    output = Kp_Ang * (e_k - ang_e_k_1) + Ki * e_k + Kd * (e_k - 2 * ang_e_k_1 + ang_e_k_2);

    ang_e_k_2 = ang_e_k_1;
    ang_e_k_1 = e_k;

    return (int32_t)output;
}

/**
 * @brief 角速度环
 */
int32_t Pal_Loop(int32_t target_pal, float measure_pal, float dt )
{
    float e_k, output;
    float Ki, Kd;

    if (Flag_Stop == 1) {
        pal_e_k_1 = 0; pal_e_k_2 = 0;
        return 0;
    }

    

    // 示例逻辑：误差 = 目标角速度 - 实际 Gyro[1] (Pitch 轴)
    // 注意：你的车如果是前后平衡，且用 Roll 轴平衡，请改为 gyrox
    // 根据之前代码逻辑：current_angle = -roll, 所以这里应该用 gyrox
   
    
    e_k = (float)target_pal - measure_pal;

    Ki = Kp_Pal * dt / Ti_Pal;
    Kd = Kp_Pal * Td_Pal / dt;

    output = Kp_Pal * (e_k - pal_e_k_1) + Ki * e_k + Kd * (e_k - 2 * pal_e_k_1 + pal_e_k_2);

    pal_e_k_2 = pal_e_k_1;
    pal_e_k_1 = e_k;

    return (int32_t)(output * 0.80f);
}

/**
 * @brief 转向环
 */
int32_t Turn_Loop(float gyrozixl, float dt)
{
    float error, output;
    float Ki, Kd;

    if (Flag_Stop == 1) {
        turn_e_k_1 = 0; turn_integral = 0;
        return 0;
    }

    

    // 示例逻辑：误差 = (Omega_Turn + Trim) - 实际 Gyro[2] (Yaw 轴)
    error = (float)Omega_Turn + (float)Turn_Trim - (float)gyrozixl;

    Ki = Kp_Turn * dt / Ti_Turn;
    Kd = Kp_Turn * Td_Turn / dt;

    // 示例中 turn_integral 似乎是累加的，但原代码注释了 error_sum += 0
    // 这里我们恢复正常的积分累加
    turn_integral += error;
    // 限幅
    if (turn_integral > 2000) turn_integral = 2000;
    if (turn_integral < -2000) turn_integral = -2000;

    output = Kp_Turn * error + Ki * turn_integral + Kd * (error - turn_e_k_1);
    
    turn_e_k_1 = error;

    return (int32_t)output;
}
