//#ifndef __MOTOR_H
//#define __MOTOR_H
//#include <sys.h>	 
//  /**************************************************************************
//作者：平衡小车之家
//我的淘宝小店：http://shop114407458.taobao.com/
//**************************************************************************/

//#define Left_Direction   PAout(3) //设定左电机旋转方向
//#define Right_Direction  PAout(4)  //设定右电机旋转方向
//#define ST   PBout(14) //失能电机
//void MiniBalance_PWM_Init(void);
//void MiniBalance_Motor_Init(void);


//#endif
/* motor.h */
#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "tim.h"
#include "gpio.h"
#include <sys.h>
/* ================= 用户配置区 ================= */

// 1. 定时器时钟配置 (必须与 tim.c 中的 Prescaler 设置一致)
// 假设系统时钟 100MHz, Prescaler = 9 -> 计数频率 = 10MHz (10,000,000 Hz)
#define TIM_CLOCK_FREQ      10000000UL  
#define TIM_PRESCALER       9

// 2. 电机方向引脚定义 (请根据实际电路连接修改)
// 左电机 (Moto1) 方向引脚
#define DIR_LEFT_PORT       GPIOA   // 示例：GPIOB
#define DIR_LEFT_PIN        GPIO_PIN_3 // 示例：PB0

// 右电机 (Moto2) 方向引脚
#define DIR_RIGHT_PORT      GPIOA   // 示例：GPIOB
#define DIR_RIGHT_PIN       GPIO_PIN_4 // 示例：PB1

// 3. 频率限制 (保护电机驱动器)
#define MOTOR_MAX_FREQ      20000   // 最大脉冲频率 20kHz
#define MOTOR_MIN_START     200     // 最小启动频率，低于此值视为停止


#define ST   PBout(14) //失能电机

/* ================= 函数声明 ================= */

/**
 * @brief 电机驱动刷新函数 (带参数版本)
 * 
 * 根据传入的速度值，自动计算 PWM 频率并设置方向引脚。
 * 此函数应在控制循环 (如 800Hz 定时中断) 中高频调用。
 * 
 * @param moto1: 左电机目标速度 (单位：Hz / 脉冲数每秒)
 *               - 正值：正转 (DIR_PIN = Low, 具体视电路而定)
 *               - 负值：反转 (DIR_PIN = High)
 *               - 0   : 停止 (PWM 占空比置 0)
 * 
 * @param moto2: 右电机目标速度 (单位：Hz / 脉冲数每秒)
 *               - 同上
 */
void Motor_Refresh_Drive(int moto1, int moto2);

/**
 * @brief 电机系统初始化函数
 * 
 * 配置 GPIO 方向引脚，并启动 TIM1 的 PWM 输出通道。
 * 此函数应在 main() 初始化阶段调用一次。
 */
void Motor_Init_System(void);

#endif /* __MOTOR_H */
