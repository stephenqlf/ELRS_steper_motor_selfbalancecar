/* motor.c */
#include "motor.h"
#include "tim.h"
#include "gpio.h"
#include "math.h"

// 外部定时器句柄 (由 tim.c 定义)
extern TIM_HandleTypeDef htim1;

/**
 * @brief 单个通道频率设置辅助函数
 * @param channel: TIM_CHANNEL_1 或 TIM_CHANNEL_2
 * @param freq_hz: 目标频率 (绝对值，必须 >= 0)
 */
static void Stepper_Set_Frequency_Internal(uint32_t channel, int freq_hz)
{
    uint32_t new_arr;
    uint32_t new_ccr;
    
    // 1. 处理停止情况 (频率 <= 0 或 过低)
    // 设定一个最小启动频率，低于此值电机可能只震动不转，直接视为停止
    const int MIN_START_FREQ = 200; 
    
    if (freq_hz < MIN_START_FREQ)
    {
        // 将占空比设为 0，停止输出脉冲，但定时器保持运行
        __HAL_TIM_SET_COMPARE(&htim1, channel, 0);
        return;
    }

    // 2. 限制最大频率 (保护驱动器)
    const int MAX_FREQ = 20000; 
    if (freq_hz > MAX_FREQ) freq_hz = MAX_FREQ;

    // 3. 计算 ARR (自动重装载值)
    // 公式: Freq = TimClock / ((PSC+1) * (ARR+1))
    // 推导: ARR = (TimClock / ((PSC+1) * Freq)) - 1
    float temp = (float)TIM_CLOCK_FREQ / ((float)(TIM_PRESCALER + 1) * (float)freq_hz);
    
    // 保护：ARR 必须至少为 1 (否则无法产生 50% 占空比)
    if (temp < 2.0f) temp = 2.0f;
    
    new_arr = (uint32_t)(temp - 1.0f);
    
    // 4. 计算 CCR (比较值)，设定为 50% 占空比
    new_ccr = new_arr / 2;
    if (new_ccr == 0) new_ccr = 1;

    // 5. 写入寄存器
    // 先写 CCR，再写 ARR。由于开启了 Preload，它们会在下一个更新事件同时生效
    __HAL_TIM_SET_COMPARE(&htim1, channel, new_ccr);
    
    // 直接操作 ARR 寄存器 (HAL 库没有安全的动态 SET_ARR 宏，直接写实例寄存器最快)
    htim1.Instance->ARR = new_arr;
}

/**
 * @brief 电机驱动主刷新函数 (带参数版本)
 * @param moto1: 左电机速度 (Hz)，正负代表方向
 * @param moto2: 右电机速度 (Hz)，正负代表方向
 * 
 * 调用时机：在 PID 控制循环 (如 800Hz 中断) 中调用
 */
void Motor_Refresh_Drive(int moto1, int moto2)
{
    // --- 处理左电机 (TIM1 Channel 1) ---
    
    // 1. 设置方向引脚
    if (moto1 > 0)
    {
        HAL_GPIO_WritePin(DIR_LEFT_PORT, DIR_LEFT_PIN, GPIO_PIN_RESET); // 假设 Low = 正转
    }
    else if (moto1 < 0)
    {
        HAL_GPIO_WritePin(DIR_LEFT_PORT, DIR_LEFT_PIN, GPIO_PIN_SET);   // 假设 High = 反转
    }
    // 如果 moto1 == 0，方向保持不变，或者你可以强制设为某个默认状态
    
    // 2. 设置频率 (取绝对值)
    Stepper_Set_Frequency_Internal(TIM_CHANNEL_1, (moto1 > 0) ? moto1 : -moto1);


    // --- 处理右电机 (TIM1 Channel 2) ---
    
    // 1. 设置方向引脚
    if (moto2 > 0)
    {
        HAL_GPIO_WritePin(DIR_RIGHT_PORT, DIR_RIGHT_PIN, GPIO_PIN_RESET);
    }
    else if (moto2 < 0)
    {
        HAL_GPIO_WritePin(DIR_RIGHT_PORT, DIR_RIGHT_PIN, GPIO_PIN_SET);
    }
    
    // 2. 设置频率 (取绝对值)
    Stepper_Set_Frequency_Internal(TIM_CHANNEL_2, (moto2 > 0) ? moto2 : -moto2);
}

/**
 * @brief 电机系统初始化
 * 调用时机：在 main() 函数开头，进入 while(1) 之前
 */
void Motor_Init_System(void)
{
    // 1. 启动 PWM 输出
    // 一旦启动，硬件开始根据当前的 ARR/CCR 发波
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    
    // 2. 初始状态设为停止 (占空比 0)
    // 此时定时器在跑，但没有脉冲输出
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    
    // 3. 初始化方向引脚为安全状态 (可选)
    HAL_GPIO_WritePin(DIR_LEFT_PORT, DIR_LEFT_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIR_RIGHT_PORT, DIR_RIGHT_PIN, GPIO_PIN_RESET);
}

