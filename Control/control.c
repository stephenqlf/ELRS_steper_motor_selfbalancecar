/* control.c */
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"
#include "stm32f4xx.h"
#include "oldtype.h"
#include "motor.h"
#include "control.h"
#include "MPU6050.h"
#include "printf.h"
#include "crsf_parse.h"
#include "balance.h"

/* ================= 配置区域 ================= */
#define MAX_MOTO        (13000)
#define MOTO_DEADZONE   (100)
#define CRSF_CHANNEL_VALUE_MIN  172
#define CRSF_CHANNEL_VALUE_MAX  1811
/* ================= 全局变量声明 ================= */
extern u8 Flag_Stop;
extern float pitch, roll, yaw;
extern  short gyrox, gyroy, gyroz;

extern int Remoter_Ch1, Remoter_Ch2, Arm_ch6;
extern u8 Flag_Qian, Flag_Hou, Flag_Left, Flag_Right, Flag_sudu;
extern int Moto1, Moto2; 
extern int Target_Velocity;

extern u8 mpu_dmp_flag;
extern float Zhongzhi;
extern u8 Flag_Zhongzhi;

// 【新增】模仿示例程序的全局变量
int32_t Speed_Trim = 0;
int32_t Turn_Trim = 0;
int32_t Omega_Turn = 0;

u8 mpu6050_data_flag;
float AutoZero_SumAngle = 0.0f;
int AutoZero_Count = 0;
u8 AutoZero_InProgress = 0;
static int start_delay_cnt = 0;


const float Math_PI = 3.1415926;

/* ================= 中断级静态变量 ================= */
static uint8_t vel_cnt = 0;
static uint8_t ang_cnt = 0;
static uint8_t turn_cnt = 0;

static float target_ang = 0.0f;
static int32_t target_pal = 0;
static int32_t pwm_pal = 0;
static int32_t pwm_turn = 0;

static float roll_1 = 0, roll_2 = 0;
static float gyro_1 = 0, gyro_2 = 0;

#define DT_800HZ  (0.00125f)
#define DT_400HZ  (0.00250f)
#define DT_200HZ  (0.00500f)

/* ================= 函数声明 ================= */
void Auto_Calibrate_Zhongzhi(void);
int myabs(int a);
void Get_Elrs(void);
u8 click(void);
u8 Turn_Off(float angle);
// void Set_Pwm(int moto1, int moto2); // <--- 删除此声明，不再需要
u16 Linear_Conversion(int moto);
int map(int val, int I_Min, int I_Max, int O_Min, int O_Max);

/* ================= 主控制循环 (800Hz) ================= */
void ControlLoopPackage()
{
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    // 1. 读取传感器 & 平滑
    mpu_dmp_get_data(&gyrox, &gyroy, &gyroz, &pitch, &roll, &yaw);        //更新欧拉角数据
    //MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);	 //更新陀螺仪数据,这里读到的是原始16位short型数据，范围从-32,768 ~ 32,767， 这正确吗？陀螺仪数据不应该是度/秒才对吗，要检查一下参考程序是取的什么值，另外这个值要转化的话，应该还与量程有关，这个值大小与kp的取值大小密切相关陀螺仪传感器,±2000dps
	
	
//	// 假设 gyrox 是读出来的 short 类型原始数据 (范围 -32768 ~ 32767)
//short gyrox_raw = ...; 

//// 方法 A: 直接计算 (推荐，简单直观)
//float gyro_x_deg_per_sec = (float)gyrox_raw / 16.4f;

//// 方法 B: 使用乘法优化 (在极老旧的单片机上为了省除法耗时，但在 STM32F4 上没必要)
//// float gyro_x_deg_per_sec = (float)gyrox_raw * 0.0609756f; // 0.0609756 ≈ 1/16.4

//printf("角速度: %.2f °/s\r\n", gyro_x_deg_per_sec);
	
	
	
    float tmp_roll = (roll + roll_1 + roll_2) / 3.0f;
    roll_2 = roll_1;
    roll_1 = roll;
    roll = tmp_roll;

    float tmp_gyro = ((float)gyrox + gyro_1 + gyro_2) / 3.0f;
    gyro_2 = gyro_1;
    gyro_1 = (float)gyrox;
    gyrox = (short)tmp_gyro;
    printf("roll is %f, gyrox is %d \r\n", roll, gyrox);
	
    // 2. 读取遥控器数据
    Get_Elrs();
   // CRSF_Debug();
	
	
    // 3. 启动自平衡按键
    if (click() == 1)
    {
        Flag_Stop = 0;

        Balance_Init();//pid数据累积项清零
        target_ang = 0;
        target_pal = 0;
        pwm_pal = 0;
        pwm_turn = 0;
        // 假设 PB3 是状态指示灯灭
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
    }

    if (Flag_Stop == 0)
    {
        Auto_Calibrate_Zhongzhi();

        // --- 准备输入 ---
        int32_t speed_cmd = 0;
        if (Flag_Qian) speed_cmd = Target_Velocity;
        else if (Flag_Hou) speed_cmd = -Target_Velocity;

        if (Flag_Left) Omega_Turn = -400;
        else if (Flag_Right) Omega_Turn = 400;
        else Omega_Turn = 0;

        // --- PID 计算 (保持原有逻辑) ---

		//400hz
        if (++vel_cnt >= 2)
        {
            vel_cnt = 0;
            
           target_ang += Vel_Loop(speed_cmd, Moto1, Moto2, DT_400HZ);

			//这里是不是错了，这是增量式pid计算，应该是在上次值的基础上加上这次的pid输出，下面的逻辑是每次都从中值开始加
//			if (vel_cnt == 0) target_ang = Zhongzhi;
//            target_ang += vel_out;
			
			
			
            if (target_ang > Zhongzhi + 20.0f) target_ang = Zhongzhi + 20.0f;
            if (target_ang < Zhongzhi - 20.0f) target_ang = Zhongzhi - 20.0f;
        }
		//400hz
        if (++ang_cnt >= 2)
        {
            ang_cnt = 0;
			//这里的角度目标值应该要减去中值来修正，我改成如下：
            target_pal += Ang_Loop(target_ang, roll, DT_400HZ);
        }
		//800hz
        pwm_pal  += Pal_Loop(0, gyrox, DT_800HZ);

        //pwm_pal = Pal_Loop(target_pal, gyrox, DT_800HZ);


		//200hz
        if (++turn_cnt >= 4)
        {
            turn_cnt = 0;
            pwm_turn = Turn_Loop(gyroz, DT_200HZ);
        }


        // --- 合成输出 ---
        int32_t pwm_L = pwm_pal ;
        int32_t pwm_R = pwm_pal ;
       //printf("pwm_L is %d  \r\n", pwm_pal);

        //		int32_t pwm_L = pwm_pal + pwm_turn;
        //        int32_t pwm_R = pwm_pal - pwm_turn;
        //


        // 限幅
        if (pwm_L > MAX_MOTO) pwm_L = MAX_MOTO;
        if (pwm_L < -MAX_MOTO) pwm_L = -MAX_MOTO;
        if (pwm_R > MAX_MOTO) pwm_R = MAX_MOTO;
        if (pwm_R < -MAX_MOTO) pwm_R = -MAX_MOTO;

        // 死区
        if (pwm_L != 0 && myabs(pwm_L) < MOTO_DEADZONE) //小于100就取100
            pwm_L = (pwm_L > 0) ? MOTO_DEADZONE : -MOTO_DEADZONE;
        if (pwm_R != 0 && myabs(pwm_R) < MOTO_DEADZONE)
            pwm_R = (pwm_R > 0) ? MOTO_DEADZONE : -MOTO_DEADZONE;

        Moto1 = pwm_L;
        Moto2 = pwm_R;
       // printf("M1 is %d  \r\n", Moto1);
        // --- 执行驱动 (关键修改) ---
        // 不再使用 Set_Pwm 和 ST 变量，直接调用新函数
        if (Turn_Off(roll) == 0)
        {
            // 正常运行
            Motor_Refresh_Drive(Moto1, Moto2);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); // 绿灯/运行指示
            ST = 0;                // 电机使能
        }
        else
        {
            // 保护停止
            ST = 1; // 电机失能
            Motor_Refresh_Drive(0, 0);
            Flag_Stop = 1;
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET); // 黄led灯亮，停止指示
        }
    }
    else
    {
        // 已停止状态
        Motor_Refresh_Drive(0, 0);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

        // 重置 PID 状态
        Balance_Init();
        target_ang = 0;
        target_pal = 0;
        pwm_pal = 0;
        pwm_turn = 0;
        vel_cnt = 0;
        ang_cnt = 0;
        turn_cnt = 0;
    }

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

/* ================= 辅助函数实现 ================= */

void Auto_Calibrate_Zhongzhi(void)
{
    if (Flag_Zhongzhi == 1) return;
    if (fabsf(roll) > 15.0f)
    {
        AutoZero_SumAngle = 0;
        AutoZero_Count = 0;
        AutoZero_InProgress = 0;
        return;
    }
    if (Flag_Stop == 0) start_delay_cnt++;
    if (start_delay_cnt > 400 && fabsf(roll) < 10.0f) AutoZero_InProgress = 1;
    if (AutoZero_InProgress)
    {
        AutoZero_SumAngle += (roll);
        AutoZero_Count++;
        if (AutoZero_Count >= 2000)
        {
            Zhongzhi = AutoZero_SumAngle / AutoZero_Count;
            Flag_Zhongzhi = 1;
            AutoZero_SumAngle = 0;
            AutoZero_Count = 0;
            AutoZero_InProgress = 0;
            //printf("Zhongzhi: %f\r\n", Zhongzhi);
        }
    }
}





u8 Turn_Off(float angle)
{
    u8 temp;
    // 解锁条件：通道6在中间，且角度小于20度
    if(Arm_ch6 > 800 && Arm_ch6 < 1400 && fabsf(angle) < 20.0f)
    {
        temp = 0;
    }
    else
    {
        temp = 1;
        // 可选：打印调试信息，频率不要太高
        printf("Arm: %d, Angle: %f\r\n", Arm_ch6, angle);
    }
    return temp;
}

u8 click(void)
{
    static u8 k = 1;
    // 假设 KEY 是在 gpio.h 或 main.h 中定义的宏
    if(k && KEY == 0)
    {
        k = 0;
        return 1;
    }
    if(KEY) k = 1;
    return 0;
}

void Get_Elrs()
{

    Remoter_Ch1 =  map(CRSF_RX_packet.CH[1], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, -1000, 1000);
    Remoter_Ch2 = map(CRSF_RX_packet.CH[3], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, -1000, 1000);
    Arm_ch6 = map(CRSF_RX_packet.CH[5], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, -1000, 1000);
    //printf( "ch1 %ld, ch2 %ld ,arm ch6 is %ld \r\n", Remoter_Ch1, Remoter_Ch2, Arm_ch6);

    float a = atan2(Remoter_Ch1, Remoter_Ch2);
    float p = sqrt(pow(Remoter_Ch1, 2) + pow(Remoter_Ch2, 2));

    if (p > 200)
    {
        if (a > Math_PI / 4 && a <= Math_PI * 3 / 4)
        {
            Flag_Qian = 1;
            Flag_Hou = 0;
            Flag_Left = 0;
            Flag_Right = 0;
        }
        else if (a > -Math_PI * 3 / 4 && a <= -Math_PI / 4)
        {
            Flag_Qian = 0;
            Flag_Hou = 1;
            Flag_Left = 0;
            Flag_Right = 0;
        }
        else if (a > -Math_PI / 4 && a <= Math_PI / 4)
        {
            Flag_Left = 0;
            Flag_Right = 1;
            Flag_Qian = 0;
            Flag_Hou = 0;
        }
        else
        {
            Flag_Left = 1;
            Flag_Right = 0;
            Flag_Qian = 0;
            Flag_Hou = 0;
        }
    }
    else
    {
        Flag_Left = 0;
        Flag_Right = 0;
        Flag_Qian = 0;
        Flag_Hou = 0;
    }
}

// Linear_Conversion 和 map 函数如果不再被外部调用（因为电机驱动逻辑移走了），也可以删除
// 但为了保留代码完整性，暂时保留，虽然它们现在可能没被用到。


int map(int val, int I_Min, int I_Max, int O_Min, int O_Max)
{
  return (int)((val - I_Min) * (O_Max - O_Min) / (I_Max - I_Min) + O_Min);
}
int myabs(int a)
{
    return (a < 0) ? -a : a;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        ControlLoopPackage();
    }
}
