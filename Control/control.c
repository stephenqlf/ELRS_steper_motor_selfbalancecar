

#include "math.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"
#include "stm32f4xx.h" // Device header
#include "oldtype.h"
#include "motor.h"
#include "control.h"
#include "MPU6050.h"
#include "printf.h"
#include "crsf_parse.h"
#include "oldtype.h"
#define PRESCALER 99
#define speedFilterConstant 0.7
#define CRSF_CHANNEL_VALUE_MIN  172
#define CRSF_CHANNEL_VALUE_MAX  1811

#define TURN_GAIN 1000
// 定义 PI
#ifndef PI
#define PI 3.14159265358979323846f
#endif


// --- 全局变量：存储解算后的角度 (单位：度) ---
float Angle_Pitch = 0.0f; // 前后倾角 (平衡车核心)
float Angle_Roll  = 0.0f; // 左右倾角
float Angle_Yaw   = 0.0f; // 航向角 (会漂移)

// --- 静态变量：保存上一时刻的角度，用于积分 ---
static float Last_Pitch = 0.0f;
static float Last_Roll  = 0.0f;
static float Last_Yaw   = 0.0f;

// --- 配置参数 ---
// 互补滤波系数：0.96 ~ 0.98 之间
// 越大越信任陀螺仪(反应快但漂移)，越小越信任加速度计(稳但受震动影响)
#define COMPLIMENTARY_ALPHA 0.98f










u8 Flag_Stop = 1, ZeroRequirementSpeedPid = 0, ZeroRequirementMean = 0;
float pitch, roll, yaw; //欧拉角
short aacx, aacy, aacz;	 //加速度传感器原始数据
short gyrox, gyroy, gyroz; //陀螺仪原始数据
extern u32 Arm_ch6;
extern int Voltage;                                              // 电池电压采样相关的变量
extern float Remoter_Ch1, Remoter_Ch2;                           // 航模遥控接收变量
extern float Balance_Kp, Balance_Kd, Velocity_Kp, Velocity_Ki;   // PID参数
extern u8 Flag_Qian, Flag_Hou, Flag_Left, Flag_Right, Flag_sudu; // 蓝牙遥控相关的变量
extern int Moto1, Moto2, Final_Moto1, Final_Moto2;               // 电机PWM变量 应是Motor的 向Moto致敬
extern int Target_Velocity;
extern u8 Flag_Qian, Flag_Hou, Flag_Left, Flag_Right;            // 蓝牙遥控相关的变量                //停止标志位和 显示标志位 默认停止 显示打开

extern float Acceleration_Z; // Z轴加速度计
extern int _channels[];
extern int MPU_Flag;
extern int Rx_Available_Flag;
extern int Maximum_delta_speed;
u8 mpu6050_data_flag;  //是否正常读取数据
float xpeed = 0.0;
const float Math_PI = 3.1415926;
int DMP_Ready_Flag = 0;
extern u8 mpu_dmp_flag;


extern float Zhongzhi ;
u8 Flag_Zhongzhi = 0; // 0: 未校准，1: 已校准

// 自动找中值专用变量
float AutoZero_SumAngle = 0.0f;
int AutoZero_Count = 0;
u8 AutoZero_InProgress = 0;


//************MPU6050相关数据读取********************
void MPU6050_Data_read(void)
{
    mpu6050_data_flag = mpu_dmp_get_data(&pitch, &roll, &yaw);  //得到角度数据
    //MPU_Get_Accelerometer(&aacx,&aacy,&aacz); //得到加速度传感器数据
    //printf("roll angle is %f \r\n ", roll);

}
/**************************************************************************
函数功能：按键扫描
入口参数：双击等待时间
返回  值：按键状态 0：无动作 1：单击 2：双击
**************************************************************************/
u8 click_N_Double (u8 time)
{
    static	u8 flag_key, count_key, double_key;
    static	u16 count_single, Forever_count;
    if(KEY == 0)  Forever_count++; //长按标志位未置1
    else        Forever_count = 0;
    if(0 == KEY && 0 == flag_key)		flag_key = 1;
    if(0 == count_key)
    {
        if(flag_key == 1)
        {
            double_key++;
            count_key = 1;
        }
        if(double_key == 2)
        {
            double_key = 0;
            count_single = 0;
            return 2;//双击执行的指令
        }
    }
    if(1 == KEY)			flag_key = 0, count_key = 0;

    if(1 == double_key)
    {
        count_single++;
        if(count_single > time && Forever_count < time)
        {
            double_key = 0;
            count_single = 0;
            return 1;//单击执行的指令
        }
        if(Forever_count > time)
        {
            double_key = 0;
            count_single = 0;
        }
    }
    return 0;
}
/**************************************************************************
函数功能：按键扫描
入口参数：无
返回  值：按键状态 0：无动作 1：单击
**************************************************************************/
u8 click(void)
{
    static u8 flag_key = 1; //按键按松开标志
    if(flag_key && KEY == 0)
    {
        flag_key = 0;
        return 1;	// 按键按下
    }
    else if(1 == KEY)			flag_key = 1;
    return 0;//无按键按下
}
/**************************************************************************
函数功能：长按检测
入口参数：无
返回  值：按键状态 0：无动作 1：长按2s
**************************************************************************/
u8 Long_Press(void)
{
    static u16 Long_Press_count, Long_Press;
    if(Long_Press == 0 && KEY == 0)  Long_Press_count++; //长按标志位未置1
    else                       Long_Press_count = 0;
    if(Long_Press_count > 400)
    {
        Long_Press = 1;
        Long_Press_count = 0;
        return 1;
    }
    if(Long_Press == 1)   //长按标志位置1
    {
        Long_Press = 0;
    }
    return 0;
}


/**************************************************************************
函数功能：直立PD控制
入口参数：角度
返回  值：直立控制PWM

**************************************************************************/
int balance(float Angle, float Gyro)
{
    float Bias; // 这里D为零

    // AI指出的解决方案-根据前进/后退速度动态调整平衡中值
    //float Dynamic_Zhongzhi = Zhongzhi - Movement * 0.001f; // 前进时目标角度略后仰
    //balanceValue = Balance_Kp * (Angle - Dynamic_Zhongzhi) + Balance_Kd * Gyro;


    int balanceValue;
    Bias =  Zhongzhi - Angle ; //===求出平衡的角度中值 和机械相关

    balanceValue = (int)(Balance_Kp * Bias + Balance_Kd * Gyro); //===计算平衡控制的电机PWM
    // printf("%f, %d\r\n",Bias, balanceValue);
    return balanceValue;
}


/**************************************************************************
函数功能：速度PI控制 (改进版：软启动 + 积分抗静摩擦 + 噪声滤波)
入口参数：左轮速度、右轮速度 (实际未直接使用，用于兼容接口)
返回  值：速度控制PWM
**************************************************************************/
int velocity(int velocity_left, int velocity_right)
{
    // --- 1. 静态变量定义 ---
    static float SpeedError_Integral = 0;       // 积分项
    static float Estimated_Speed = 0;           // 估算的实际速度
    static float Smooth_Target_Velocity = 0;    // 平滑后的目标速度 (软启动用)
    static float Last_Velocity_Output = 0;      // 上一轮速度环输出

    // --- 2. 参数配置区 (根据实际效果微调这里) ---
    const float Kp = -0.3f;         // 比例系数：负值！绝对值小一点保稳定 (0.2 ~ 0.4)
    const float Ki = -0.08f;        // 积分系数：负值！负责克服静摩擦 (0.05 ~ 0.15)
    const float Ramp_Step = 40.0f;  // 软启动步长：越大起步越猛，越小越平滑 (20 ~ 60)
    const float Integral_Limit = 8000.0f; // 积分限幅：防止积分过大导致失控
    const float Dead_Zone = 50.0f;  // 死区：目标速度小于此值视为停止

    float Raw_Target = 0;
    float Velocity_Output = 0;

    // --- 3. 获取原始目标速度 ---
    if (Flag_Qian)
        Raw_Target = (float)Target_Velocity;
    else if (Flag_Hou)
        Raw_Target = -(float)Target_Velocity;
    else
        Raw_Target = 0;

    // --- 4. 软启动逻辑 (斜坡函数) ---
    // 让 Smooth_Target 慢慢接近 Raw_Target，避免突变
    if (Raw_Target > Smooth_Target_Velocity)
    {
        Smooth_Target_Velocity += Ramp_Step;
        if (Smooth_Target_Velocity > Raw_Target) Smooth_Target_Velocity = Raw_Target;
    }
    else if (Raw_Target < Smooth_Target_Velocity)
    {
        Smooth_Target_Velocity -= Ramp_Step;
        if (Smooth_Target_Velocity < Raw_Target) Smooth_Target_Velocity = Raw_Target;
    }

    // --- 5. 死区处理 (停车时彻底清零) ---
    if (fabsf(Smooth_Target_Velocity) < Dead_Zone && fabsf(Raw_Target) < Dead_Zone)
    {
        Smooth_Target_Velocity = 0;
        SpeedError_Integral = 0;      // 停车清除积分，防止累积
        Estimated_Speed = 0;
        Last_Velocity_Output = 0;
        return 0; // 直接返回0，让车停稳
    }

    // --- 6. 估算实际速度 (简化模型) ---
    // 开环步进电机没有编码器，我们用“上一轮输出”经过低通滤波来模拟实际速度
    // 这比使用 Moto1+Moto2 更干净，因为 Moto 里包含直立环的高频抖动
    Estimated_Speed = Estimated_Speed * 0.92f + Last_Velocity_Output * 0.08f;

    // --- 7. 计算误差 ---
    float SpeedError = Smooth_Target_Velocity - Estimated_Speed;

    // --- 8. 积分运算 ---
    SpeedError_Integral += SpeedError;

    // 积分限幅 (非常重要，防止累积过大)
    if (SpeedError_Integral > Integral_Limit) SpeedError_Integral = Integral_Limit;
    if (SpeedError_Integral < -Integral_Limit) SpeedError_Integral = -Integral_Limit;

    // --- 9. 计算最终输出 (P + I) ---
    // 注意：Kp 和 Ki 都是负数，实现“想前进先退轮”的平衡车逻辑
    Velocity_Output = (SpeedError * Kp) + (SpeedError_Integral * Ki);

    // 保存当前输出供下一轮估算使用
    Last_Velocity_Output = Velocity_Output;

    return (int)Velocity_Output;






    /*原来的方式*/
    //    static float SpeedError_Least = 0, SpeedError_Integral = 0, SpeedError = 0, Movement;
    //    static int Velocity;
    //

    //    if (1 == Flag_Qian)
    //        Movement = Target_Velocity; //===前进标志位置1
    //    else if (1 == Flag_Hou)
    //        Movement = -Target_Velocity; //===后退标志位置1
    //    else
    //        Movement = 0;

    //    // printf("%f\r\n",Movement);
    //    if (ZeroRequirementSpeedPid == 1)  //这里有问题，几个地方有Turn_off
    //    {
    //        SpeedError = 0;
    //        SpeedError_Integral = 0; //===电机关闭后清除积分
    //        ZeroRequirementSpeedPid = 0;
    //        // printf("%d\r\n",Velocity);
    //    }

    //    //=============速度PI控制器=======================//
    //    SpeedError_Least = Mean_Filter(velocity_left,velocity_right) ; // 上一个循环的速度滤波 , 此处究竟是加上Movement 还是减去Movement 应该与正负极性有关，须在验证极性后再来确定，与Kp, Ki值一样需要验证极性Mean_Filter(velocity_left, velocity_right) - Movement;

    //    SpeedError *= 0.7f;                 //===一阶低通滤波器
    //    SpeedError += SpeedError_Least * 0.3f; //===一阶低通滤波器


    //    SpeedError_Integral += SpeedError;     //===积分出位移
    //    SpeedError_Integral += Movement;    //===接收遥控器数据，控制前进后退
    //    if (SpeedError_Integral > 320000) //
    //        SpeedError_Integral = 320000; //===积分限幅，输出限幅在8000左右，怎么积分限幅可以这么大？？？？
    //    if (SpeedError_Integral < -320000)
    //        SpeedError_Integral = -320000; //===积分限幅

    //    Velocity = (int)(SpeedError * Velocity_Kp/100  + SpeedError_Integral * Velocity_Ki / 100 ); //===速度控制

    //    return Velocity;
}

/**************************************************************************
函数功能：转向控制 (重构版：比例型 + 平滑滤波，彻底解决抖动)
入口参数：velocity_left, velocity_right (未直接使用，仅兼容接口)
返回  值：转向控制PWM (左轮减去此值，右轮加上此值)
**************************************************************************/
int turn(int velocity_left, int velocity_right)
{

    if (Flag_Left)
    {
        return -TURN_GAIN; // 左转：负值
    }
    else if (Flag_Right)
    {
        return TURN_GAIN;  // 右转：正值
    }
    else
    {
        return 0;          // 松手：目标归零
    }



}

/**************************************************************************
函数功能：赋值给PWM寄存器,并且判断转向
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int moto1, int moto2)
{
    /*这里没有说moto1和moto2等于0时怎么办*/

    if (moto1 > 0)
        Right_Direction = 0;
    else
        Right_Direction = 1;
    if (moto2 > 0)
        Left_Direction = 0;
    else
        Left_Direction = 1;


    Final_Moto1 = Linear_Conversion(moto1); // 线性化
    Final_Moto2 = Linear_Conversion(moto2);
    // printf("M1 is %d  \r\n", Final_Moto1);
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：倾角和电压
返回  值：1：异常  0：正常
**************************************************************************/

u8 Turn_Off(float angle)
{
    u8 temp;



    if(Arm_ch6 > 800 && Arm_ch6 < 1200 && fabsf(angle) < 20.0f ) // 解锁按钮在解锁位置,竖直状态  myabs(angle) < 50,Angle 0.00000时，这个比较反馈为False，不知道为什么
    {

        temp = 0;


    }
    else
    {
        temp = 1;
        printf("Arm value %ld, Angle value %f \r\n", Arm_ch6, angle);


    }




    return temp;
}
/**************************************************************************
函数功能：检测小车是否被拿起
入口参数：int
返回  值：unsigned int
**************************************************************************/
int Pick_Up(float Acceleration, float Angle, int encoder_left, int encoder_right)
{
    static u16 flag, count0, count1, count2;
    if(flag == 0)                                                                 //第一步
    {
        if(myabs(encoder_left) + myabs(encoder_right) < 700)                     //条件1，小车接近静止
            count0++;
        else
            count0 = 0;
        if(count0 > 10)
            flag = 1, count0 = 0;
    }
    if(flag == 1)                                                                //进入第二步
    {
        if(++count1 > 400)       count1 = 0, flag = 0;                          //超时不再等待2000ms
        if(Acceleration > 28000 && (Angle > (-20 + Zhongzhi)) && (Angle < (20 + Zhongzhi)) && Flag_Qian != 1 && Flag_Hou != 1 && Flag_Left != 1 && Flag_Right != 1) //条件2，小车是在0度附近被拿起
            flag = 2;
    }
    if(flag == 2)                                                                //第三步
    {
        if(++count2 > 200)       count2 = 0, flag = 0;                            //超时不再等待1000ms
        if(myabs(encoder_left + encoder_right) > 9900)                             //条件3，小车的轮胎因为正反馈达到最大的转速
        {
            flag = 0;
            return 1;                                                               //检测到小车被拿起
        }
    }
    return 0;
}

/**************************************************************************
函数功能：采集遥控器的信号
入口参数：无
返回  值：无
**************************************************************************/
void Get_Elrs()
{

    Remoter_Ch1 = map(CRSF_RX_packet.CH[1], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, -1000, 1000);
    Remoter_Ch2 =  map(CRSF_RX_packet.CH[3], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, -1000, 1000);
    Arm_ch6 = map(CRSF_RX_packet.CH[5], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, -1000, 1000);


    // printf( "ch1 %f, ch2 %f ,arm ch6 is %f \r\n", Remoter_Ch1,Remoter_Ch2,Arm_ch6);



    float a = atan2(Remoter_Ch1, Remoter_Ch2);
    float p = sqrt(pow(Remoter_Ch1, 2) + pow(Remoter_Ch2, 2));

    if (p > 200)
    {
        if (a > Math_PI / 4 && a <= Math_PI / 4 * 3)
        {
            Flag_Qian = 1, Flag_Hou = 0;
            Flag_Left = 0, Flag_Right = 0;
            // printf("前进\r\n");
        }
        else if (a > -Math_PI / 4 * 3 && a <= -Math_PI / 4)
        {
            Flag_Qian = 0, Flag_Hou = 1;
            Flag_Left = 0, Flag_Right = 0;
            // printf("后退\r\n");
        }
        else if (a > -Math_PI / 4 && a <= Math_PI / 4)
        {
            Flag_Left = 0, Flag_Right = 1;
            Flag_Qian = 0, Flag_Hou = 0;
            // printf("右转\r\n");
        }
        else
        {
            Flag_Left = 1, Flag_Right = 0;
            Flag_Qian = 0, Flag_Hou = 0;

            // printf("左转\r\n");
        }
    }
    else if (p < 200)

    {
        Flag_Left = 0, Flag_Right = 0;
        Flag_Qian = 0, Flag_Hou = 0;

        // printf("停止\r\n");
    }


}


/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{
    int temp;
    if (a < 0)
        temp = -a;
    else
        temp = a;
    return temp;
}

/**************************************************************************
函数功能：速度滤波
入口参数：速度
返回  值：滤波后的速度,求过去10次速度的平均值

#include "control.h"
#include "filter.h"
**************************************************************************/
int Mean_Filter(int moto1, int moto2)
{
    u8 i;
    s32 Sum_Speed = 0;
    s16 Filter_Speed;
    static s16 Speed_Buf[FILTERING_TIMES] = {0};
    if (ZeroRequirementMean == 1)   //这里有问题，几个地方都有Turn_off
    {
        memset(Speed_Buf, 0, sizeof(Speed_Buf)); //倾倒后就把过去10次记录的速度值全部清零
        ZeroRequirementMean = 0;
    }



    for (i = 1; i < FILTERING_TIMES; i++)
    {
        Speed_Buf[i - 1] = Speed_Buf[i];
    }
    Speed_Buf[FILTERING_TIMES - 1] = moto1 + moto2;

    for (i = 0; i < FILTERING_TIMES; i++)
    {
        Sum_Speed += Speed_Buf[i];
    }
    Filter_Speed = (s16)(Sum_Speed / FILTERING_TIMES);
    return Filter_Speed;
}

/**************************************************************************
函数功能：对控制输出的PWM线性化,便于给系统寄存器赋值
入口参数：PWM
返回  值：线性化后的PWM，小值变大，大值变小，其结果是moto值相当于转速，转速越小，CCR值越大, 本工程使用的F411芯片，TIM1连到APB2上，速度是100MHz
**************************************************************************/
u16 Linear_Conversion(int moto)
{
    u32 temp;
    u16 Linear_Moto;
    if (abs(moto) < 10) moto = 0;
    if (moto == 0)
    {
        return 65535; // 或一个极大值，让频率≈0
    }

    temp = 100000000 / (99 + 1) / myabs(moto) / 2; //timer oc toggle 两个周期才能输出一个完整的脉冲，驱动步进电机的一个微步
    if (temp > 65535)
        Linear_Moto = 65535;
    // if(temp > 4000) Linear_Moto = 4000;//计时器的频率为100万每秒，1ms只能读数到1000，5ms只能读数到5000，超过5000的值则在一个控制周期内还未读完，这里ccr值又变大,可能导致永远也不跳转
    else
        Linear_Moto = (u16)temp;
    //    if (temp < 75)
    //        Linear_Moto = 75; // 最小脉冲个数值60个，比这更低马达功率的扭矩不够，转不起来
    //    else
    //        Linear_Moto = (u16)temp;
    static int print_count = 0;
    print_count++;
    if (print_count == 50)
    {
        print_count = 0;

        //printf("Linear_Moto is %d \r\n", Linear_Moto);
    }
    return Linear_Moto; // 计算结果会用1000000/Linear_Moto得到每秒能发多少个脉冲，然后/16/200,就是多少n/s，如果最大转速为
}

/**************************************************************************
函数功能：控制主逻辑
入口参数：无
返回  值：无
**************************************************************************/



void ControlLoopPackage()
{
    //pid计算频率越高越好，做到非常稳定的是达到1ms一次，1000Hz的频率，那么就不能用DMP来获取值了，只能自己解算姿态，芯片能不能达到计算的能力，后面可以试试看别的姿态解算方法，提高频率
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); //最后翻转PC13LED灯,示波器检查pid计算能否在1ms内完成

    static int last_Moto1 = 0, last_Moto2 = 0;

    last_Moto1 = Moto1;
    last_Moto2 = Moto2;



//    if(DMP_Ready_Flag == 1)
//    {
//        //Get_Angle(); //===更新姿态, DMP模式是FIFO,如果有数据不读，堵塞住了后是不是就再也读不到新数据了，确实是这样的
//        //MPU6050_Data_read();	//获取陀螺仪数据	每5ms读取一次
//        Get_Elrs(); //===读取航模遥控器的数据
//        DMP_Ready_Flag = 0;


//    }

    if ( click() == 1 )  //((Flag_Stop == 1) && (click() == 1) )
    {
        Flag_Stop = 0;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); //黄色把警灯熄灭

    }


    if (Flag_Stop == 0)
    {
        static float dt = 0.001f; // 1ms = 0.001秒

        // 1. 读取传感器数据 (假设你的读取函数已填充这些变量)
        // 确保读取的是最新的数据

        MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz); //更新陀螺仪数据

        MPU_Get_Accelerometer(&aacx, &aacy, &aacz); //更新加速度计数据

        // 2. 解算角度
        MPU6050_Solve_Angle(aacx, aacy, aacz, gyrox, gyroy, gyroz, dt);



        // Kalman_Filter(Accel_Angle, Gyro_X); //卡尔曼滤波


        //Yijielvbo(Accel_Angle,Gyro_X);    //互补滤波

        //得到  -roll和gyrox;X轴朝前，Y轴朝左，Z轴朝上

        // 3. 使用角度进行控制
        // Angle_Pitch 是你直立环需要的核心角度 (前后)
        // Angle_Roll 可用于限制最大倾斜或防侧翻
        // gyroz (或 Angle_Yaw 的变化率) 用于转向环

        // 示例：打印调试
        printf("Pitch: %.2f, Roll: %.2f\n", Angle_Pitch, Angle_Roll);
  Get_Elrs(); //===读取航模遥控器的数据


        //printf("Gyro_y is %d \r\n", gyrox);
        Balance_Pwm = balance(-Angle_Roll, gyrox); //===平衡控制 balanceValue = (int)(Balance_Kp * Bias + Balance_Kd * Gyro); //===计算平衡控制的电机PWM
        Velocity_Pwm = velocity(Moto1, Moto2);
        Turn_Pwm = turn(Moto1, Moto2); //===速度环PI控制	 速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点

        // 	Moto1 = Balance_Pwm;
        //	Moto2 = Balance_Pwm;

        Moto1 = Balance_Pwm + Velocity_Pwm - Turn_Pwm; //===计算左轮电机最终PWM  通过第一章的推导，输出方程可以将串级PID的算法转化成为：一个单独的负反馈的直立环 + 一个单独的正反馈的速度环。Moto1=Balance_Pwm-Velocity_Pwm-Turn_Pwm; 这里究竟应该是用加号还是减号？
        Moto2 = Balance_Pwm + Velocity_Pwm + Turn_Pwm; //===计算右轮电机最终PWM 即脉冲/秒， 需要8000/s的脉冲频率才能达到2.5n/s, 150rpm

        // printf("%d \r\n",Moto1);

        // *!步进电机的转速不能突然变化，加速度过大会导致失步和啸叫，电机失能，所以必须有一个控制输出值突变的手段

        Xianfu_Pwm(last_Moto1, last_Moto2); // 限幅只能限定最大最小值，并没有限定加速度;步进电机频率要到13kHZ，转速要达到250rpm，即4n/s的速度

        // printf("M1 is %d  \r\n", Moto1);




        if (Turn_Off(roll) == 0) //===如果不存在异常
        {

            ST = 0;                // 电机使能

            Set_Pwm(Moto1, Moto2); //===赋值给PWM寄存器
            //				 Get_Zhongzhi	();
            // printf("Zhongzhi is %d \r\n", Zhongzhi);
        }
        else
        {
            ST = 1; // 电机失能
            Moto1 = 0;
            Moto2 = 0;
            Set_Pwm(Moto1, Moto2);


            Flag_Stop = 1;
            ZeroRequirementMean = 1;
            ZeroRequirementSpeedPid = 1;

            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);//报警黄灯亮起
            // printf("Failed, turn off!\r\n");
        }


        if (Flag_Zhongzhi == 0)
        {
            Get_Zhongzhi(); // 自动寻找平衡点
        }


    }

    // else  printf("Zhongzhi is %f \r\n", Zhongzhi);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); //最后翻转PC13LED灯，示波器可以从led引脚翻转频率看是否能保证1000hz的pid控制
}

/**************************************************************************
函数功能：限制PWM赋值
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(int lastMoto1, int lastMoto2)
{
    //const int Maximum_delta_speed = 2000; // 800
    /*限幅 */
    int Amplitude_H = 13000, Amplitude_L = -13000; // 经计算13000可达4n/s,246rpm,此电机最大转速在5.2n/s, 312rpm左右。
    if (Moto1 < Amplitude_L)
        Moto1 = Amplitude_L;
    if (Moto1 > Amplitude_H)
        Moto1 = Amplitude_H;
    if (Moto2 < Amplitude_L)
        Moto2 = Amplitude_L;
    if (Moto2 > Amplitude_H)
        Moto2 = Amplitude_H;

    /*限加速度*/

    //        if ((Moto1 - lastMoto1) > Maximum_delta_speed) // Maximum_delta_speed 值为2000,对应转速差别为0.339n/s,对应加速度为 0.339*200=6.8n/s2,因为计算的频率为每5ms一次
    //       {

    //            Moto1 = lastMoto1 + Maximum_delta_speed;
    //        }
    //        else if ((Moto1 - lastMoto1) < -Maximum_delta_speed)
    //        {
    //            Moto1 = lastMoto1 - Maximum_delta_speed;
    //        }

    //        if ((Moto2 - lastMoto2) > Maximum_delta_speed)
    //        {

    //            Moto2 = lastMoto2 + Maximum_delta_speed;
    //        }
    //        else if ((Moto2 - lastMoto2) < -Maximum_delta_speed)
    //        {
    //            Moto2 = lastMoto2 - Maximum_delta_speed;
    //        }

    /*如果Moto值小于300，是否将Moto值置零？*/

    // if (myabs(Moto1)<100) Moto1=0;
    // if (myabs(Moto2)<100) Moto2=0;

    // 不能用

    //	if (myabs(Moto1 - lastMoto1) > Maximum_delta_speed) //Maximum_delta_speed 值为2000,对应转速差别为0.339n/s,对应加速度为 0.339*200=6.8n/s2,因为计算的频率为每5ms一次
    //    {
    //        if ((Moto1 - lastMoto1) > 0)
    //        {
    //            Moto1 = lastMoto1 + Maximum_delta_speed;
    //        }
    //        else
    //        {
    //            Moto1 = lastMoto1 - Maximum_delta_speed;

    //        }

    //    }

    //    if (myabs(2 - lastMoto2) > Maximum_delta_speed)
    //    {
    //        if ((Moto2 - lastMoto2) > 0)
    //        {
    //            Moto2 = lastMoto2 + Maximum_delta_speed;
    //        }
    //        else
    //        {
    //            Moto2 = lastMoto2 - Maximum_delta_speed;

    //        }

    //    }
}

/**************************************************************************
函数功能：自适应中值 (改进版：允许小幅摆动中校准)
入口参数：无
返回  值：无
**************************************************************************/
void Get_Zhongzhi(void)
{
    // 如果已经校准过，直接退出 (除非你想做实时动态校准，否则校准一次即可)
    if (Flag_Zhongzhi == 1) return;

    // 安全检测：如果车快倒了 (角度过大)，重置采样
    if (fabs(roll) > 15.0f)
    {
        AutoZero_SumAngle = 0;
        AutoZero_Count = 0;
        AutoZero_InProgress = 0;
        return;
    }

    // 启动条件：
    // 1. 电机已使能 (Flag_Stop == 0)
    // 2. 角度在合理范围内 (-10度 到 10度)
    // 3. 已经运行了一小段时间 (避免上电瞬间的瞬态干扰)，比如进入控制循环 100 次后
    static int start_delay = 0;
    if (Flag_Stop == 0) start_delay++;

    if (start_delay > 100 && fabs(roll) < 10.0f)
    {
        AutoZero_InProgress = 1;
    }

    // 执行采样
    if (AutoZero_InProgress == 1)
    {
        AutoZero_SumAngle += (-roll); // 累加当前角度 (注意符号，你的 balance 函数用的是 -roll)
        AutoZero_Count++;

        // 采样足够次数 (假设 1kHz 中断，采样 2000 次 = 2 秒)
        // 2 秒内车会左右晃动几个周期，平均值即为机械中值
        if (AutoZero_Count >= 10000)
        {
            // 计算平均值
            float new_zhongzhi = AutoZero_SumAngle / AutoZero_Count;

            // 【关键】平滑过渡：不要直接赋值，防止突变导致摔倒
            // 如果新旧值差别不大，直接更新；如果差别大，分步更新或直接更新但配合低速启动
            Zhongzhi = new_zhongzhi;
            Flag_Zhongzhi = 1; // 标记校准完成

            // 提示用户
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET); // 亮红灯表示中值已经找到
            printf("Zhongzhi Auto-Calibrated: %f (Samples: %d)\r\n", Zhongzhi, AutoZero_Count);

            // 重置采样状态
            AutoZero_SumAngle = 0;
            AutoZero_Count = 0;
            AutoZero_InProgress = 0;
        }
    }
}

/**************************************************************************
函数功能：所有的控制代码都在这里面
        5ms定时中断由MPU6050的INT引脚触发
        严格保证采样和数据处理的时间同步
        在中断中处理运算是否有问题，原则上中断不是只是提供事件标记，事件处理在主循环中处理吗？
        如果直接在中断中处理，是否会阻拦其它关键事件的及时处理？
**************************************************************************/
// PA6 与MPU6050的INT引脚连接，设置为下降沿触发中断，中断处理函数在这里，这将使控制程序以5ms一次运行，与陀螺仪采样数据同步
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

    // MPU_Flag=1;
    DMP_Ready_Flag = 1;
}




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 检查是否是 TIM2 引发的中断
    if (htim->Instance == TIM2)
    {
        // 在这里添加您的中断处理代码

        ControlLoopPackage();

        // 例如：翻转LED指示灯
        //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

        // 或者执行其他周期性任务
        //printf("Timer 2 Update Interrupt Occurred!\n");
    }
}
int map(int val, int I_Min, int I_Max, int O_Min, int O_Max)
{
    return (val - I_Min) * (O_Max - O_Min) / (I_Max - I_Min) + O_Min;
}

/**
 * @brief MPU6050 姿态解算函数 (互补滤波)
 * @param aacx, aacy, aacz: 加速度计原始数据 (short/int16_t)
 * @param gyrox, gyroy, gyroz: 陀螺仪原始数据 (short/int16_t)
 * @param dt: 采样时间间隔 (秒)。例如：2ms中断则填 0.002f
 */
void MPU6050_Solve_Angle(short aacx, short aacy, short aacz,
                         short gyrox, short gyroy, short gyroz,
                         float dt)
{
    // 1. 数据单位转换
    // 加速度计：量程±2g，灵敏度 16384 LSB/g -> 转换为 g (重力单位)
    float ax = (float)aacx / 16384.0f;
    float ay = (float)aacy / 16384.0f;
    float az = (float)aacz / 16384.0f;

   // 陀螺仪：±2000dps -> 16.4 LSB/(deg/s)  <-- 这里必须改！之前是131.0
    float gx = (float)gyrox / 16.4f;
    float gy = (float)gyroy / 16.4f;
    float gz = (float)gyroz / 16.4f;

    // 2. 从加速度计计算参考角度 (atan2 返回弧度，乘以 180/PI 转为角度)

    // Pitch (前后倾): 绕Y轴转。由 X轴加速度(ax) 和 Z轴加速度(az) 决定
    // 公式推导：当车前倾，ax为正(假设)，az减小。atan2(-ax, az) 或 atan2(ax, sqrt(ay^2+az^2))
    // 这里使用标准公式：Pitch = atan2(-ax, sqrt(ay*ay + az*az))
    float Accel_Pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;

    // Roll (左右倾): 绕X轴转。由 Y轴加速度(ay) 和 Z轴加速度(az) 决定
    // 公式：Roll = atan2(ay, az)
    float Accel_Roll = atan2f(ay, az) * 180.0f / PI;

    // 3. 互补滤波融合 (核心算法)
    // 公式：新角度 = Alpha * (旧角度 + 陀螺仪*dt) + (1-Alpha) * 加速度计角度

    // --- 计算 Pitch ---
    Angle_Pitch = COMPLIMENTARY_ALPHA * (Last_Pitch + gy * dt) + (1.0f - COMPLIMENTARY_ALPHA) * Accel_Pitch;

    // --- 计算 Roll ---
    Angle_Roll = COMPLIMENTARY_ALPHA * (Last_Roll + gx * dt) + (1.0f - COMPLIMENTARY_ALPHA) * Accel_Roll;

    // --- 计算 Yaw (仅积分，无修正) ---
    // 警告：MPU6050无磁力计，此角度会随时间漂移，仅适用于短时转向控制
    Angle_Yaw = Last_Yaw + gz * dt;

    // Yaw 角度归一化到 -180 ~ 180 (可选，防止数值过大)
    if (Angle_Yaw > 180.0f) Angle_Yaw -= 360.0f;
    if (Angle_Yaw < -180.0f) Angle_Yaw += 360.0f;

    // 4. 更新历史值，供下一次循环使用
    Last_Pitch = Angle_Pitch;
    Last_Roll = Angle_Roll;
    Last_Yaw = Angle_Yaw;
}

