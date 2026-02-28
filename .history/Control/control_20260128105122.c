

#include "math.h"
#include "usart.h"
#include "stm32f4xx.h" // Device header
#include "oldtype.h"
#include "motor.h"
#include "control.h"
#include "MPU6050.h"
#include "printf.h"
#include "crsf.h"
#define PRESCALER 99
#define speedFilterConstant 0.7
extern u32 Arm_ch6;
extern uint8_t receive_buff[255];
extern int Voltage;                                              // 电池电压采样相关的变量
extern float Remoter_Ch1, Remoter_Ch2;                           // 航模遥控接收变量
extern float Balance_Kp, Balance_Kd, Velocity_Kp, Velocity_Ki;   // PID参数
extern u8 Flag_Qian, Flag_Hou, Flag_Left, Flag_Right, Flag_sudu; // 蓝牙遥控相关的变量
extern int Moto1, Moto2, Final_Moto1, Final_Moto2;               // 电机PWM变量 应是Motor的 向Moto致敬
extern float Roll, Pitch, Angle_Balance, Gyro_Balance;           // 平衡倾角
extern u8 Flag_Qian, Flag_Hou, Flag_Left, Flag_Right;            // 蓝牙遥控相关的变量                //停止标志位和 显示标志位 默认停止 显示打开
extern float Zhongzhi, Flag_Zhongzhi;
extern float Acceleration_Z; // Z轴加速度计
extern int _channels[];
extern int MPU_Flag;
extern int Rx_Available_Flag;
extern int Maximum_delta_speed;
float xpeed = 0.0;
const float Math_PI = 3.1415926;
/**************************************************************************
函数功能：直立PD控制
入口参数：角度
返回  值：直立控制PWM

**************************************************************************/
int balance(float Angle, float Gyro)
{
    float Bias; // 这里D为零
    int balanceValue;
    Bias = Angle - Zhongzhi; //===求出平衡的角度中值 和机械相关

    balanceValue = (int)(Balance_Kp * Bias + Balance_Kd * Gyro); //===计算平衡控制的电机PWM
    // printf("%f, %d\r\n",Bias, balanceValue);
    return balanceValue;
}

/**************************************************************************
函数功能：速度PI控制 修改前进后退速度，请修Target_Velocity
入口参数：左轮速度、右轮速度
返回  值：速度控制PWM

**************************************************************************/
int velocity(int velocity_left, int velocity_right)
{
    static float Encoder_Least, Encoder, Movement;
    static int Velocity;
    static float Target_Velocity = 3000; // 默认速度快慢
    static float Encoder_Integral;

    if (1 == Flag_Qian)
        Movement = Target_Velocity; //===前进标志位置1
    else if (1 == Flag_Hou)
        Movement = -Target_Velocity; //===后退标志位置1
    else
        Movement = 0;

    // printf("%f\r\n",Movement);

    //=============速度PI控制器=======================//
    Encoder_Least = Mean_Filter(velocity_left, velocity_right); // 上一个循环的速度滤波 , 此处究竟是加上Movement 还是减去Movement 应该与正负极性有关，须在验证极性后再来确定，与Kp, Ki值一样需要验证极性

    Encoder *= 0.7f;                 //===一阶低通滤波器
    Encoder += Encoder_Least * 0.3f; //===一阶低通滤波器
    Encoder_Integral += Encoder;     //===积分出位移
    Encoder_Integral += Movement;    //===接收遥控器数据，控制前进后退
    if (Encoder_Integral > 320000)
        Encoder_Integral = 320000; //===积分限幅，输出限幅在8000左右，怎么积分限幅可以这么大？？？？
    if (Encoder_Integral < -320000)
        Encoder_Integral = -320000; //===积分限幅

    Velocity = (int)(Encoder * Velocity_Kp / 100 + Encoder_Integral * Velocity_Kp / 200 / 100); //===速度控制
    if (Turn_Off(Angle_Balance) == 1)
    {
        Encoder = 0;
        Encoder_Integral = 0; //===电机关闭后清除积分
        // printf("%d\r\n",Velocity);
    }
    return Velocity;
}

/**************************************************************************
函数功能：转向控制
入口参数：无
返回  值：转向控制PWM

**************************************************************************/
int turn(int velocity_left, int velocity_right) // 转向控制
{
    static float Turn_Amplitude = 1000, Turn_Target;

    if (1 == Flag_Left)
        Turn_Target -= 60;
    else if (1 == Flag_Right)
        Turn_Target += 60;
    else
        Turn_Target = 0;
    if (Turn_Target > Turn_Amplitude)
        Turn_Target = Turn_Amplitude; //===转向速度限幅
    if (Turn_Target < -Turn_Amplitude)
        Turn_Target = -Turn_Amplitude;
    return Turn_Target;
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
    // printf("Arm value %ld, Angle value %f \r\n", Arm_ch6, myabs(angle));
    if (Arm_ch6 > 800 && Arm_ch6 < 1000 && myabs(angle) < 50) // 解锁按钮在解锁位置,竖直状态

    {
        temp = 0; // 可以开始自平衡
    }

    else

    {

        temp = 1;
    }

    return temp;
}

/**************************************************************************
函数功能：采集遥控器的信号
入口参数：无
返回  值：无
**************************************************************************/
void Get_Elrs()
{

    Remoter_Ch1 = _channels[1];
    Remoter_Ch2 = _channels[0];
    Arm_ch6 = _channels[5];

    // printf( "ch1 %f, ch2 %f ,arm ch6 is %f \r\n", Remoter_Ch1,Remoter_Ch2,Arm_ch6);

    xpeed = map(Remoter_Ch1, -1000, 1000, -10000, 10000);

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

    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t *)&receive_buff, 255); // 数据获取到后再开启传输，以免浪费资源
}

/**************************************************************************
函数功能：获取角度
入口参数：获取角度的算法 1：DMP
返回  值：无
**************************************************************************/
void Get_Angle()
{
    //  float Accel_Y,Accel_Angle,Accel_Z,Gyro_X,Gyro_Z;

    Read_DMP();                //===读取加速度、角速度、倾角
    Angle_Balance = -Roll;     //===原来平衡倾角Angle_Balance=-Roll，这是错的;
    Gyro_Balance = gyro[0];    //===更新平衡角速度，Pitch以y轴旋转
    Acceleration_Z = accel[2]; //===更新Z轴加速度计
    // printf(" Roll is %f    ", Angle_Balance);
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
    if (moto == 0)
    {
        return 65535; // 或一个极大值，让频率≈0
    }

    temp = 100000000 / (99 + 1) / myabs(moto);
    if (temp > 65535)
        Linear_Moto = 65535;
    // if(temp > 4000) Linear_Moto = 4000;//计时器的频率为100万每秒，1ms只能读数到1000，5ms只能读数到5000，超过5000的值则在一个控制周期内还未读完，这里ccr值又变大,可能导致永远也不跳转
    else
        Linear_Moto = (u16)temp;
    if (temp < 75)
        Linear_Moto = 75; // 最小脉冲个数值60个，比这更低马达功率的扭矩不够，转不起来
    else
        Linear_Moto = (u16)temp;
    return Linear_Moto; // 计算结果会用1000000/Linear_Moto得到每秒能发多少个脉冲，然后/16/200,就是多少n/s，如果最大转速为
}

void ControlLoopPackage()
{

    static int last_Moto1 = 0, last_Moto2 = 0;
    static int LED_count = 0;
    last_Moto1 = Moto1;
    last_Moto2 = Moto2;

    Get_Angle(); //===更新姿态

    Get_Elrs(); //===读取航模遥控器的数据

    Balance_Pwm = balance(Angle_Balance, Gyro_Balance); //===平衡控制 balanceValue = (int)(Balance_Kp * Bias + Balance_Kd * Gyro); //===计算平衡控制的电机PWM

    Velocity_Pwm = velocity(Moto1, Moto2);
    Turn_Pwm = turn(Moto1, Moto2); //===速度环PI控制	 速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点

    Moto1 = Balance_Pwm + Velocity_Pwm - Turn_Pwm; //===计算左轮电机最终PWM  通过第一章的推导，输出方程可以将串级PID的算法转化成为：一个单独的负反馈的直立环 + 一个单独的正反馈的速度环。Moto1=Balance_Pwm-Velocity_Pwm-Turn_Pwm; 这里究竟应该是用加号还是减号？
    Moto2 = Balance_Pwm + Velocity_Pwm + Turn_Pwm; //===计算右轮电机最终PWM

    // printf("%d \r\n",Moto1);

    // 步进电机的转速不能突然变化，加速度过大会导致失步和啸叫，电机失能，所以必须有一个控制输出值突变的手段

    Xianfu_Pwm(last_Moto1, last_Moto2); // 限幅只能限定最大最小值，并没有限定加速度;步进电机频率要到13kHZ，转速要达到250rpm，即4n/s的速度

    // printf("M1 is %d  \r\n", Moto1);

    /* Moto的值还要经过限幅和线性化，才能赋值给计时器输出pwm波

    void Xianfu_Pwm(void)
    {
        int Amplitude_H = 19000, Amplitude_L = -19000;   //经计算如果分频14，则需要最大值11834才能达到最大转速4n/s的速度，分频改成21的话，差不多8000 （原来是5000）
        if(Moto1 < Amplitude_L)  Moto1 = Amplitude_L;
        if(Moto1 > Amplitude_H)  Moto1 = Amplitude_H;
        if(Moto2 < Amplitude_L)  Moto2 = Amplitude_L;
        if(Moto2 > Amplitude_H)  Moto2 = Amplitude_H;
    }


    u16  Linear_Conversion(int moto)
    {
        u32 temp;
        u16 Linear_Moto;
        temp = 36000000 / (PRESCALER + 1) / 13000 * 5000 / myabs(moto);
        if(temp > 65535) Linear_Moto = 65535;
        else Linear_Moto = (u16)temp;
        if (temp < 80) Linear_Moto = 80; //最小脉冲个数值60个，比这更低马达功率的扭矩不够，转不起来
        else Linear_Moto = (u16)temp;
        return Linear_Moto;
    }








    */

    if (Turn_Off(Angle_Balance) == 0) //===如果不存在异常
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
        // printf("Failed, turn off!\r\n");
    }
    if (Flag_Zhongzhi == 0)
    {
        Get_Zhongzhi(); // 自动寻找平衡点
    }
    else
    {
        LED_count++;
        if (LED_count == 100)
        {
            LED_count = 0;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // 最后翻转PC13LED灯
        }
    }
    // else  printf("Zhongzhi is %f \r\n", Zhongzhi);
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); //最后翻转PC13LED灯
}

/**************************************************************************
函数功能：限制PWM赋值
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(int lastMoto1, int lastMoto2)
{
    const int Maximum_delta_speed = 1200; // 800
    /*限幅 */
    int Amplitude_H = 13000, Amplitude_L = -13000; // 经计算如果分频100，则需要最大值9600才能达到最大转速3n/s的速度, 4200是80rpm
    if (Moto1 < Amplitude_L)
        Moto1 = Amplitude_L;
    if (Moto1 > Amplitude_H)
        Moto1 = Amplitude_H;
    if (Moto2 < Amplitude_L)
        Moto2 = Amplitude_L;
    if (Moto2 > Amplitude_H)
        Moto2 = Amplitude_H;

    /*限加速度*/

    if ((Moto1 - lastMoto1) > Maximum_delta_speed) // Maximum_delta_speed 值为2000,对应转速差别为0.339n/s,对应加速度为 0.339*200=6.8n/s2,因为计算的频率为每5ms一次
    {

        Moto1 = lastMoto1 + Maximum_delta_speed;
    }
    else if ((Moto1 - lastMoto1) < -Maximum_delta_speed)
    {
        Moto1 = lastMoto1 - Maximum_delta_speed;
    }

    if ((Moto2 - lastMoto2) > Maximum_delta_speed)
    {

        Moto2 = lastMoto2 + Maximum_delta_speed;
    }
    else if ((Moto2 - lastMoto2) < -Maximum_delta_speed)
    {
        Moto2 = lastMoto2 - Maximum_delta_speed;
    }

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
函数功能：自适应中值
入口参数：无
返回  值：无
**************************************************************************/
void Get_Zhongzhi(void)
{
    static int count;

    if (myabs(Moto1) < 100 && myabs(Moto2) < 100)
        count++; // 采样
    else
        count = 0;

    if (count > 300) // 连线3秒处于平衡位置，读取中值
    {
        Zhongzhi = Angle_Balance;
        Flag_Zhongzhi = 1;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        printf("Zhongzhi is %f \r\n", Zhongzhi);
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
    ControlLoopPackage();
}
