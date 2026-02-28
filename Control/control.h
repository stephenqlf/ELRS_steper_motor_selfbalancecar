#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
#include "oldtype.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
#define PI 3.14159265
#define FILTERING_TIMES  10
#define KEY PBin(4)

u8 click_N_Double (u8 time);  //单击按键扫描和双击按键扫描
u8 click(void);               //单击按键扫描
u8 Long_Press(void);           //长按扫描  
extern	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
int EXTI15_10_IRQHandler(void);
int balance(float Angle ,float Gyro);
int velocity(int velocity_left,int velocity_right);
void Set_Pwm(int moto1,int moto2);
void Key(void);
void Xianfu_Pwm(int lastMoto1, int lastMoto2);
u8 Turn_Off(float angle);
void Get_Angle(void);
int myabs(int a);
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right);
int Put_Down(float Angle,int encoder_left,int encoder_right);
void speed_filter(void);
int Mean_Filter(int moto1,int moto2);
u16  Linear_Conversion(int moto);
void  Get_Zhongzhi(void);
void MPU6050_Data_read(void);
int turn(int velocity_left,int velocity_right);//转向控制
void  Get_Elrs(void);
int map(int val, int I_Min, int I_Max, int O_Min, int O_Max);
#endif
