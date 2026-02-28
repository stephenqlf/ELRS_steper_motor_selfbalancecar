#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/

#define Left_Direction   PAout(3) //设定左电机旋转方向
#define Right_Direction  PAout(4)  //设定右电机旋转方向
#define ST   PBout(14) //失能电机
void MiniBalance_PWM_Init(void);
void MiniBalance_Motor_Init(void);


#endif
