#include "motor.h"


#include "main.h"

#include "tim.h"

/**************************************************************************
函数功能：TIM1 输出比较中断服务函数 步进电机调频PWM输出，当CCR与CNT相等时，发生中断；此时立即把CCR值更新再加上一个要设置的
入口参数：无
返回  值：无



**************************************************************************/

//HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_1);  /* 以中断方式启动TIM1通道1的输出比较,使用前需先用这两句话开启输出比较中断 */
//HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_2);  /* 以中断方式启动TIM1通道2的输出比较 */

extern int Moto1, Moto2, Final_Moto1, Final_Moto2;



void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{

    int Capture1, Capture2; //这里原来是float类型，我在想如果Capture1+Final_Moto1 >65535后，怎么办？按道理CNT值应该重装载为0，从0开始计数，那么将永远达不到一个比65535大的值，还有这里为什么要设置为float 类型，这里应该是一个整型或长整型 (因为计时器值为16位,最大值是65535，超过这个数自动从零开始）

    if(htim->Instance == TIM1) 			    /* 判断是否是定时器1 */
    {
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
            /* 判断是否是通道1 */
            Capture1 = TIM1->CCR1;
            TIM1->CCR1 = Capture1 + Final_Moto1; //左电机是channell capture值一直加，最后溢出了怎么办？ 溢出重新从零开始，计数器也会从零开始，所以没问题  TIM1->CCR1 = (uint16_t)(next_ccr & 0xFFFF); // 自动处理溢出 非必须，(uint16_t) 隐式转换效果相同，但显式写更清晰
            //printf("new CCR %d \r\n", TIM1 -> CCR1);
            //__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,Capture1 + Final_Moto1); //标准的HAL设置CCR值参数，就是cubemx中oc下的pulse值

        }
        else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
        {
            /* 判断是否是通道2 */

            Capture2 = TIM1->CCR2;
            TIM1->CCR2 = Capture2 + Final_Moto2; //右电机是channel2
        }
    }
}









