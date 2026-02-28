/*****************************************************************************************************
Title: 				delay header file
Current 			version: v1.0
Function:			Define variables,Statement subfunction.
processor: 	                STM32F10
Clock:				8-72M  Hz
Author:				huanghuajian
Company:
Contact:
E-MAIL:				huajian11@qq.com
Data:				  2017-3-21
*******************************************************************************************************/
#ifndef __DELAY_H
#define __DELAY_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "stm32f4xx_hal.h"
    //////////////////////////////////////////////////////////////////////////////////
    //使用SysTick的普通计数模式对延迟进行管理
    // delay_us,delay_ms
    //********************************************************************************
    // V1.2修改说明
    //修正了中断中调用出现死循环的错误
    //防止延时不准确,采用do while结构!
    //////////////////////////////////////////////////////////////////////////////////
    void delay_init(uint8_t SYSCLK);
    void delay_ms(uint16_t nms);
    void delay_us(uint32_t nus);

#ifdef __cplusplus
}
#endif

#endif

