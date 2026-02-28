/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define CurrentAdjust_Pin GPIO_PIN_1
#define CurrentAdjust_GPIO_Port GPIOA
#define LDIR_Pin GPIO_PIN_3
#define LDIR_GPIO_Port GPIOA
#define RDIR_Pin GPIO_PIN_4
#define RDIR_GPIO_Port GPIOA
#define MPU_INT_Pin GPIO_PIN_6
#define MPU_INT_GPIO_Port GPIOA
#define MPU_INT_EXTI_IRQn EXTI9_5_IRQn
#define M0_Pin GPIO_PIN_0
#define M0_GPIO_Port GPIOB
#define M1_Pin GPIO_PIN_1
#define M1_GPIO_Port GPIOB
#define M2_Pin GPIO_PIN_2
#define M2_GPIO_Port GPIOB
#define EN_Pin GPIO_PIN_14
#define EN_GPIO_Port GPIOB
#define LSTEP_Pin GPIO_PIN_8
#define LSTEP_GPIO_Port GPIOA
#define RSTEP_Pin GPIO_PIN_9
#define RSTEP_GPIO_Port GPIOA
#define BLRX_Pin GPIO_PIN_10
#define BLRX_GPIO_Port GPIOA
#define ELRSTX_Pin GPIO_PIN_11
#define ELRSTX_GPIO_Port GPIOA
#define ELRSRX_Pin GPIO_PIN_12
#define ELRSRX_GPIO_Port GPIOA
#define BLTX_Pin GPIO_PIN_15
#define BLTX_GPIO_Port GPIOA
#define StatusLed_Pin GPIO_PIN_3
#define StatusLed_GPIO_Port GPIOB
#define InputKey_Pin GPIO_PIN_4
#define InputKey_GPIO_Port GPIOB
#define MPUSCL_Pin GPIO_PIN_6
#define MPUSCL_GPIO_Port GPIOB
#define MPUSDA_Pin GPIO_PIN_7
#define MPUSDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#include "oldtype.h"
#include "mpu6050.h"
#include "printf.h"
#include "delay.h"
#include "control.h"
#include "math.h"
#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "sys.h"
#include "motor.h"
#include "time.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "mpu6050.h"
#include "dmpKey.h"
#include "dmpmap.h"

void	testrun(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
