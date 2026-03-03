/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "kalman_filter.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//#include "arm_math.h" //确保使用FPU加速浮点计算





int received_size = 0;
int RxDoneFlag = 0; 
float currentLeanAngle = 0.0f;
float currentOmega = 0.0;
float verticleAngle = 0;
int MPU_Falg = 0;

u8 mpu_dmp_flag = 0;  //陀螺仪初始化判断
int i = 0;
int flag = 1;
u8 Way_Angle = 1 ;                           
u8 Flag_Qian, Flag_Hou, Flag_Left, Flag_Right, Flag_sudu = 1; 

int Moto1 = 0, Moto2 = 0, Final_Moto1 = 0, Final_Moto2 = 0; 
int Temperature;                            
int Voltage;                               

float Show_Data_Mb = 0, Show_Data_Mb2 = 0;  
u8 delay_50, delay_flag, Bi_zhang = 0, PID_Send, Flash_Send; 
u32 Distance;                               
float Acceleration_Z;                       
long Remoter_Ch1 = 1500, Remoter_Ch2 = 1500, Arm_ch6 = 1500; 



float Balance_Kp = 350, Balance_Kd =-0.05; //Balance_Kd =- 0.12
int Target_Velocity = 3000; // 默认速度快慢


float Zhongzhi = -0.55f;

extern Kalman_Filter_t balance_kf; // 全局变量

int Balance_Pwm, Velocity_Pwm, Turn_Pwm;
int MPU_Flag;
void	testrun(void);
static void MX_NVIC_Init(void);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t receive_buff[255];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
    
    MX_USART6_UART_Init();
  
	
	MX_TIM1_Init();
    MX_TIM2_Init();
    /* USER CODE BEGIN 2 */
    delay_init(100); 
	
	 MPU6050_Init();					                           //初始化MPU6050,一定要放到中断之前初始化
	  
	  /*想要尝试1000hz的pid控制频率，只能使用mpu的基础功能，自已解算出倾角，下面的dmp初始化代码注释掉*/
   // mpu_dmp_flag = mpu_dmp_init();                 //初始化MPU6050的DMP
//    printf("MPU6050 DMP Initiate flag is %d \r\n", mpu_dmp_flag);	
	
	// ??? 关键步骤：初始化卡尔曼滤波器 ???
    // 参数说明: &balance_kf, Q_angle, Q_gyro, R_angle, dt(秒)
    // 如果你的控制周期是 5ms (0.005s)，dt 必须填 0.005f
    // 如果不确定参数，先用这一组经典值：
	/*
	想平滑（去抖），加大R 或减小Q 。 
    想反应快（跟手），减小 R 或加大 Q 。
	=== 卡尔曼滤波调参指南 ===
第一步：解决“剧烈抖动” (高频噪声)
现象：如果角度数值像心电图一样乱跳（例如在 0.0 附近 ±2.0 快速波动）。
原因：R (测量噪声) 太小了，滤波器太相信有噪声的加速度计。
操作：增大 R_angle。
尝试步骤：0.5 -> 1.0 -> 2.0 -> 5.0
经验值：对于步进电机驱动的平衡车，震动很大，R 甚至可能需要设到 3.0 或 5.0。
第二步：解决“反应迟钝/滞后” (相位延迟)
现象：如果小车推一下，角度要过 0.5 秒才慢慢变过来，或者车倒了滤波器还没反应过来。
原因：R 太大，或者 Q_angle (角度过程噪声) 太小。
操作：
稍微减小 R_angle (比如从 2.0 降到 1.0)。
或者增大 Q_angle (比如从 0.001 升到 0.005)。
注意：这会引入一些抖动，需要在平滑和快速之间找平衡点。
第三步：解决“缓慢漂移” (低频漂移)
现象：如果小车静止不动，角度却在几分钟内慢慢从 0.0 飘到了 5.0。
原因：Q_gyro (陀螺仪偏置噪声) 太小，滤波器不相信陀螺仪有零漂，或者积分时间太长。
操作：增大 Q_gyro。
尝试步骤：0.003 -> 0.005 -> 0.01
说明：通常 Q_gyro 对动态性能影响不大，主要影响长期稳定性。
	
	
	
	void Kalman_Filter_Init(Kalman_Filter_t *kf, float Q_angle, float Q_gyro, float R_angle, float dt)
*/
    Kalman_Filter_Init(&balance_kf, 0.001f, 0.003f, 5.0f, 0.0025f);
	
	
    MX_NVIC_Init();





    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); 
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

    delay_ms(200);
	
	
    HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2); 
    HAL_TIM_Base_Start_IT(&htim2);  // <<< 启动 TIM2 更新中断！
 

    //HAL_UART_Receive_DMA(&huart6, (uint8_t *)&receive_buff, 255);  
    //HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    //Test Motor Rotation
    //		Left_Direction=1;
    //		Right_Direction=1;
    ST = 1; //????1???????
	CRSF_Init(&huart6);
//    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t *)&receive_buff, 255);  
//    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE); 
	
	 
   delay_ms(200);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {

        //		if(MPU_Flag ==1)
        //		{
        //
        //			MPU_Flag=0;
        //			ControlLoopPackage();
        //
        //
        //		}

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        //Get_Zhongzhi	();
        //testrun() 


        //
        //		Final_Moto1=300;
        //		Final_Moto2=300;





        //ControlLoopPackage();

        //printf("come to end of while \r\n");

    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 12;
    RCC_OscInitStruct.PLL.PLLN = 96;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
static void MX_NVIC_Init(void)
{
    /* USART6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
