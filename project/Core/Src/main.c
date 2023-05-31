/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "stdio.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include <includes.h>
#include "ANO_TC.h"
#include "driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float pitch, roll, yaw;
uint8_t send_data[] = "Hello";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

struct MPU_DATA
{
	int16_t gx;
	int16_t gy;
	int16_t gz;
	int16_t ax;
	int16_t ay;
	int16_t az;
	float roll;
	float pitch;
	float yaw;
	/* data */
};


static CPU_STK   Task_Start_STK[APP_TASK_START_STK_SIZE] ;
static CPU_STK   Task_Base_STK[APP_TASK_START_STK_SIZE] ;






static OS_TCB Task_Start_TCB ;
static OS_TCB Task_Base_TCB ;
static OS_TMR Tmr1;
static OS_TMR Tmr2;






void Task_Start(void *p_arg) ;
void Task_Base(void *p_arg) ;
void Task_Tim1(void *p_arg) ;
void Task_Tim2(void *p_arg) ;



struct MPU_DATA mpu_data;
int16_t encoder;
int16_t motor_pwm,balance_pwm;
#define TARGET 90

void OS_Start(void ) 
{
	OS_ERR err ;
	
	OSInit (&err) ;
	
	OSTaskCreate ( (OS_TCB     *) &Task_Start_TCB,
							   (CPU_CHAR   *) "Task_Start",
	               (OS_TASK_PTR ) Task_Start ,
								 (void       *) 0 ,
								 (OS_PRIO     ) 4 ,
                 (CPU_STK    *) Task_Start_STK ,
                 (CPU_STK_SIZE)	APP_TASK_START_STK_SIZE /10 ,
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE ,
                 (OS_MSG_QTY  ) 0u ,
                 (OS_TICK     ) 0u ,
                 (void       *) 0 ,
                 (OS_OPT      ) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),	
                 (OS_ERR     *) &err );

  OSStart (&err ) ;								 					
}

void Task_Start(void *p_arg)
{
	CPU_INT32U cpu_clk_freq ;
	CPU_INT32U cnts ;
	OS_ERR err ;
	
	(void) p_arg ;
	
	
	
	BSP_Init ();  
	//led_on ;
	
	CPU_Init () ;  
	
	cpu_clk_freq = BSP_CPU_ClkFreq () ; 
	
	cnts = cpu_clk_freq / (CPU_INT32U ) OSCfg_TickRate_Hz ;
	
	OS_CPU_SysTickInit (cnts );
	
	Mem_Init () ;

	
	OSTaskCreate ( (OS_TCB     *) &Task_Base_TCB,
							   (CPU_CHAR   *) "Task_Base",
	               (OS_TASK_PTR ) Task_Base ,
								 (void       *) 0 ,
								 (OS_PRIO     ) 12 ,
                 (CPU_STK    *) Task_Base_STK ,
                 (CPU_STK_SIZE)	APP_TASK_START_STK_SIZE /10 ,
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE ,
                 (OS_MSG_QTY  ) 0u ,
                 (OS_TICK     ) 0u ,
                 (void       *) 0 ,
                 (OS_OPT      ) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),	
                 (OS_ERR     *) &err );
		
  OSTmrCreate(    &Tmr1,
                  (CPU_CHAR *) "Task_Tim1",
                  0,
                  5,
                  OS_OPT_TMR_PERIODIC,
                  (OS_TMR_CALLBACK_PTR)Task_Tim1,
                  0,
                  &err);

  OSTmrCreate(    &Tmr2,
                  (CPU_CHAR *) "Task_Tim2",
                  0,
                  2,
                  OS_OPT_TMR_PERIODIC,
                  (OS_TMR_CALLBACK_PTR)Task_Tim2,
                  0,
                  &err);

  OSTmrStart(&Tmr1,NULL);
  OSTmrStart(&Tmr2,NULL);
  


  OSTaskDel ( & Task_Start_TCB, & err );


}	

void Task_Base(void *p_arg)
{
	
	
	  OS_ERR      err;
    
    while (1) {                                          /* Task body, always written as an infinite loop.       */
			//led_toggle ;
		
			//printf("this is base\n");
      
			//motor_control(120);
			//
      //ANO_TC_SendEuler(mpu_data.roll,mpu_data.pitch,mpu_data.yaw);
      //ANO_TC_Send(300,400,500,-300,-400,-500);
			OSTimeDly ( 1, OS_OPT_TIME_DLY, & err );
    }

	
}
void Task_Tim1(void *p_arg) 
{
    encoder = get_encoder();
    mpu_dmp_get_data(&mpu_data.pitch, &mpu_data.roll,&mpu_data.yaw);
    //led_off;
	  MPU_Get_Gyroscope(&mpu_data.gx,&mpu_data.gy,&mpu_data.gz);
    //MPU_Get_Accelerometer(&mpu_data.ax,&mpu_data.ay,&mpu_data.az);
		//printf("this is tim1\n");
		ANO_TC_Send(encoder,motor_pwm,350,mpu_data.ax,mpu_data.ay,mpu_data.az);
	  //ANO_TC_Send(mpu_data.gx,mpu_data.gy,mpu_data.gz,mpu_data.ax,mpu_data.ay,mpu_data.az);
		//printf("ax : %d ay : %d az : %d\n",mpu_data.ax,mpu_data.ay,mpu_data.az);
    //ANO_TC_SendEuler(mpu_data.roll,mpu_data.pitch,mpu_data.yaw);
}
void Task_Tim2(void *p_arg) 
{
    //printf("this is tim2\n");
		static short cnt = 0;
    static float vel_out = 0.0;
    static int16_t ang_out = 0;
	  motor_pwm = motor_pi(encoder,TARGET);
    if(cnt % 32 == 0){
      vel_out = velocity_control(encoder,0);
    }else if(cnt % 8 == 0){
      ang_out = angle_control(mpu_data.pitch,vel_out);
    }
    balance_pwm = palstance_control(mpu_data.gx,ang_out);
}
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_Base_Start(&htim1);
  mpu_dmp_init();
	
  motor_control(0);
  
  
  OS_Start();
	

  

	
	
  while (1)
  {
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
