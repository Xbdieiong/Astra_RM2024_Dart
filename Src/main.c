/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "CAN_receive.h"
#include "pid.h"
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
RC_Ctl_t RC_Ctl;   							//遥控器结构体、
uint8_t sbus_rx_buffer[18]; 		//接受缓存区
uint8_t stop_time=0 ;
uint8_t lastmode=0 ;
uint16_t start_time=0;

pid_type_def motor1_pid,motor2_pid,motor3_pid,motor4_pid,motor5_pid,motor6_pid;

const motor_measure_t *motor1_data,*motor2_data,*motor3_data,*motor4_data,*motor5_data,*motor6_data;
int set_speed3_launch1,set_speed3_launch2,set_current_Yaw,set_current_transmit,set_current_Pitch; //���÷��䡢̧�������䡢ת��ĵ���
int set_speed3,set_speed2,set_speed1;
const fp32 PID[3]={5,0.01,0};	//P,I,D

//static motor_measure_t motor_chassis[7];


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
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
	PID_init(&motor1_pid,PID_POSITION,PID,10000,000);
	PID_init(&motor2_pid,PID_POSITION,PID,10000,000);
	PID_init(&motor3_pid,PID_POSITION,PID,10000,000);
	PID_init(&motor4_pid,PID_POSITION,PID,10000,000);
	PID_init(&motor5_pid,PID_POSITION,PID,6000,000);
	PID_init(&motor6_pid,PID_POSITION,PID,6000,000);
	
	motor1_data = get_chassis_motor_measure_point(0); 		//moto1数据
	motor2_data = get_chassis_motor_measure_point(1); 		//moto2数据
	motor3_data = get_chassis_motor_measure_point(2); 		//moto3数据
	motor4_data = get_chassis_motor_measure_point(3); 		//moto4数据
	motor5_data = get_yaw_gimbal_motor_measure_point();		//moto3数据
	motor6_data = get_pitch_gimbal_motor_measure_point(); 		//moto4数据	
	
  can_filter_init();
	
	HAL_UART_Receive_DMA(&huart3,sbus_rx_buffer,18);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	int control_count = 1;
	while(1)
	{
    if(RC_Ctl.rc.s1 == 1)//摇杆在上
    {
        set_speed3 = 5200; //max7000
			  set_speed2= 5200;
			  set_speed1= 3000;			
        PID_calc(&motor1_pid, motor1_data->speed_rpm, -set_speed3);
        PID_calc(&motor2_pid, motor2_data->speed_rpm, set_speed3);
        PID_calc(&motor3_pid, motor3_data->speed_rpm, -set_speed2);
        PID_calc(&motor4_pid, motor4_data->speed_rpm, set_speed2);
        PID_calc(&motor5_pid, motor5_data->speed_rpm, -set_speed1);
        PID_calc(&motor6_pid, motor6_data->speed_rpm, set_speed1);
        CAN_cmd_chassis(motor1_pid.out, motor2_pid.out, motor3_pid.out, motor4_pid.out);
				CAN_cmd_gimbal(motor5_pid.out, motor6_pid.out,0, 0);
			  //HAL_Delay(1);
        //CAN_Launch_Dir_Receive(motor5_pid.out, motor6_pid.out, 0, 0);
    }
		else if(RC_Ctl.rc.s1 == 3)//摇杆中间
		{
        set_speed3 = 5100; //max7000
			  set_speed2= 5100;
			  set_speed1= 3000;
        PID_calc(&motor1_pid, motor1_data->speed_rpm, -set_speed3);
        PID_calc(&motor2_pid, motor2_data->speed_rpm, set_speed3);
        PID_calc(&motor3_pid, motor3_data->speed_rpm, -set_speed2);
        PID_calc(&motor4_pid, motor4_data->speed_rpm, set_speed2);
        PID_calc(&motor5_pid, motor5_data->speed_rpm, -set_speed1);
        PID_calc(&motor6_pid, motor6_data->speed_rpm, set_speed1);
        CAN_cmd_chassis(motor1_pid.out, motor2_pid.out, motor3_pid.out, motor4_pid.out);
				CAN_cmd_gimbal(motor5_pid.out, motor6_pid.out,0, 0);
			  //HAL_Delay(1);
        //CAN_Launch_Dir_Receive(motor5_pid.out, motor6_pid.out, 0, 0);			
		}
		else
		{
        set_speed3 = 0; //max7000
        PID_calc(&motor1_pid, motor1_data->speed_rpm, -set_speed3);
        PID_calc(&motor2_pid, motor2_data->speed_rpm, set_speed3);
        PID_calc(&motor3_pid, motor3_data->speed_rpm, -set_speed3);
        PID_calc(&motor4_pid, motor4_data->speed_rpm, set_speed3);
        PID_calc(&motor5_pid, motor5_data->speed_rpm, -0);
        PID_calc(&motor6_pid, motor6_data->speed_rpm, 0);
        CAN_cmd_chassis(motor1_pid.out, motor2_pid.out, motor3_pid.out, motor4_pid.out);
				CAN_cmd_gimbal(motor5_pid.out, motor6_pid.out,0, 0);			
		}
		HAL_Delay(1);
    //control_count++;
	}
//while (1)
//{
//		stop_time=0;
////		set_current_launch = (RC_Ctl.rc.ch3-1024)*2;
//		
////		set_current_transmit = (RC_Ctl.rc.ch3-1024)*2;
//		if((RC_Ctl.rc.ch3-1024)*2>=1000)
//		{
//			set_current_transmit = 1000 ;
//			stop_time=1;
//		}
//		else if((RC_Ctl.rc.ch3-1024)*2<=-1000)
//		{
//			set_current_transmit = -1000 ;
//			stop_time=2;
//		}else
//		{
//			switch (stop_time)
//			{
//				case 0:set_current_transmit = 0 ;stop_time=0;break;
//				case 1:stop_time=0;set_current_transmit = 0 ;break;
//				case 2:stop_time=0;set_current_transmit = 0 ;break;	
////				case 1:CAN_Launch_Dir_Receive(-2000,0,0);HAL_Delay(1);stop_time=0;set_current_transmit = 0 ;break;
////				case 2:CAN_Launch_Dir_Receive(2000,0,0);HAL_Delay(1);stop_time=0;set_current_transmit = 0 ;break;	
//			}
//		}
//		set_current_Yaw = (RC_Ctl.rc.ch0-1024)*2;		
//		set_current_Pitch = (RC_Ctl.rc.ch1-1024)*2;
    /* USER CODE END WHILE */
//		set_speed3_launch1 = 1000;//设置第一组速度值  上
//		set_speed3_launch2 = 1000;//设置第二组速度值  下
		
//		PID_calc(&motor1_pid,motor1_data->speed_rpm,-set_speed3_launch1);
//		PID_calc(&motor2_pid,motor2_data->speed_rpm,set_speed3_launch1);
//		PID_calc(&motor3_pid,motor3_data->speed_rpm,set_speed3_launch2);
//		PID_calc(&motor4_pid,motor4_data->speed_rpm,-set_speed3_launch2);

//		if(RC_Ctl.rc.s1 == 1)
//		{
//			if(lastmode==0)
//			{
//				for(start_time=0;start_time<=5000;start_time++)
//				{
//						set_speed3_launch1 = start_time/2;
//		        set_speed3_launch2 = start_time/2;
//					  uint8_t i=0;
//						for(i=0;i<200;i++)
//					 {
//						set_speed3_launch1 = start_time/2;
//		        set_speed3_launch2 = start_time/2;
//					 }
//				}
//				if(start_time>=5000)
//				{
//					set_speed3_launch1 = 1000;//调速最大8000
//					set_speed3_launch2 = 1000;//调速最大8000
//					lastmode=1;
//					start_time=0;
//				}
//			}
//			PID_calc(&motor1_pid,motor1_data->speed_rpm,-set_speed3_launch1);
//			PID_calc(&motor2_pid,motor2_data->speed_rpm,set_speed3_launch1);
//			PID_calc(&motor3_pid,motor3_data->speed_rpm,set_speed3_launch2);
//			PID_calc(&motor4_pid,motor4_data->speed_rpm,-set_speed3_launch2);
//			CAN_cmd_chassis(motor1_pid.out,motor2_pid.out,motor3_pid.out,motor4_pid.out);
//			CAN_Launch_Dir_Receive(set_current_transmit,set_current_Yaw,set_current_Pitch);
//		}
//		else 
//		{
//			CAN_cmd_chassis(0,0,0,0) ;
//			lastmode=0;
//		}
//		HAL_Delay(1);
//	
//  }
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) 
{  
				RC_Ctl.rc.ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff;          
				RC_Ctl.rc.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff;       
				RC_Ctl.rc.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff;          
				RC_Ctl.rc.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff;           
				RC_Ctl.rc.s1  = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2;                           
				RC_Ctl.rc.s2  = ((sbus_rx_buffer[5] >> 4)& 0x0003);    

				RC_Ctl.mouse.x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8);                    //!< Mouse X axis        
				RC_Ctl.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8);                    //!< Mouse Y axis      
				RC_Ctl.mouse.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8);                  //!< Mouse Z axis         
				RC_Ctl.mouse.press_l = sbus_rx_buffer[12];                                        //!< Mouse Left Is Press      
				RC_Ctl.mouse.press_r = sbus_rx_buffer[13];                                        //!< Mouse Right Is Press 
				RC_Ctl.key.v = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8);   			//!< KeyBoard value
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
  /* USER CODE BEGIN 6 */`
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
