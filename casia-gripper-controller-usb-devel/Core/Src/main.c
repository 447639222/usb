/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "adc.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"
#include "math.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"

#include "oled.h"
#include "bsp_can.h"
#include "pid.h"
#include "usbd_cdc_if.h"
//#include "remote_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
PID_TypeDef motor_pid[4];
int32_t set_spd = 0;
int32_t set_spd1 = 0;
int32_t set_spd2 = 0;
static int key_sta = 0;
int speed_step_sign = +1;
uint32_t a;
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern uint32_t buffsize;
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
extern uint32_t UserTxBufPtrOut;

char message[50];
char TxBuffer[50];
uint16_t TIM_COUNT[2];
#define SpeedStep 500
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Key_Scan()
{
    if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
    {
        if (key_sta == 0)
        {
            key_sta = 1;
            set_spd1 += SpeedStep * speed_step_sign;

            if (set_spd1 > 1000)
            {
                speed_step_sign = -1;
            }
            if (set_spd1 < -1000)
            {
                speed_step_sign = 1;
            }
        }
    }
    else
    {
        key_sta = 0;
    }

}



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
  MX_CAN1_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM10_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
    // Power on the motor output 24V (PH2)
    HAL_GPIO_WritePin(PWR_GPIO_Port, PWR_Pin, GPIO_PIN_SET);
    // Power on the motor output 24V (PH3)
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, GPIO_PIN_SET);

    HAL_TIM_Base_Start_IT(&htim10);

    // Config & Start CAN1
    BSP_CAN1_Init();

    // OLED init
    led_off();
    oled_init();
    oled_clear(Pen_Clear);
    oled_refresh_gram();

    /*< 初始化PID参数 >*/
    for (int i = 0; i < 4; i++)
    {
        pid_init(&motor_pid[i]);
        motor_pid[i].f_param_init(&motor_pid[i], PID_Speed, 16384, 5000, 10, 0,
                8000, 0, 1.5, 0.1, 0);
    }

    Key_Scan();
    set_spd1 = 0;
    set_spd2 = 0;
    set_spd = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while(1)
    {

        Main_loop();
        if(buffsize==4){
        	set_spd=UserRxBufferFS[1]-48;
//      set_spd = UserRxBufferFS[2]-48;
        }
        if(buffsize==5){
        	 set_spd=UserRxBufferFS[1]-48;
        	 set_spd=set_spd*10;
        	 set_spd =set_spd+UserRxBufferFS[2]-48;
        }
        if(UserRxBufferFS[0]=='a'){
                      	set_spd1 = set_spd*100;
              		}
      	if(UserRxBufferFS[0]=='b'){
      					set_spd2 = set_spd*100;
                    }
    }
//    for(int k=1;k<buffsize;k++){
//    				set_spd=set_spd*10+(UserRxBufferFS[k]-48);
//    		}

    while (1)
    {
        Key_Scan();

        for (int i = 0; i < 3; i++)
        {
            motor_pid[i].target = set_spd1;
            motor_pid[i].f_cal_pid(&motor_pid[i], moto_chassis[i].speed_rpm);

        }
        motor_pid[3].target = set_spd2;
                   motor_pid[3].f_cal_pid(&motor_pid[3], moto_chassis[3].speed_rpm);

        set_moto_current(&hcan1, motor_pid[0].output, motor_pid[1].output,
                motor_pid[2].output, motor_pid[3].output);

        HAL_Delay(10);      //PID控制频率100HZ
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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

    if (htim->Instance == TIM10)
    {
        oled_clear(Pen_Clear);
        oled_showstring1(0, 2, "Parameters:");
        sprintf(message, "%c", UserRxBufferFS[0]);
        //sprintf(message, "%d", moto_chassis[1].last_angle);
        oled_showstring1(1, 2, message);
        sprintf(message, "%ld",set_spd);
        //sprintf(message, "%d", moto_chassis[1].angle);
        oled_showstring1(2, 2, message);
        //sprintf(message, "%d", moto_chassis[1].speed_rpm);
        sprintf(message, "%ld",set_spd1);
        oled_showstring1(3, 2, message);
        //sprintf(message, "%.3f", moto_chassis[1].real_current);
        sprintf(message, "%ld",set_spd2);
        oled_showstring1(4, 2, message);
        oled_refresh_gram();
    }
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
