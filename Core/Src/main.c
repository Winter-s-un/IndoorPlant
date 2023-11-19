/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "string.h"
#include "stdio.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SHT20_I2C_ADDRESS 0x80
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float temperature = 0.00;
float humidity;
static char buffer[] = "\r\nData: ";
 static volatile int a = 0;
 static char dididi[20];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void floatToString(float value, char *buffer, int bufferSize, int precision) {
    int intValue = (int)value;
    int fractional = (int)((value - intValue) * 1000000); // 6位小数，可以根据需要调整
    snprintf(buffer, bufferSize, "%d.%06d", intValue, fractional);
}

void LED_Blink(void) {
    uint32_t previousMillis = 0;
    uint32_t currentMillis = HAL_GetTick();

    if (currentMillis - previousMillis >= 500) {
      //control the LED Blink
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      previousMillis = currentMillis;
    }
}


void TempHumidityRead(void) {
    uint32_t previousMillis = 0;
    uint32_t currentMillis = HAL_GetTick();

    if (currentMillis - previousMillis >= 200) {
      //read Temp & Humid
      uint8_t data[4];
      uint8_t command[2] = {0xE3, 0xE5};
      HAL_I2C_Master_Transmit(&hi2c1, 0x80, &command[0], 1, HAL_MAX_DELAY);
      HAL_Delay(100);

      HAL_I2C_Master_Receive(&hi2c1, 0x80, data, 4, HAL_MAX_DELAY);

      // change the data to readable
      uint16_t rawTemperature = (data[0] << 8) | data[1];
      temperature = -46.85f + 175.72f * (float)rawTemperature / 65536.0f;

      // HAL_Delay(500);
      // HAL_I2C_Master_Transmit(&hi2c1, 0x80, &command[1], 1, HAL_MAX_DELAY);
      // HAL_Delay(200);

      // HAL_I2C_Master_Receive(&hi2c1, 0x80, data, 4, HAL_MAX_DELAY);
      // uint16_t rawHumidity = (data[2] << 8) | data[3];
      // humidity = -6.0f + 125.0f * (float)rawHumidity / 65536.0f;



        previousMillis = currentMillis;
    }
}

void BlueToothSend(void)
{
    uint32_t previousMillis = 0;
    uint32_t currentMillis = HAL_GetTick();
    if (currentMillis - previousMillis >= 200)
    {
      float t = temperature;
      char byteBuffer[21];


      floatToString(t, dididi, sizeof(dididi), 6);
      // // uint32_t tempData;
      //memcpy(&dididi, &t, sizeof(19));
      // // tempData = (tempData); 
      // memcpy(buffer, &t, sizeof(50));
      
      // dididi = byteBuffer;
      // static int rData[4] = {0};
    
       HAL_UART_Transmit(&huart1, (uint8_t*)dididi, sizeof(dididi)-1, HAL_MAX_DELAY);

      //HAL_UART_Receive(&huart1,(uint8_t*)rData,sizeof(rData)-1, HAL_MAX_DELAY);
      a++;
      previousMillis = currentMillis;
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  static float temp = 0.0f;
  static float humid = 0.0f;

  LED_Blink();


  TempHumidityRead();

 

  temp = temperature;
  humid = humidity;
  
   BlueToothSend();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
