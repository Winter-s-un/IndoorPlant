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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
float temperature;
float humidity;
float DQtemperature;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// float SHT20_ReadTemperature(void) {
//     uint8_t data[3];

//     // 发�?�温度测量命�?
//     uint8_t command = 0xF3;
//     if (HAL_I2C_Master_Transmit(&hi2c1, 0x80, &command, 1, HAL_MAX_DELAY) != HAL_OK) {
//         // 发�?�命令失�?
//         Error_Handler();
//     }

//     // 等待测量完成（具体时间根据SHT20规格来确定）
//     HAL_Delay(100); 

//     // 读取温度数据
//     if (HAL_I2C_Master_Receive(&hi2c1, 0x80, data, 3, HAL_MAX_DELAY) != HAL_OK) {
//         // 读取数据失败
//         Error_Handler();
//     }

//     // 解析温度数据
//     uint16_t rawTemperature = (data[0] << 8) | data[1];
//     float temperature = -46.85 + 175.72 * (float)rawTemperature / 65536.0;

//     return temperature;
// }

void LED_Blink(void) {
    static uint32_t previousMillis = 0;
    uint32_t currentMillis = HAL_GetTick();

    if (currentMillis - previousMillis >= 1000) {
        // 控制LED状�?�切�?
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        previousMillis = currentMillis;
    }
}

// void TempRead(void) {
//     static uint32_t previousMillis = 0;
//     uint32_t currentMillis = HAL_GetTick();

//     if (currentMillis - previousMillis >= 100) {
//         // 读取温度
//         temperature = SHT20_ReadTemperature();
//         // 处理温度数据

//         previousMillis = currentMillis;
//     }
// }

void TempHumidityRead(void) {
    static uint32_t previousMillis = 0;
    uint32_t currentMillis = HAL_GetTick();

    if (currentMillis - previousMillis >= 300) {
        // 读取温度和湿�?
        uint8_t data[4];
        uint8_t command = 0xF5;
        HAL_I2C_Master_Transmit(&hi2c1, 0x80, &command, 1, HAL_MAX_DELAY);
        HAL_Delay(200);

        HAL_I2C_Master_Receive(&hi2c1, 0x80, data, 4, HAL_MAX_DELAY);

        // 解析温度和湿度数�?
        uint16_t rawTemperature = (data[0] << 8) | data[1];
        temperature = -46.85 + 175.72 * (float)rawTemperature / 65536.0;

        uint16_t rawHumidity = (data[2] << 8) | data[3];
        humidity = -6.0 + 125.0 * (float)rawHumidity / 65536.0;

        // 处理温度和湿度数�?

        previousMillis = currentMillis;
    }
}

// 写入�?个位
void OW_WriteBit(uint8_t bit, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); // 拉低总线
    HAL_Delay(2); // 持续时间表示 0
    if (bit) {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET); // 拉高总线
    }
    HAL_Delay(60); // 持续时间表示 1
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET); // 释放总线
}

// 读取�?个位
uint8_t OW_ReadBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); // 拉低总线
    HAL_Delay(2); // 允许总线稳定
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET); // 拉高总线
    HAL_Delay(12); // 等待读取信号
    uint8_t bit = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin); // 读取总线
    HAL_Delay(46); // 等待总线恢复到空闲状�?
    return bit;
}

// 读取�?个字�?
uint8_t OW_ReadByte(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte |= (OW_ReadBit(GPIOx, GPIO_Pin) << i);
    }
    return byte;
}

float DS18B20_ReadTemperature(void) {

  static uint32_t previousMillis = 0;
    uint32_t currentMillis = HAL_GetTick();

    if (currentMillis - previousMillis >= 1000) {
        // 发送初始化序列，开始温度转换
    OW_WriteBit(GPIO_PIN_RESET, DQ_GPIO_Port, DQ_Pin);
    OW_WriteBit(0xCC, DQ_GPIO_Port, DQ_Pin);
    OW_WriteBit(0x44, DQ_GPIO_Port, DQ_Pin);

    // 等待温度转换完成（DS18B20转换时间取决于分辨率）
    HAL_Delay(750); // 延时时间根据DS18B20规格手册来确定

    // 发送初始化序列，读取温度数据
    OW_WriteBit(GPIO_PIN_RESET, DQ_GPIO_Port, DQ_Pin);
    OW_WriteBit(0xCC, DQ_GPIO_Port, DQ_Pin);
    OW_WriteBit(0xBE, DQ_GPIO_Port, DQ_Pin);

    // 读取温度数据
    uint8_t data[9];
    for (int i = 0; i < 9; i++) {
        data[i] = OW_ReadByte(DQ_GPIO_Port, DQ_Pin);
    }

    // 计算温度值
    int16_t rawTemperature = (data[1] << 8) | data[0];
    DQtemperature = (float)rawTemperature / 16.0;
        previousMillis = currentMillis;
    }
    

    // return temperature;
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
  //   HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  // HAL_Delay(1000);
  // HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  // HAL_Delay(1000);
  static float temp = 0.0f;
  static float humid = 0.0f;
  static float dqTemp = 0.0f;
  // float temprature = SHT20_ReadTemperature();

        LED_Blink();

        // 读取温度
        // TempRead();
        TempHumidityRead();
        // DS18B20_ReadTemperature();
  temp = temperature;
  dqTemp = DQtemperature;
  humid = humidity;
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
