/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  * * Portions Copyright (c) 2026 [Your Name]
  * Licensed under the MIT License.
  
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
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

// water sensor
uint32_t water_raw = 0;
float water_percent = 0;

// DHT11 Sensor
uint8_t dht_data[5];
uint8_t temp_val = 0;
uint8_t hum_val = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us (uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);  // Reset the TIM2 counter to 0
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);  // Wait until it reaches the requested 'us'
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // start TIM2
  HAL_TIM_Base_Start(&htim2);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    // Start ADC

    HAL_ADC_Start(&hadc1);
    
    // check for the sensor
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
      {
        // get the value of the sensor
        water_raw = HAL_ADC_GetValue(&hadc1);

        // convert the value to percentage
        water_percent = ((float)water_raw / 4095.0F) * 100.0F;

      }
      HAL_ADC_Stop(&hadc1);

      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // force pin to 0
      HAL_Delay(18); // sensor needs a start signal of at least 18ms

      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // let pin go high again
      delay_us(30); // give sensor time to take over wire

      GPIO_InitTypeDef GPIO_InitStruct = {0}; // temp struct for gpio config
      GPIO_InitStruct.Pin = GPIO_PIN_4; // set pin 4 as pin to read
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // set pin to input mode
      GPIO_InitStruct.Pull = GPIO_NOPULL; // set PUPDR for pin 4
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 

      delay_us(40); // some time to let sensor take control
      if (!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4))) // check if sensor is ready by low signal
      {
        while (!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4))); // wait for sensor pulse
        while ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4))); // 

        for (int i = 0; i < 5; i++) // counts 5 bytes
        {
          for (int j = 0; j < 8; j++) // counts the 8 bits in each byte
          {
            while (!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4))); // checking for the 50us low pulse that happens at each bit
            delay_us(40); // wait for enough time to get the signal of a "1" or "0"

              if (!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4))) // if the pin is low, then it's a "0"
              {
                dht_data[i] &= ~(1 << (7-j)); // clear the bit at position (7-j) in dht_data[i]
              }

              else // if the pin is high, then it's a "1"
              {
                dht_data[i] |= (1 << (7-j)); // set the bit at position (7-j) in dht_data[i]
                while ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4))); // wait for the high pulse to end
              }
          }
        }
        uint8_t checksum = dht_data[0] + dht_data[1] + dht_data[2] + dht_data[3]; // calculate checksum

        if (checksum == dht_data[4]) // check if checksum is correct
        {
          temp_val = dht_data[2]; // get temperature value from data array
          hum_val = dht_data[0]; // get humidity value from data array
        }
      }

      HAL_Delay(2000); // wait 2 seconds before next reading
    
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
#ifdef USE_FULL_ASSERT
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
