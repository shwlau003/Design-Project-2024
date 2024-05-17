/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_stm32f0.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Threshold value for ADC reading */
#define THRESHOLD 1900 // Adjust threshold value as needed

/* LED pins */
#define LED_LEFT_Pin  GPIO_PIN_7
#define LED_LEFT_GPIO_Port  GPIOB
#define LED_FRONT_Pin GPIO_PIN_6
#define LED_FRONT_GPIO_Port GPIOB
#define LED_RIGHT_Pin GPIO_PIN_5
#define LED_RIGHT_GPIO_Port GPIOB
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//JUSTIN - START 1/3
uint16_t adc_value_left;
uint16_t adc_value_front;
uint16_t adc_value_right;

// char LCD_buffer[]; 
//JUSTIN - END
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
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // init_LCD();

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //JUSTIN - START 2/3
    // lcd_command(CLEAR);   

    ADC1 -> CHSELR = ADC_CHSELR_CHSEL3; //left sensor's channel
    HAL_ADC_Start(&hadc);
    HAL_Delay(10);
    while(HAL_ADC_PollForConversion(&hadc,HAL_MAX_DELAY)!= HAL_OK);
    HAL_ADC_Stop(&hadc);
    adc_value_left = HAL_ADC_GetValue(&hadc);

    ADC1 -> CHSELR = ADC_CHSELR_CHSEL4; //front sensor's channel
    HAL_ADC_Start(&hadc);
    HAL_Delay(10);
    while(HAL_ADC_PollForConversion(&hadc,HAL_MAX_DELAY)!= HAL_OK);
    HAL_ADC_Stop(&hadc);
    adc_value_front = HAL_ADC_GetValue(&hadc);

    ADC1 -> CHSELR = ADC_CHSELR_CHSEL5; //right sensor's channel
    HAL_ADC_Start(&hadc);
    HAL_Delay(10);
    while(HAL_ADC_PollForConversion(&hadc,HAL_MAX_DELAY)!= HAL_OK);
    HAL_ADC_Stop(&hadc);
    adc_value_right = HAL_ADC_GetValue(&hadc);

    if (adc_value_left > THRESHOLD) {
        // Activate LED for PA3 (left)
        HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, GPIO_PIN_RESET);
    }

    if (adc_value_front > THRESHOLD) {
        // Activate LED for PA4 (front)
        HAL_GPIO_WritePin(LED_FRONT_GPIO_Port, LED_FRONT_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(LED_FRONT_GPIO_Port, LED_FRONT_Pin, GPIO_PIN_RESET);
    }

    if (adc_value_right > THRESHOLD) {
        // Activate LED for PA5 (right)
        HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, GPIO_PIN_RESET);
    }

    // // Prints to LCD
    // sprintf(LCD_buffer, "CHANNEL 4: %d", adc_value_front); // Convert integer to string
    // lcd_putstring(LCD_buffer);
    // lcd_command(LINE_TWO);
    // sprintf(LCD_buffer, "CHANNEL 5: %d", adc_value_right); // Convert integer to string
    // lcd_putstring(LCD_buffer);
    // HAL_Delay(500);

    //JUSTIN - END
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//JUSTIN - START 3/3
/* I took this code from the adc.c file - In the init section.
* Regrettably setting up the ADC for reading multiple channels is not great without using DMA (scary)
* So you would need to just look at that file and work out how to modify your set channel accordingly.
*/
void adc_set_channel(uint16_t adcChannelNum) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = adcChannelNum;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {
      Error_Handler();
    }
}
//JUSTIN - END
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
