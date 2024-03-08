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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led.h"
#include "stdio.h"
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum
{
  HIGH = 1,
  LOW = 0
} frequence_choose;

int mode = 0, choose = 0;
int R = 1, K = 1, N = 0;
float vilocity, pulse;
unsigned int f, aim_pwm = 8000;
bool lock = false, change_PWM = false;
#define PI 3.14
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
void change_pulse(void)
{
  u16 arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
  if (lock == false)
  {
    double v = getADC(&hadc2);
    if (v < 1)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, arr / 10);
      pulse = 10;
    }
    else if (v > 3)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, arr * 0.85);
      pulse = 85;
    }
    else
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, arr * (0.375 * v - 0.275));
      pulse = 37.5 * v - 27.5;
    }
  }
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 当触发时，100ms执行一次
void change_pwm(void)
{
  u16 arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
  led_Toggle(2);
  if (frequence_choose == LOW) // 1000->500
  {
    aim_pwm += 200;
    __HAL_TIM_SET_AUTORELOAD(&htim2, 1000000 / aim_pwm);
  }
  else // 500->1000
  {
    aim_pwm -= 200;
    __HAL_TIM_SET_AUTORELOAD(&htim2, 1000000 / aim_pwm);
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
  u32 uwtick;
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
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  led_init();
  LCD_Init();
  LCD_Clear(Black);
  LCD_SetBackColor(Black);
  LCD_SetTextColor(White);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim17, TIM_CHANNEL_1);
  while (1)
  {
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0) // B1
    {
      while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0)
        ;
      mode++;
      if (mode == 3)
        mode = 0;
    }
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0) // B2
    {
      while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0)
        ;
      if (mode == 0)
      {
        change_PWM = true;
      }
      else if (mode == 1)
      {
        if (choose == 0)
          choose = 1;
        else
          choose = 0;
      }
    }
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0) // B3
    {
      while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0)
        ;
      if (choose == 0)
      {
        R++;
        if (R == 11)
          R = 1;
      }
      else
      {
        K++;
        if (K == 11)
          K = 1;
      }
    }
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0) // B4
    {
      uwtick = HAL_GetTick();
      while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0)
      {
        while (!((SysTick->CTRL) & (1 << 16)))
          ;
        uwtick++;
      }
      if (HAL_GetTick() - uwtick < 2000) // 2秒的判断时间点
      {
        // 短按
        if (mode == 1)
        {
          if (choose == 0)
          {
            R--;
            if (R == 0)
              R = 10;
          }
          else
          {
            K--;
            if (K == 0)
              K = 10;
          }
        }
        else if (mode == 0)
        {
          lock = false;
          led_OFF(3);
        }
      }
      else
      {
        // 长按
        if (mode == 1)
        {
          if (choose == 0)
          {
            R--;
            if (R == 0)
              R = 10;
          }
          else
          {
            K--;
            if (K == 0)
              K = 10;
          }
        }
        else if (mode == 0)
        {
          lock = true;
          led_ON(3);
        }
      }
    }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
char text[30];
int change_pwm_counter = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    if (change_PWM == true)
    {
      change_pwm();
      change_pwm_counter++;
      if (change_pwm_counter == 40)
      {
        if (frequence_choose == LOW)
        {
          frequence_choose = HIGH;
          aim_pwm = 8000;
          __HAL_TIM_SET_AUTORELOAD(&htim2, 1000000 / aim_pwm);
        }
        else
        {
          frequence_choose = LOW;
          aim_pwm = 4000;
          __HAL_TIM_SET_AUTORELOAD(&htim2, 1000000 / aim_pwm);
        }
        change_PWM = false;
        led_OFF(2);
        N++;
        change_pwm_counter = 0;
      }
    }
    if (mode == 0)
      led_ON(1);
    else
      led_OFF(1);
    if (lock == false)
    {
      change_pulse();
    }
    if (mode == 0)
    {
      LCD_DisplayStringLine(Line2, (u8 *)"        DATA    ");
      if (frequence_choose == LOW)
      {
        sprintf(text, "     M=L   ");
      }
      else
      {
        sprintf(text, "     M=H   ");
      }
      LCD_DisplayStringLine(Line4, (u8 *)text);
      sprintf(text, "     P=%d%   ", pulse);
      LCD_DisplayStringLine(Line5, (u8 *)text);
      vilocity = (f * 2 * PI * R) / (100 * K);
      sprintf(text, "     V=%.1f      ", vilocity);
      LCD_DisplayStringLine(Line6, (u8 *)text);
    }
    else if (mode == 1)
    {
      LCD_DisplayStringLine(Line2, (u8 *)"        PARA    ");
      sprintf(text, "     R=%d      ", R);
      LCD_DisplayStringLine(Line4, (u8 *)text);
      sprintf(text, "     K=%d      ", K);
      LCD_DisplayStringLine(Line5, (u8 *)text);
      LCD_DisplayStringLine(Line6, (u8 *)"                  ");
    }
    else if (mode == 2)
    {
      LCD_DisplayStringLine(Line2, (u8 *)"        RECD    ");
      sprintf(text, "     N=%d      ", N);
      LCD_DisplayStringLine(Line4, (u8 *)text);
      sprintf(text, "     MH=502.4      ");
      LCD_DisplayStringLine(Line5, (u8 *)text);
      sprintf(text, "     ML=251.2      ");
      LCD_DisplayStringLine(Line6, (u8 *)text);
    }
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM17)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      u16 CCR = HAL_TIM_ReadCapturedValue(&htim17, TIM_CHANNEL_1) + 1;
      __HAL_TIM_SetCounter(&htim17, 0);
      f = 1000000 / CCR;
      HAL_TIM_IC_Start_IT(&htim17, TIM_CHANNEL_1);
    }
  }
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
