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
#include "servo.h"
#include "key.h"
#include "oled.h"
#include "serial.h"
#include "face_tracker.h"
#include "tracking_config.h"
#include <string.h>
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
static FaceTracker tracker;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if TRACK_OLED_ENABLE
static char display_cache[64];
static uint8_t display_cursor;
static void DisplayNumber(char *out, uint16_t value, uint8_t count)
{
  while (count != 0U) {
    --count; out[count] = (char)('0' + value % 10U); value /= 10U;
  }
}
static void DisplayInit(void)
{
  OLED_Init();
  OLED_Clear();
  OLED_ShowString(0, 0, "Face:HOME       ", 16, 0);
  OLED_ShowString(0, 2, "Pan: 090 deg    ", 16, 0);
  OLED_ShowString(0, 4, "Tilt:045 deg    ", 16, 0);
  OLED_ShowString(0, 6, "X:---- Y:----   ", 16, 0);
  memset(display_cache, 0, sizeof(display_cache));
}
/* At most one changed character per control period: bounded I2C latency. */
static void DisplayTask(void)
{
  char lines[4][17] = {"Face:HOME       ", "Pan: 090 deg    ",
                        "Tilt:045 deg    ", "X:---- Y:----   "};
  uint8_t i, row, col;
  if (!OLED_IsReady()) return;
  if (tracker.state == TRACK_ACTIVE) memcpy(lines[0] + 5, "FACE", 4);
  else if (tracker.state == TRACK_HOLD) memcpy(lines[0] + 5, "HOLD", 4);
  DisplayNumber(lines[1] + 5, (uint16_t)(tracker.pan.angle + 0.5f), 3);
  DisplayNumber(lines[2] + 5, (uint16_t)(tracker.tilt.angle + 0.5f), 3);
  if (tracker.state == TRACK_ACTIVE) {
    DisplayNumber(lines[3] + 2, tracker.face.x, 4);
    DisplayNumber(lines[3] + 9, tracker.face.y, 4);
  }
  for (i = 0U; i < 64U; ++i) {
    row = display_cursor / 16U; col = display_cursor % 16U;
    if (display_cache[display_cursor] != lines[row][col]) {
      display_cache[display_cursor] = lines[row][col];
      OLED_ShowChar(col * 8U, row * 2U, (uint8_t)lines[row][col], 16, 0);
      display_cursor = (display_cursor + 1U) % 64U;
      return;
    }
    display_cursor = (display_cursor + 1U) % 64U;
  }
}
#endif
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Key_Init();
  Servo_InitAll();
  FaceTracker_Init(&tracker, HAL_GetTick());
#if TRACK_OLED_ENABLE
  DisplayInit();
#endif
  /* Flush bytes/errors accumulated during startup, then arm UART reception. */
  Serial_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now;
    FaceFrame frame;
    Serial_Poll();
    now = HAL_GetTick();
    if ((uint32_t)(now - tracker.last_step_ms) >= TRACK_PERIOD_MS)
    {
      Key_Scan();
      if (Serial_GetLatest(&frame) &&
          (uint32_t)(now - frame.received_ms) <= TRACK_TIMEOUT_MS)
        FaceTracker_OnFrame(&tracker, &frame);
      if (FaceTracker_Update(&tracker, now))
      {
        Servo_SetAngle(&servo1, tracker.pan.angle);
        Servo_SetAngle(&servo2, tracker.tilt.angle);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,
            tracker.state == TRACK_ACTIVE ? GPIO_PIN_RESET : GPIO_PIN_SET);
#if TRACK_OLED_ENABLE
        DisplayTask();
#endif
      }
    }
    HAL_Delay(1);
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
