/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  LED_MODE_ALL_OFF = 0,
  LED_MODE_SINGLE_PRESS,    // Green LED only
  LED_MODE_DOUBLE_PRESS,    // Green and Blue LEDs
  LED_MODE_LONG_PRESS,      // All LEDs blinking
  LED_MODE_COUNT
} LED_Mode_t;

typedef enum {
  BUTTON_IDLE = 0,
  BUTTON_PRESSED,
  BUTTON_RELEASED,
  BUTTON_WAIT_DOUBLE
} Button_State_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static LED_Mode_t currentLedMode = LED_MODE_ALL_OFF;
static Button_State_t buttonState = BUTTON_IDLE;
static uint32_t buttonPressTime = 0;
static uint32_t buttonReleaseTime = 0;
static uint32_t lastDebounceTime = 0;
static uint8_t clickCount = 0;
static uint8_t buttonPressed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static void ProcessButtonPress(void);
static void UpdateLEDs(void);
static void SetLEDMode(LED_Mode_t mode);
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
  int count = 0;
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("Button LED Control System Started\n");
  printf("Press patterns:\n");
  printf("- Single press: Green LED\n");
  printf("- Double press: Green + Blue LEDs\n");
  printf("- Long press: All LEDs blinking\n\n");
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    ProcessButtonPress();
    UpdateLEDs();
    HAL_Delay(10); // 10ms loop delay for responsive button handling
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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

/**
  * @brief  Process button press events and detect press patterns
  * @retval None
  */
static void ProcessButtonPress(void)
{
  uint32_t currentTime = HAL_GetTick();
  
  // Handle button state machine
  switch (buttonState) {
    case BUTTON_IDLE:
      // Waiting for button press (handled in interrupt)
      break;
      
    case BUTTON_PRESSED:
      // Check for long press
      if (buttonPressed && (currentTime - buttonPressTime) >= BUTTON_LONG_PRESS_TIME) {
        SetLEDMode(LED_MODE_LONG_PRESS);
        buttonState = BUTTON_IDLE;
        clickCount = 0;
        printf("Long press detected! Mode: All LEDs blinking\n");
      }
      break;
      
    case BUTTON_RELEASED:
      // Button was released, wait for possible double click
      buttonState = BUTTON_WAIT_DOUBLE;
      break;
      
    case BUTTON_WAIT_DOUBLE:
      // Check if double click window expired
      if (currentTime - buttonReleaseTime > BUTTON_DOUBLE_CLICK_TIME) {
        if (clickCount == 1) {
          SetLEDMode(LED_MODE_SINGLE_PRESS);
          printf("Single press detected! Mode: Green LED only\n");
        } else if (clickCount == 2) {
          SetLEDMode(LED_MODE_DOUBLE_PRESS);
          printf("Double press detected! Mode: Green + Blue LEDs\n");
        }
        clickCount = 0;
        buttonState = BUTTON_IDLE;
      }
      break;
  }
}

/**
  * @brief  Update LEDs based on current mode
  * @retval None
  */
static void UpdateLEDs(void)
{
  static uint32_t lastBlinkTime = 0;
  static uint8_t blinkState = 0;
  uint32_t currentTime = HAL_GetTick();
  
  switch (currentLedMode) {
    case LED_MODE_ALL_OFF:
      HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
      break;
      
    case LED_MODE_SINGLE_PRESS:
      HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
      break;
      
    case LED_MODE_DOUBLE_PRESS:
      HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
      break;
      
    case LED_MODE_LONG_PRESS:
      // Blink all LEDs
      if (currentTime - lastBlinkTime >= 250) {
        blinkState = !blinkState;
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, blinkState ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, blinkState ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, blinkState ? GPIO_PIN_SET : GPIO_PIN_RESET);
        lastBlinkTime = currentTime;
      }
      break;
      
    default:
      break;
  }
}

/**
  * @brief  Set LED mode and handle mode transitions
  * @param  mode: New LED mode to set
  * @retval None
  */
static void SetLEDMode(LED_Mode_t mode)
{
  currentLedMode = mode;
  
  // Reset LEDs when changing modes
  if (mode != LED_MODE_LONG_PRESS) {
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
  }
}

/**
  * @brief  EXTI line detection callback
  * @param  GPIO_Pin: Specifies the pins connected to EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t currentTime = HAL_GetTick();
  
  if (GPIO_Pin == USER_BUTTON_Pin) {
    // Debounce check
    if (currentTime - lastDebounceTime < BUTTON_DEBOUNCE_TIME) {
      return;
    }
    lastDebounceTime = currentTime;
    
    // Check current button state
    if (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_SET) {
      // Button pressed (Nucleo button is active HIGH)
      buttonPressed = 1;
      buttonPressTime = currentTime;
      buttonState = BUTTON_PRESSED;
      
      // Check if this is a subsequent click for double-click detection
      if (currentTime - buttonReleaseTime < BUTTON_DOUBLE_CLICK_TIME) {
        clickCount++;
      } else {
        clickCount = 1;
      }
    } else {
      // Button released
      buttonPressed = 0;
      buttonReleaseTime = currentTime;
      
      // Only register as released if it wasn't a long press
      if (buttonState == BUTTON_PRESSED && (currentTime - buttonPressTime) < BUTTON_LONG_PRESS_TIME) {
        buttonState = BUTTON_RELEASED;
      } else if (buttonState == BUTTON_PRESSED && (currentTime - buttonPressTime) >= BUTTON_LONG_PRESS_TIME) {
        // Long press already handled, reset to idle
        buttonState = BUTTON_IDLE;
      }
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  /* Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_BLUE_Pin|LED_RED_Pin, GPIO_PIN_RESET);
  
  /* Configure GPIO pins : LED_GREEN_Pin LED_BLUE_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_BLUE_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /* Configure User Button pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);
  
  /* EXTI interrupt init */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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
