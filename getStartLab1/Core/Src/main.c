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
#include "cmsis_os.h"

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
UART_HandleTypeDef huart2;

/* Definitions for autoGreen */
osThreadId_t autoGreenHandle;
const osThreadAttr_t autoGreen_attributes = {
  .name = "autoGreen",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for autoYellow */
osThreadId_t autoYellowHandle;
const osThreadAttr_t autoYellow_attributes = {
  .name = "autoYellow",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for autoRed */
osThreadId_t autoRedHandle;
const osThreadAttr_t autoRed_attributes = {
  .name = "autoRed",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for pedRed */
osThreadId_t pedRedHandle;
const osThreadAttr_t pedRed_attributes = {
  .name = "pedRed",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for pedGreen */
osThreadId_t pedGreenHandle;
const osThreadAttr_t pedGreen_attributes = {
  .name = "pedGreen",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for binarySemAutoGreen */
osSemaphoreId_t binarySemAutoGreenHandle;
const osSemaphoreAttr_t binarySemAutoGreen_attributes = {
  .name = "binarySemAutoGreen"
};
/* Definitions for binarySemAutoYellow */
osSemaphoreId_t binarySemAutoYellowHandle;
const osSemaphoreAttr_t binarySemAutoYellow_attributes = {
  .name = "binarySemAutoYellow"
};
/* Definitions for binarySemAutoRed */
osSemaphoreId_t binarySemAutoRedHandle;
const osSemaphoreAttr_t binarySemAutoRed_attributes = {
  .name = "binarySemAutoRed"
};
/* Definitions for binarySemPedGreen */
osSemaphoreId_t binarySemPedGreenHandle;
const osSemaphoreAttr_t binarySemPedGreen_attributes = {
  .name = "binarySemPedGreen"
};
/* Definitions for binarySemPedRed */
osSemaphoreId_t binarySemPedRedHandle;
const osSemaphoreAttr_t binarySemPedRed_attributes = {
  .name = "binarySemPedRed"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartAutoGreen(void *argument);
void StartAutoYellow(void *argument);
void StartAutoRed(void *argument);
void StartPedRed(void *argument);
void startPedGreen(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t dataTask1[] = "Task1: How are you?\r\n";
uint8_t dataTask2[] = "Task2: I am fine and you.\r\n";
uint8_t dataTask3[] = "Task3: It is nice you see here, Task3\r\n";


int flag = 0;

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of binarySemAutoGreen */
  binarySemAutoGreenHandle = osSemaphoreNew(1, 1, &binarySemAutoGreen_attributes); // Initialize to 1 for phase 1.

  /* creation of binarySemAutoYellow */
  binarySemAutoYellowHandle = osSemaphoreNew(1, 0, &binarySemAutoYellow_attributes);

  /* creation of binarySemAutoRed */
  binarySemAutoRedHandle = osSemaphoreNew(1, 0, &binarySemAutoRed_attributes);

  /* creation of binarySemPedGreen */
  binarySemPedGreenHandle = osSemaphoreNew(1, 0, &binarySemPedGreen_attributes);

  /* creation of binarySemPedRed */
  binarySemPedRedHandle = osSemaphoreNew(1, 1, &binarySemPedRed_attributes); // Initialize to 1 for phase 1.

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of autoGreen */
  autoGreenHandle = osThreadNew(StartAutoGreen, NULL, &autoGreen_attributes);

  /* creation of autoYellow */
  autoYellowHandle = osThreadNew(StartAutoYellow, NULL, &autoYellow_attributes);

  /* creation of autoRed */
  autoRedHandle = osThreadNew(StartAutoRed, NULL, &autoRed_attributes);

  /* creation of pedRed */
  pedRedHandle = osThreadNew(StartPedRed, NULL, &pedRed_attributes);

  /* creation of pedGreen */
  pedGreenHandle = osThreadNew(startPedGreen, NULL, &pedGreen_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartAutoGreen */
/**
  * @brief  Function implementing the autoGreen thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartAutoGreen */
void StartAutoGreen(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

	  // Phase 1 and 7.
	  // Acquire autoGreen semaphore.
	  osSemaphoreAcquire(binarySemAutoGreenHandle, osWaitForever);
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3, GPIO_PIN_SET);//Auto Green LED

	  while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)); // Wait for button press.

	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3, GPIO_PIN_RESET);
	  osSemaphoreRelease(binarySemAutoYellowHandle); // Release the semaphore for Auto Yellow to Transition to phase 2.

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartAutoYellow */
/**
* @brief Function implementing the autoYellow thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAutoYellow */
void StartAutoYellow(void *argument)
{
  /* USER CODE BEGIN StartAutoYellow */
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreAcquire(binarySemAutoYellowHandle, osWaitForever);
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5, GPIO_PIN_SET);

	  osDelay(3000);

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // initial state.
	  osSemaphoreRelease(binarySemAutoRedHandle);

	  osSemaphoreAcquire(binarySemAutoYellowHandle, osWaitForever);
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5, GPIO_PIN_SET);

	  osDelay(2000);

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  }
  /* USER CODE END StartAutoYellow */
}

/* USER CODE BEGIN Header_StartAutoRed */
/**
* @brief Function implementing the autoRed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAutoRed */
void StartAutoRed(void *argument)
{
  /* USER CODE BEGIN StartAutoRed */
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreAcquire(binarySemAutoRedHandle, osWaitForever);
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4, GPIO_PIN_SET);

	  osDelay(12000);
	  osSemaphoreRelease(binarySemAutoYellowHandle);

	  osDelay(2000);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	  osSemaphoreRelease(binarySemAutoGreenHandle);


  }
  /* USER CODE END StartAutoRed */
}

/* USER CODE BEGIN Header_StartPedRed */
/**
* @brief Function implementing the pedRed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPedRed */
void StartPedRed(void *argument)
{
  /* USER CODE BEGIN StartPedRed */
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreAcquire(binarySemPedRedHandle, osWaitForever);
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, GPIO_PIN_SET); // Initial state for phase 1.

	  while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)); // Wait for button press.

	  osDelay(4000); // Wait 4 seconds to release semaphore for phase 4.

	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, GPIO_PIN_RESET); // Set off.

	  osSemaphoreRelease(binarySemPedGreenHandle); // For phase 4.
  }
  /* USER CODE END StartPedRed */
}

/* USER CODE BEGIN Header_startPedGreen */
/**
* @brief Function implementing the pedGreen thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startPedGreen */
void startPedGreen(void *argument)
{
  /* USER CODE BEGIN startPedGreen */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9, GPIO_PIN_RESET); // Initial State.

	  osSemaphoreAcquire(binarySemPedGreenHandle, osWaitForever);

	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9, GPIO_PIN_SET);

	  osDelay(10000);

	  osSemaphoreRelease(binarySemPedRedHandle);


  }
  /* USER CODE END startPedGreen */
}

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
