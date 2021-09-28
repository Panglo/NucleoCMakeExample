/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
osThreadId_t rxTaskHandle;
const osThreadAttr_t rxTask_attributes = {
  .name = "rxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t txTaskHandle;
const osThreadAttr_t txTask_attributes = {
  .name = "txTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t cliTaskHandle;
const osThreadAttr_t cliTask_attributes = {
  .name = "cliTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

// for rx task
size_t rxBufferPos = 0; 
uint8_t rxBuffer[RX_BUFFER_SIZE]; // Stores the partially received message
uint8_t rxByte; // Stores the recieved byte
size_t txBufferSize = 0;
uint8_t txBuffer[TX_BUFFER_SIZE];
char cliBuffer[CLI_BUFFER_SIZE];

// For rx stream
uint8_t rxStreamBufferWorkspace[RX_BUFFER_SIZE+1];
StaticStreamBuffer_t rxStreamBufferStruct;
StreamBufferHandle_t rxStreamBufferHandle;

// For tx message buffer
uint8_t txMessageBufferWorkspace[TX_BUFFER_SIZE+1];
StaticMessageBuffer_t txMessageBufferStruct;
MessageBufferHandle_t txMessageBufferHandle;
SemaphoreHandle_t txCompleteSemaphore;


uint8_t cliMessageBufferWorkspace[CLI_BUFFER_SIZE+1];
StaticMessageBuffer_t cliMessageBufferStruct;
MessageBufferHandle_t cliMessageBufferHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void StartRxTask(void * args);
void StartTxTask(void * args);
void StartCLITask(void * args);

extern void RegisterCLICommands(void);
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  RegisterCLICommands();

  HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, (uint8_t*) "Starting...\n", sizeof "Starting...\n", 1000);
  if (status != HAL_OK) {
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    for (;;){}
  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  txCompleteSemaphore = xSemaphoreCreateBinary();
  if (txCompleteSemaphore == NULL) Error_Handler();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  rxStreamBufferHandle = xStreamBufferCreateStatic(RX_BUFFER_SIZE, 1, rxStreamBufferWorkspace, &rxStreamBufferStruct);
  txMessageBufferHandle = xMessageBufferCreateStatic(TX_BUFFER_SIZE, txMessageBufferWorkspace, &txMessageBufferStruct);
  cliMessageBufferHandle = xMessageBufferCreateStatic(CLI_BUFFER_SIZE, cliMessageBufferWorkspace, &cliMessageBufferStruct);
/* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  rxTaskHandle = osThreadNew(StartRxTask, NULL, &rxTask_attributes);
  txTaskHandle = osThreadNew(StartTxTask, NULL, &txTask_attributes);
  cliTaskHandle = osThreadNew(StartCLITask, NULL, &cliTask_attributes);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (rxByte != (uint8_t) '\r'){ // Ignore carriage return character
    xStreamBufferSendFromISR(rxStreamBufferHandle, &rxByte, 1, NULL);
  }

  HAL_UART_Receive_IT(huart, &rxByte, 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  xSemaphoreGiveFromISR(txCompleteSemaphore, NULL);
}


void StartRxTask(void * args)
{
  HAL_UART_Receive_IT(&huart2, &rxByte, 1);
  size_t messageIndex = 0;
  uint8_t* currentMessage = pvPortMalloc(RX_BUFFER_SIZE);

  if (currentMessage == NULL) {
    Error_Handler();
  }

  for (;;) 
  {
    // TODO: Should occasionally timeout to check that no error has occured. 
    size_t num_received = xStreamBufferReceive(
      rxStreamBufferHandle, 
      rxBuffer + rxBufferPos, 
      RX_BUFFER_SIZE - rxBufferPos, 
      portMAX_DELAY);

    // Add new data to tx message queue, do not wait for space in queue
    xMessageBufferSend(txMessageBufferHandle, rxBuffer + rxBufferPos, num_received, 0);
    
    for (size_t i = 0; i < num_received; i++)
    {
      if (rxBuffer[i] != '\n')
      {
        currentMessage[messageIndex] = rxBuffer[i];
        messageIndex++;
      }
      else 
      {
        xMessageBufferSend(cliMessageBufferHandle, currentMessage, messageIndex, 0);
        messageIndex = 0;
      }
    }
  }
}

#define START_MESSAGE "Hello from Tx!\n"
void StartTxTask(void * args) 
{
  HAL_UART_Transmit_IT(&huart2, (uint8_t*) START_MESSAGE, sizeof START_MESSAGE);
  xSemaphoreTake(txCompleteSemaphore, portMAX_DELAY);

  for (;;) 
  {
    // Wait for a message to send
    size_t messageSize = xMessageBufferReceive(txMessageBufferHandle, txBuffer, TX_BUFFER_SIZE, portMAX_DELAY);

    HAL_UART_Transmit_IT(&huart2, txBuffer, messageSize);

    // Wait for notification that message has been sent
    xSemaphoreTake(txCompleteSemaphore, portMAX_DELAY);
  }
}

void StartCLITask(void * args) 
{
  for (;;) {
    xMessageBufferReceive(cliMessageBufferHandle, cliBuffer, CLI_BUFFER_SIZE, portMAX_DELAY);
    char* cliOutputBuffer = pvPortMalloc(CLI_BUFFER_SIZE);
    if (cliOutputBuffer == NULL) continue;  // If unable to allocate mem to heap for buffer, skip this message.

    // Parse command
    BaseType_t hasData;
    do {
      hasData = FreeRTOS_CLIProcessCommand(cliBuffer, cliOutputBuffer, CLI_BUFFER_SIZE);
      
      // Send response if there is a new line available
      if (hasData) xMessageBufferSend(txMessageBufferHandle, cliOutputBuffer, strlen(cliOutputBuffer), 0);
    } while (hasData != pdFALSE);

    // Cleanup memory
    vPortFree(cliOutputBuffer);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    // Delay a second
    osDelay(1000);
    // Flash LED2
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
