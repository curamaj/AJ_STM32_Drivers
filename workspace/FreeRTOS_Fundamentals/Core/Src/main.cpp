/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include <main.hpp>
// #include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//AJ - C includes
#include <stdio.h>
#include <stdint.h>
#include <string.h>

//AJ - Custom includes

//AJ - FreeRTOS header includes
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
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
UART_HandleTypeDef huart1;

//osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
// AJ - Task handles
xTaskHandle producer_ThreadHandle;
xTaskHandle consumer_ThreadHandle;

// AJ - Queue handles
//xQueueHandle queue;
//xQueueHandle queue_challenge_queue1;
//xQueueHandle queue_challenge_queue2;
//xQueueHandle semphr_challenge_queue;

// AJ - Semaphore handles
//xSemaphoreHandle binarySem;
//xSemaphoreHandle mutex;

// AJ - Semaphore Challenge variables
// Settings
const int buf_size = 5;       // Size of buffer
int num_prod_tasks = 5; // Number of producers
int num_cons_tasks = 2; // Number of consumers
int num_writes = 3;     // Number of writes producers will place in buffer.

// Globals
static int buf[buf_size];    // Shared buffer
static int head = 0;         // Writing index to buffer
static int tail = 0;         // Reading index to buffer
xSemaphoreHandle binarySem;  // Waits for parameter to be read
xSemaphoreHandle countingSemFilled;
xSemaphoreHandle countingSemEmpty;
xSemaphoreHandle mutex;
xSemaphoreHandle write_mutex; //for writes to UART.

char buff[256]; // Global buffer to handle sprintf / HAL_UART_TRANSMIT calls for basic print functionality.
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
// AJ - my tasks.

/*-----------------------------------------------------------*/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
void producer(void* p) {

	for (int i = 0; i <= 255; i++) buff[i] = 0;
    sprintf((char*)buff, "in Producer.\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);


	char* message = "Hello World! From producer.\r\n";
    for (int i = 0; i <= 255; i++) buff[i] = 0;
    sprintf((char*)buff, "%s\r\n",message);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);

    xQueueSend(queue, (void*)message, 5000);

    xSemaphoreTake(semaphore, 5000);
	for (int i = 0; i <= 255; i++) buff[i] = 0;
    sprintf((char*)buff, "(should be second!) Producer: after xQueueSend & Semaphore has been taken.\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);

}

void consumer(void* p) {

	xSemaphoreTake(semaphore, 5000);
    for (int i = 0; i <= 255; i++) buff[i] = 0;
    sprintf((char*)buff, "in Consumer.\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);

    char messagerecieved[30];
    xQueueReceive(queue, (void*)messagerecieved, 5000);

    for (int i = 0; i <= 255; i++) buff[i] = 0;
    sprintf((char*)buff, "(should be first!) Consumer: %s\r\n",messagerecieved);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
    xSemaphoreGive(semaphore);
    vTaskSuspend(consumer_ThreadHandle);
}

void QueueChallengeTaskA(void* p) {
	while(1) {
    uint32_t input;
    char message_from_TaskB_Queue2[64];
    input = 100;
    xQueueSend(queue_challenge_queue1, (void*)&input, 1000);
    xQueueReceive(queue_challenge_queue2, (void*)&message_from_TaskB_Queue2, 1000);
    for (int i = 0; i <= 255; i++) buff[i] = 0;
    sprintf((char*)buff, "QueueChallengeTaskA - message_from_TaskB_Queue2: %s\r\n",message_from_TaskB_Queue2);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
    vTaskDelay(5000);
	}
}

void QueueChallengeTaskB(void* p) {
	while(1) {
    uint32_t input_from_TaskA_Queue1;
    char* message = "hello world";
    xQueueReceive(queue_challenge_queue1, (void*)&input_from_TaskA_Queue1, 1000);
    xQueueSend(queue_challenge_queue2, (void*)message, 1000);
    for (int i = 0; i <= 255; i++) buff[i] = 0;
    sprintf((char*)buff, "QueueChallengeTaskB - input_from_TaskA: %d\r\n",input_from_TaskA_Queue1);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
    vTaskDelay(5000);
	}
}
*/

// Producer: write a given number of times to shared buffer
void SemphrChallengeProducer(void* p) {

	  xSemaphoreTake(write_mutex, portMAX_DELAY); //mutex to protect writing to UART.
      for (int i = 0; i <= 255; i++) buff[i] = 0;
      sprintf((char*)buff, "producer [%i]\r\n", *(int*)p);
      HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
      xSemaphoreGive(write_mutex);

//      xSemaphoreGive(binarySem); //mutex to protect copying parameters.

	  xSemaphoreTake(write_mutex, portMAX_DELAY); //mutex to protect writing to UART.
      for (int i = 0; i <= 255; i++) buff[i] = 0;
      sprintf((char*)buff, "binarySem Given\r\n");
      HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
      xSemaphoreGive(write_mutex);


	  // Copy the parameters into a local variable
	  int num = *(int *)p;

	  // Fill shared buffer with task number
	  for (int i = 0; i < num_writes; i++) {
	      // Critical section (accessing shared buffer)
          xSemaphoreTake(countingSemEmpty, 1000);   // -1 to countingSemEmpty (starts at 5) down to 0.

          xSemaphoreTake(mutex, 1000); //mutex to protect writing.
          buf[head] = num;
	      head = (head + 1) % buf_size;
          xSemaphoreGive(mutex);

          xSemaphoreGive(countingSemFilled);        // +1 to countingSemFilled (starts at 0) up to 5.
	  }

	  // Delete self task
	  vTaskDelete(NULL);
}

// Consumer: continuously read from shared buffer
void SemphrChallengeConsumer(void* p) {

	xSemaphoreTake(write_mutex, portMAX_DELAY); //mutex to protect writing to UART.
    for (int i = 0; i <= 255; i++) buff[i] = 0;
    sprintf((char*)buff, "consumer [%i]\r\n", *(int*)p);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
    xSemaphoreGive(write_mutex);

	  int val;

	  // Read from buffer
	  while (1) {
          // Critical section (accessing shared buffer and Serial)
          xSemaphoreTake(countingSemFilled, 1000);       // -1 to countingSemFilled from MAX of 5 down to 0.

          xSemaphoreTake(mutex, 1000); // mutex to protect buf access & printing.
          val = buf[tail];
	      tail = (tail + 1) % buf_size;

		  xSemaphoreTake(write_mutex, portMAX_DELAY); //mutex to protect writing to UART.
          for (int i = 0; i <= 255; i++) buff[i] = 0;
          sprintf((char*)buff, "%i\r\n", val);
          HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
    	  xSemaphoreGive(write_mutex);

          xSemaphoreGive(mutex);

          xSemaphoreGive(countingSemEmpty);              // +1 to countingSemEmpty from MIN of 0 to 5. Wait till transmit.

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
//  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // AJ - Initialize LEDs. I am doing this in my tasks.
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  /* definition and creation of defaultTask */
//  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
//  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    //uint8_t start_program_1[100] = "!!!Program Start!!!\r\n";
    //HAL_UART_Transmit(&huart1, start_program_1, 25, 1000);
    // /*** AJ - Simple Producer/Consumer Task setup ***/
    // /* Setup Queue */
    // queue = xQueueCreate(5, sizeof(char)*64);
    // semaphore = xSemaphoreCreateMutex();
    // /* producer Task */
    // xTaskCreate(producer, "producer", 128, NULL, 1, &producer_ThreadHandle);
    // /* consumer Task */
    // xTaskCreate(consumer, "consumer", 128, NULL, 2, &consumer_ThreadHandle);

    // /*** Queue Challenge Task Setup ***/
    // /* Queue setup */
    // challenge_queue1 = xQueueCreate(5, sizeof(uint32_t));
    // challenge_queue2 = xQueueCreate(5, sizeof(char)*64);
    // /* producer Task */
    // xTaskCreate(QueueChallengeTaskA, "QueueChallengeTaskA", 128, NULL, 2, &producer_ThreadHandle);
    // /* consumer Task */
    // xTaskCreate(QueueChallengeTaskB, "QueueChallengeTaskB", 128, NULL, 1, &consumer_ThreadHandle);

    /*** Semaphore Challenge Task Setup ***/
    /* Queue setup */

    /* Semaphore setup */
    countingSemFilled = xSemaphoreCreateCounting(5, 0); //filled slots - 5 is `buf_size`. start off with 0 filled slots.
    countingSemEmpty  = xSemaphoreCreateCounting(5, 5); //empty slot - 5 is `buf_size`. start off with 5 empty slots.
    mutex             = xSemaphoreCreateMutex();
    write_mutex       = xSemaphoreCreateMutex(); // KC - Write Mutex to prevent writes to UART.
    binarySem         = xSemaphoreCreateBinary();

    uint8_t start_program_1[100] = "!!!Semaphores Init'd!!!\r\n";
    HAL_UART_Transmit(&huart1, start_program_1, 25, 1000);

    /* producer Task */
    for (int j = 0; j < num_prod_tasks; j++) {

    	// Check Heap size available.
	    xSemaphoreTake(write_mutex, portMAX_DELAY); //mutex to protect writing to UART.
	    for (int i = 0; i <= 255; i++) buff[i] = 0;
	    sprintf((char*)buff, "xPortGetFreeHeapSize(): %d\r\n", xPortGetFreeHeapSize());
	    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
	    xSemaphoreGive(write_mutex);

    	xSemaphoreTake(write_mutex, portMAX_DELAY); //mutex to protect writing to UART.
        for (int i = 0; i <= 255; i++) buff[i] = 0;
        sprintf((char*)buff, "--Producer[%i]--\r\n", j);
        HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
        xSemaphoreGive(write_mutex);

        BaseType_t prod = xTaskCreate(SemphrChallengeProducer, "SemphrChallengeProducer", 128, (void*) &j, 1, NULL); //need to pass in correct info here.
        if (prod == pdFAIL) {
        	xSemaphoreTake(write_mutex, portMAX_DELAY); //mutex to protect writing to UART.
            for (int i = 0; i <= 255; i++) buff[i] = 0;
            sprintf((char*)buff, "Unable to create Producer[%i]\r\n", j);
            HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
            xSemaphoreGive(write_mutex);
        } else {
        	xSemaphoreTake(write_mutex, portMAX_DELAY); //mutex to protect writing to UART.
            for (int i = 0; i <= 255; i++) buff[i] = 0;
            sprintf((char*)buff, "Created Producer[%i]\r\n", j);
            HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
            xSemaphoreGive(write_mutex);
        }
//        xSemaphoreTake(binarySem, 1000);
    }
    /* consumer Task */
    for (int i = 0; i < num_cons_tasks; i++) {

    	// Check Heap size available.
	    xSemaphoreTake(write_mutex, portMAX_DELAY); //mutex to protect writing to UART.
	    for (int i = 0; i <= 255; i++) buff[i] = 0;
	    sprintf((char*)buff, "xPortGetFreeHeapSize(): %d\r\n", xPortGetFreeHeapSize());
	    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
	    xSemaphoreGive(write_mutex);

    	xSemaphoreTake(write_mutex, portMAX_DELAY); //mutex to protect writing to UART.
        for (int i = 0; i <= 255; i++) buff[i] = 0;
        sprintf((char*)buff, "--Consumer[%i]--\r\n", i);
        HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
        xSemaphoreGive(write_mutex);

        BaseType_t cons = xTaskCreate(SemphrChallengeConsumer, "SemphrChallengeConsumer", 128, (void*) &i, 1, NULL); //need to pass in correct info here.
        if (cons == pdFAIL) {
        	xSemaphoreTake(write_mutex, portMAX_DELAY); //mutex to protect writing to UART.
            for (int i = 0; i <= 255; i++) buff[i] = 0;
            sprintf((char*)buff, "Unable to create Consumer[%i]\r\n", i);
            HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
            xSemaphoreGive(write_mutex);
        } else {
        	xSemaphoreTake(write_mutex, portMAX_DELAY); //mutex to protect writing to UART.
            for (int i = 0; i <= 255; i++) buff[i] = 0;
            sprintf((char*)buff, "Created Consumer[%i]\r\n", i);
            HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
            xSemaphoreGive(write_mutex);
        }
    }

    /* Start Scheduler */
    uint8_t start_program_2[100] = "!!!Tasks Created!!!\r\n";
    HAL_UART_Transmit(&huart1, start_program_2, 21, 1000);
    vTaskStartScheduler();
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  //  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	    uint8_t start_program_4[100] = "!!!In While!!!\r\n";
	    HAL_UART_Transmit(&huart1, start_program_4, 21, 1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	for (int i = 0; i <= 255; i++) buff[i] = 0;
	sprintf((char*)buff, "%s %d\r\n", "StartDefaultTask(). xTaskGetSchedulerState(): ", xTaskGetSchedulerState());
	HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
    vTaskDelay(1000);

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
