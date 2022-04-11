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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//AJ - C includes
#include <stdio.h>
#include <stdint.h>
#include <string.h>

//AJ - Custom includes
#include "stm32f3xx_GPIO.hpp"
#include "stm32f3xx_Interrupt.hpp"
#include "stm32f3xx.h"

//AJ - FreeRTOS header includes
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// AJ - Custom declaration for 

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

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
xTaskHandle AJ_LED5_ThreadHandle;
xTaskHandle AJ_LED6_ThreadHandle;
xTaskHandle AJ_UserButton_ThreadHandle;


xQueueHandle queue;
xSemaphoreHandle semaphore;
char buff[256]; // Global buffer to handle sprintf / HAL_UART_TRANSMIT calls for basic print functionality.
int isr_detect_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
// AJ - my tasks.
void AJ_LED5_Thread1(void *p);
void StartDefaultTask(void const *argument);

/*-----------------------------------------------------------*/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// [2-20-22]: Lets add an extern C block to override our Interrupt Handler.
#ifdef __cplusplus
extern "C" {
#endif

// EXTI_TypeDef* myEXTI = EXTI;
// Override `EXTI0_IRQHandler` for user button PA0 - method we fill in below will be our ISR handler.
// Need to figure out how to allow execution of a custom call back function rather than doing this. a bit hacky..?
void EXTI0_IRQHandler(void) {
    //Do what we want to do. could be giving a binary sem or incrementing a counter!
    EXTI_TypeDef* myEXTI = EXTI;
    isr_detect_counter++;
    // If we find that PR bit is set, an Interrupt has been detected.
    if ( ( (myEXTI->PR & (1 << 0)) >> 0 ) == 1) {
        //Clear interrupt! Lets clear it for Line 0.
        myEXTI->PR |= (1 << 0); // set 1 to pending register to clear pending bit when interrupt arrives. PR -> Bit 0 for PR0.
    }
}

#ifdef __cplusplus
}
#endif

/**
  * @brief  Toggle LED5 thread
  * @param  argument not used
  * @retval None
  */
void AJ_LED5_Thread1(void *p) {
/* AJ - LED & Button list onboard STM32F303VC Discovery Board:
    LEDs: Configure as output. I am not sure - but the PORT may refer to `E` & PIN may refer to the number.. Like Port `PE`, Pin `11`..
    LD3  -  PE9
    LD4  -  PE8  (0x0100)
    LD5  -  PE10
    LD6  -  PE15 (0x8000)
    LD7  -  PE11
    LD8  -  PE14
    LD9  -  PE12
    LD10 -  PE13
  
    Button(s): Configure as input.
    B1   -  PA0 (User button)
    B2   -  RESET (Likely reserved for reset)
*/

    for (int i = 0; i <= 255; i++) buff[i] = 0;
    sprintf((char*)buff, "%s %d, %s %d\r\n", "AJ_LED5_Thread1 start, xTaskGetSchedulerState():", xTaskGetSchedulerState(), "uxTaskGetStackHighWaterMark", uxTaskGetStackHighWaterMark(NULL));
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);

    //Need to init RCC->AHBENR |= (0x00200000) to enable the clock for GPIO Port E. Should be in driver.
    RCC->AHBENR |= (0x00200000);

    LabGPIO_InitStruct myLED_InitStruct;     //This will be my struct for LED5 - PE10.
    myLED_InitStruct.GPIO_port = GPIOE;      //Taken from stm32f303xc.h --> ((GPIO_TypeDef *) GPIOE_BASE)
    myLED_InitStruct.pin       = 10;         //Toggle PE10 - LD5.
    myLED_InitStruct.pupd      = 0x00000000; //PUPDR10[1:0] set to `01` for Pull Up resistor. setting to all 0's for now though.
    myLED_InitStruct.output    = 0x0000;     //OT10 - bit 10, keep as pushpull . This can be left as all 0's. 1 indicates open-drain. May test this.
    myLED_InitStruct.speed     = 0x00300000; //OSPEEDR10[1:0] set to `11` for High Speed I/O for output.
    
    // Call LabGPIO_X Constructor.
    LabGPIO_X myLED5(&myLED_InitStruct);

    // set myLED5 as a GPIO Output.
    myLED5.setAsOutput(); //sets up myLED5 as a GPIO pin that will output `0`/`1`.
    for (int i = 0; i <= 255; i++) buff[i] = 0;
    sprintf((char*)buff, "%s = %d\r\n", "myLED5.getDirection()", myLED5.getDirection());
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);

    for (;;) {
        myLED5.setHigh();
        for (int i = 0; i <= 255; i++) buff[i] = 0;
        sprintf((char*)buff, "%s = %d\r\n", "myLED5.getLevel()", myLED5.getLevel());
        HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
        vTaskDelay(5000);

        myLED5.setLow();
        for (int i = 0; i <= 255; i++) buff[i] = 0;
        sprintf((char*)buff, "%s = %d\r\n", "myLED5.getLevel()", myLED5.getLevel());
        HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
        vTaskDelay(5000);
    }
}

/**
  * @brief  Toggle LED6 thread
  * @param  argument not used
  * @retval None
  */
void AJ_LED6_Thread2(void *p) {
/* AJ - LED & Button list onboard STM32F303VC Discovery Board:
    LEDs: Configure as output. I am not sure - but the PORT may refer to `E` & PIN may refer to the number.. Like Port `PE`, Pin `11`..
    LD3  -  PE9
    LD4  -  PE8  (0x0100)
    LD5  -  PE10
    LD6  -  PE15 (0x8000)
    LD7  -  PE11
    LD8  -  PE14
    LD9  -  PE12
    LD10 -  PE13

    Button(s): Configure as input.
    B1   -  PA0 (User button)
    B2   -  RESET (Likely reserved for reset)
*/

    for (int i = 0; i <= 255; i++) buff[i] = 0;
    sprintf((char*)buff, "%s %d, %s %d\r\n", "AJ_LED6_Thread2 start, xTaskGetSchedulerState():", xTaskGetSchedulerState(), "uxTaskGetStackHighWaterMark", uxTaskGetStackHighWaterMark(NULL));
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);

    //Need to init RCC->AHBENR |= (0x00200000) to enable the clock for GPIO Port E. Should be in driver.
    RCC->AHBENR |= (0x00200000);

    LabGPIO_InitStruct myLED_InitStruct;     //This will be my struct for LED5 - PE10.
    myLED_InitStruct.GPIO_port = GPIOE;      //Taken from stm32f303xc.h --> ((GPIO_TypeDef *) GPIOE_BASE)
    myLED_InitStruct.pin       = 15;         //Toggle PE15 - LD6.
    myLED_InitStruct.pupd      = 0x00000000; //PUPDR15[1:0] set to `01` for Pull Up resistor. setting to all 0's for now though.
    myLED_InitStruct.output    = 0x0000;     //OT10 - bit 10, keep as pushpull . This can be left as all 0's. 1 indicates open-drain. May test this.
    myLED_InitStruct.speed     = 0xC0000000; //OSPEEDR15[1:0] set to `11` for High Speed I/O for output.

    // Call LabGPIO_X Constructor.
    LabGPIO_X myLED6(&myLED_InitStruct);

    // set myLED as a GPIO Output.
    myLED6.setAsOutput(); //sets up myLED as a GPIO pin that will output `0`/`1`.
    for (int i = 0; i <= 255; i++) buff[i] = 0;
    sprintf((char*)buff, "%s = %d\r\n", "myLED6.getDirection()", myLED6.getDirection());
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);

    for (;;) {
        myLED6.setHigh();
        for (int i = 0; i <= 255; i++) buff[i] = 0;
        sprintf((char*)buff, "%s = %d\r\n", "myLED6.getLevel()", myLED6.getLevel());
        HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
        vTaskDelay(5000);

        myLED6.setLow();
        for (int i = 0; i <= 255; i++) buff[i] = 0;
        sprintf((char*)buff, "%s = %d\r\n", "myLED6.getLevel()", myLED6.getLevel());
        HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
        vTaskDelay(5000);
    }
}

/**
  * @brief  Configure User Button thread
  * @param  argument not used
  * @retval None
  */
void AJ_UserButton_Thread3(void *p) {
/* AJ - LED & Button list onboard STM32F303VC Discovery Board:
    LEDs: Configure as output. I am not sure - but the PORT may refer to `E` & PIN may refer to the number.. Like Port `PE`, Pin `11`..
    LD3  -  PE9
    LD4  -  PE8  (0x0100)
    LD5  -  PE10
    LD6  -  PE15 (0x8000)
    LD7  -  PE11
    LD8  -  PE14
    LD9  -  PE12
    LD10 -  PE13

    Button(s): Configure as input.
    B1   -  PA0 (User button)
    B2   -  RESET (Likely reserved for reset)
*/

    for (int i = 0; i <= 255; i++) buff[i] = 0;
    sprintf((char*)buff, "%s %d, %s %d\r\n", "AJ_UserButton_Thread3 start, xTaskGetSchedulerState():", xTaskGetSchedulerState(), "uxTaskGetStackHighWaterMark", uxTaskGetStackHighWaterMark(NULL));
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);

    //Need to init RCC->AHBENR |= (0x0020000) to enable the clock for GPIO Port A. Should be in driver.
    RCC->AHBENR |= (0x00020000);

    LabGPIO_InitStruct myButton_InitStruct;     //This will be my struct for LED5 - PE10.
    myButton_InitStruct.GPIO_port = GPIOA;      //Taken from stm32f303xc.h --> ((GPIO_TypeDef *) GPIOA_BASE)
    myButton_InitStruct.pin       = 0;          //Toggle PA0 - User Button on STM32F303VC Discovery board.
    myButton_InitStruct.pupd      = 0x00000002; //PUPDR0[1:0] set to `10` for Pull Down resistor. After we stop pressing the button, we want it to go LO, not HI!
    myButton_InitStruct.output    = 0x0000;     //OT10 - bit 10, keep as pushpull . This can be left as all 0's. 1 indicates open-drain. May test this.
    myButton_InitStruct.speed     = 0x00000003; //OSPEEDR10[1:0] set to `11` for High Speed I/O for output.

    // Call LabGPIO_X Constructor.
    LabGPIO_X myButton(&myButton_InitStruct);

    // set myButton as a GPIO Input.
    myButton.setAsInput(); //sets up myButton as a GPIO pin that will input `0`/`1`.
    for (int i = 0; i <= 255; i++) buff[i] = 0;
    sprintf((char*)buff, "%s = %d\r\n", "myButton.getDirection()", myButton.getDirection());
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);

    for (;;) {
        for (int i = 0; i <= 255; i++) buff[i] = 0;
        sprintf((char*)buff, "buttonpressed: %d, isr_detect_counter: %d\r\n", myButton.getLevel(), isr_detect_counter);
        HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
//        vTaskDelay(5000);
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
  // AJ - Enable interrupts.


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  // MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // AJ - Initialize LEDs. I am doing this in my tasks.
  // AJ - Initialize usage of interrupts. Enable for User Button.
  

   // Enable SYSCFG Clock as we need to access a SYSCFG register..?
   RCC->APB2ENR |= (1 << 0); // Bit 0 - SYSCFGEN System Configuration Clock Enable.
   // Configure SYSCFG External Interrupt Configuration Register to allow PA[0] to use EXTI
   SYSCFG->EXTICR[0] = 0x000; // PA[0] pin.

   // Enable interrupts from stm32f3xx_Interrupt
   enableInterrupts(0); // Enable for line 0 --> PortX Pin0, for us it is PortA, Pin0

   // Set EXTI0_IRQn priority
   NVIC_SetPriority(EXTI0_IRQn, 5);

   NVIC_EnableIRQ(EXTI0_IRQn);

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
  // osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  // defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

    /* LED5 Thread 1 definition */
    // xTaskCreate(AJ_LED5_Thread1, "AJ_LED5_Thread1", 128, NULL, 1, &AJ_LED5_ThreadHandle);

    /* LED6 Thread 2 definition */
    // xTaskCreate(AJ_LED6_Thread2, "AJ_LED6_Thread2", 128, NULL, 2, &AJ_LED6_ThreadHandle);

    /* User Button Thread 3 definition */
    xTaskCreate(AJ_UserButton_Thread3, "AJ_UserButton_Thread3", 128, NULL, 3, &AJ_UserButton_ThreadHandle);

    /* Start Scheduler */
    uint8_t start_program[100] = "!!!Program Start!!!\r\n";
    HAL_UART_Transmit(&huart1, start_program, 21, 1000);
    vTaskStartScheduler();
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  // osKernelStart();

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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();

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

