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
#include <time.h>

//AJ - Custom includes
#include "stm32f3xx_GPIO.hpp"
#include "stm32f3xx_UART.hpp"

//AJ - FreeRTOS header includes
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
TIM_HandleTypeDef htim1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
//xTaskHandle producer_ThreadHandle;
//xTaskHandle consumer_ThreadHandle;

xTaskHandle UART_TX_thread;
xTaskHandle UART_RX_thread;

xQueueHandle uart_tx_queue;
xQueueHandle uart_rx_queue;
//xSemaphoreHandle semaphore;
char buff[256]; // Global buffer to handle sprintf / HAL_UART_TRANSMIT calls for basic print functionality.
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
// AJ - my tasks.

/*-----------------------------------------------------------*/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Declaring this as a global for use in UART4_IRQHandler.
LabUART_InitStruct myUART_InitStruct; // will be empty for now - except for UART_port being detailed. Detailed in tasks.

// [2-27-22]: Lets add an extern C block to override our Interrupt Handler.
#ifdef __cplusplus
extern "C" {
#endif

// Override `UART4_IRQHandler` to trasmit our UART character when TXE=1.
void UART4_IRQHandler(void) {
	// Maybe for now we can attempt to print out a character via HAL_transmit? just to see if we enter this routine.
  	// It appears this routine is now functioning.
    char local_buff [50] = {0};

    // Before exiting this routine, we likely need to clear interrupt. Lets try the code below.

//    memset(local_buff, '0', 50);
//    sprintf((char*)local_buff, "IRQ! TXE: %d TXEIE: %d, RXNE: %d, RXNEIE: %d\r\n", myUART_InitStruct.UART_port->ISR & (1 << 7) >> 7,
//                                                                                   myUART_InitStruct.UART_port->CR1 & (1 << 7) >> 7,
//                                                                                   myUART_InitStruct.UART_port->ISR & (1 << 5) >> 5,
//                                                                                   myUART_InitStruct.UART_port->CR1 & (1 << 5) >> 5);
//    HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 50);

    // [3-1-22]: Lets try the following approach:
    // Check our source of ISR (Currently will be due to TXE = 1.)
    if ((myUART_InitStruct.UART_port->CR1 & (1 << 7) >> 7) == 1) { // If we detect TXEIE = 1, TXE = 1 was the source of our interrupt.

        // [3-12-22]: Interrupt Service Routine for Transmit Related Interrupts should be handled as mentioned by dad...

        // Disable TXEIE via clearing UART->terruCR1->TXEIE bit 7. Will let `UART_TX_nonblocking` take care of re-enabling interrupts as there is more data to send.
        myUART_InitStruct.UART_port->CR1 &= ~(1 << 7); // Clear interrupt for TXEIE  (current source)
        myUART_InitStruct.UART_port->CR1 &= ~(1 << 5); // Clear inpt for RXNEIE (other source, but not current source)

        uint16_t data;
        // Recieve data that was written onto Queue.
        xQueueReceiveFromISR(uart_tx_queue, (void*) &data, NULL);
        // place data in TDR.
        myUART_InitStruct.UART_port->TDR = data; // Once this is done, TXE will be 0 until data here is placed in shift register.

        memset(local_buff, '0', 50);
        sprintf((char*)local_buff, "TX IRQ! TXE: %d TXEIE: %d, data: %c\r\n", myUART_InitStruct.UART_port->ISR & (1 << 7) >> 7, 
                                                                              myUART_InitStruct.UART_port->CR1 & (1 << 7) >> 7, 
                                                                              data);        
        HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 50);
    }

    // [3-3-22]: Lets write an IRQ handler given that Rx interrupt is detected.
    if ((myUART_InitStruct.UART_port->CR1 & (1 << 5) >> 5) == 1) { // If we detect RXNEIE = 1, RXNE = 1 was the source of our interrupt.

        // Disable RXNEIE via clearing UART->CR1->RXNEIE bit 5. Will let `UART_RX_nonblocking` take care of re-enabling interrupts as there is more data to send.
        myUART_InitStruct.UART_port->CR1 &= ~(1 << 5); // Clear interrupt for RXNEIE (current source)
        myUART_InitStruct.UART_port->CR1 &= ~(1 << 7); // Clear interrupt for TXEIE  (other source, but not current source)

        // place data recieved into `char data`
        uint16_t data = myUART_InitStruct.UART_port->RDR;

        memset(local_buff, '0', 50);
        sprintf((char*)local_buff, "RX IRQ! RXNE: %d, RXNEIE: %d data: %c\r\n", myUART_InitStruct.UART_port->ISR & (1 << 5) >> 5,
                                                                                myUART_InitStruct.UART_port->CR1 & (1 << 5) >> 5,
                                                                                data);
        HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 50);

        // Send data that was recieved onto Queue.
        xQueueSendFromISR(uart_rx_queue, (void*) &data, NULL);
    }

    // [3-12-22]: IRQ Handler for PE interrupt detected (would be detected upon Rx detecting Parity Error & PE = 1.)
    if ((myUART_InitStruct.UART_port->CR1 & (1 << 8) >> 8) == 1) { // If we detect PEIE = 1, PE = 1 was the source of our interrupt.
        // Disable source of our interrupt:
        myUART_InitStruct.UART_port->CR1 &= ~(1 << 8); // Disable PEIE interrupts (current source)
        // May need to disable other interrupts too. Check if diagram on page 928 / Figure 347 helps to determine issues with interrupt source...

        // For now - lets just indicate that we detected PE=1 Parity Error interrupt. May need to reference HAL layer for steps to take next when this occurs.
        memset(local_buff, '0', 50);
        sprintf((char*)local_buff, "PE IRQ! PE: %d, PEIE: %d \r\n", myUART_InitStruct.UART_port->ISR & (1 << 8) >> 8,
                                                                    myUART_InitStruct.UART_port->CR1 & (1 << 8) >> 8 );
        HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 50);
    }
}


#ifdef __cplusplus
}
#endif

void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

void UART_GPIO_output_at_9600bps (LabGPIO_X* myGPIO, char* buff_UART) {

	uint8_t length = strlen(buff_UART);

    // Output a buffer at 9600bps, 1 character at a time in the buffer.
    // 1/9600 = 104 *10^-6. --> 104 us.
    // Need to figure out how to do delay properly.. lets get logic down first.
    
    // Set myGPIO to OUTPUT data.
    myGPIO->setAsOutput();

    // Set our GPIO to HI - this will be our idle state.
    myGPIO->setAs(1); delay_us(910);// 10 cycles of idle state.
        
    // Iterate through buffer length
    for (uint8_t index = 0; index < length; index++) {
        // Send a START Bit. Logic 0.

        myGPIO->setAs(0); delay_us(91); //delay should be 104us, but setting to 91 us due to some issue with delay..? could be GPIO toggle taking to long????
//        for (int i = 0; i <= 255; i++) buff[i] = 0;
//        sprintf((char*)buff, "START BIT SENT.\r\n");
//        HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);

    	  // One char at a time.
        char c = buff_UART[index];

        // Slice up each character into bits & send over UART.
    	  for (uint8_t j = 0; j < 8; j++) {
//            for (int i = 0; i <= 255; i++) buff[i] = 0;
//            sprintf((char*)buff, "current char: %c\r\n", c);
//            HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
            
            // Slice up each bit from our `char c` starting from LSB to MSB for ASCI characters. STM32 is Little Endian.
            uint8_t current_bit = (c >> j) & (0x01);
//            for (int i = 0; i <= 255; i++) buff[i] = 0;
//            sprintf((char*)buff, "%s = %d | %s = %d\r\n", "current_bit: ", current_bit, "Slot: ", j);
//            HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);

            myGPIO->setAs(current_bit); delay_us(91);
//            for (int i = 0; i <= 255; i++) buff[i] = 0;
//            sprintf((char*)buff, "myGPIO->getLevel(): %d\r\n", myGPIO->getLevel());
//            HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
        }
        // Current Character is complete. Send a STOP Bit.
        myGPIO->setAs(1); delay_us(182);
//        for (int i = 0; i <= 255; i++) buff[i] = 0;
//        sprintf((char*)buff, "STOP BIT SENT.\r\n");
//        HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
    }
    // Return the line to HI for next data to output.
    myGPIO->setAs(1);
}

void UART_GPIO_Thread(void *p) {

//    for (int i = 0; i <= 255; i++) buff[i] = 0;
//    sprintf((char*)buff, "%s %d, %s %d\r\n", "UART_GPIO_Thread start, xTaskGetSchedulerState():", xTaskGetSchedulerState(), "uxTaskGetStackHighWaterMark", uxTaskGetStackHighWaterMark(NULL));
//    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);

    //Need to init RCC->AHBENR |= (0x00020000) to enable the clock for GPIO Port A. Should be in driver.
    RCC->AHBENR |= (0x00020000);

    LabGPIO_InitStruct myUARTGPIO_InitStruct;     //This will be my struct for my UART GPIO.
    myUARTGPIO_InitStruct.GPIO_port = GPIOA;      //Taken from stm32f303xc.h --> ((GPIO_TypeDef *) GPIOA_BASE)
    myUARTGPIO_InitStruct.pin       = 0;          //Toggle PA0 pin as out UART output (simulated via GPIO)
    myUARTGPIO_InitStruct.pupd      = 0x00000000; //PUPDR0[1:0] set to `00` for no pull-up / np pull-down.
    myUARTGPIO_InitStruct.output    = 0x0000;     //OT0 - bit 0, keep as pushpull . This can be left as all 0's. 1 indicates open-drain. May test this.
    myUARTGPIO_InitStruct.speed     = 0x00000003; //OSPEEDR0[1:0] set to `11` for High Speed I/O for output.

    // Call LabGPIO_X Constructor.
    LabGPIO_X myUARTGPIO(&myUARTGPIO_InitStruct);

    // Call our function to output data in a character buffer. Lets do "hello world!\r\n"
    char* my_buffer = "hello world!\r\n";

    memset(buff, '0', 255);
    sprintf((char*)buff, "hello world!\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);

    UART_GPIO_output_at_9600bps(&myUARTGPIO, my_buffer);
}

void UART4_TX_Thread_Blocking(void *p) {

    // It appears we may need to initialize clocking for UART peripherals. Refer to RCC_CFGR3 for clock settings.
    RCC->CFGR3 &= ~(3 << 20); // Place 00 at UART4SW[1:0] - Bits 20-21. This will have use use PCLK as UART4 Clock Source.
                              // From examining *.ioc, I see that PCLK / SYSCLK are the same at 8MHz. Likely can be kept as is.

    RCC->APB1ENR |= (1 << 19); // Place 1 at UART4EN - Bit 19. Should enable UART4 CLK.

    //Note: It seems we do not need to configure anything in RCC_APBRSTR register. Seems this is only used for RESET purpose.
    myUART_InitStruct.UART_port = UART4;
    LabUART myUART4(&myUART_InitStruct);  // constructor will take care of configuring UART Tx on PC10 for now.

    char* my_buffer = "hello world! UART Tx driver!\r\n";
    for (int i = 0; i < strlen(my_buffer); i++) {
        myUART4.UART_TX_blocking(my_buffer[i]);
        vTaskResume(UART_RX_thread);
    }

    memset(buff, '0', 255);
    sprintf((char*)buff, "Done with transmission.\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
}

void UART4_TX_Thread_NonBlocking(void *p) {

    // It appears we may need to initialize clocking for UART peripherals. Refer to RCC_CFGR3 for clock settings.
    RCC->CFGR3 &= ~(3 << 20); // Place 00 at UART4SW[1:0] - Bits 20-21. This will have use use PCLK as UART4 Clock Source.
                              // From examining *.ioc, I see that PCLK / SYSCLK are the same at 8MHz. Likely can be kept as is.

    RCC->APB1ENR |= (1 << 19); // Place 1 at UART4EN - Bit 19. Should enable UART4 CLK.

    //Note: It seems we do not need to configure anything in RCC_APBRSTR register. Seems this is only used for RESET purpose.
    myUART_InitStruct.UART_port = UART4;
    LabUART myUART4(&myUART_InitStruct);  // constructor will take care of configuring UART Tx on PC10 for now.

    uart_tx_queue = xQueueCreate(10, sizeof(uint16_t));
    if (uart_tx_queue == NULL) {
    	memset(buff, '0', 255);
        sprintf((char*)buff, "TX Failed xQueueCreate.\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
    } else {
    	memset(buff, '0', 255);
        sprintf((char*)buff, "TX Success xQueueCreate.\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
    }

    char* my_buffer = "hello world! UART Tx driver!\r\n";
    for (int i = 0; i < strlen(my_buffer); i++) {
        myUART4.UART_TX_nonblocking(my_buffer[i], uart_tx_queue); // [3-12-22] This function should be replaced with the state machine approach for a single character.
        vTaskResume(UART_RX_thread);
    }

    memset(buff, '0', 255);
    sprintf((char*)buff, "Done with transmission.\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
}

void UART4_RX_Thread_Blocking(void *p) {

    // It appears we may need to initialize clocking for UART peripherals. Refer to RCC_CFGR3 for clock settings.
    RCC->CFGR3 &= ~(3 << 20); // Place 00 at UART4SW[1:0] - Bits 20-21. This will have use use PCLK as UART4 Clock Source.
                              // From examining *.ioc, I see that PCLK / SYSCLK are the same at 8MHz. Likely can be kept as is.

    RCC->APB1ENR |= (1 << 19); // Place 1 at UART4EN - Bit 19. Should enable UART4 CLK.

    //Note: It seems we do not need to configure anything in RCC_APBRSTR register. Seems this is only used for RESET purpose.
    myUART_InitStruct.UART_port = UART4;
    LabUART myUART4(&myUART_InitStruct);  // constructor will take care of configuring UART Rx on PC11 for now.

    uart_rx_queue = xQueueCreate(10, sizeof(uint8_t));
    if (uart_rx_queue == NULL) {
        memset(buff, '0', 255);
        sprintf((char*)buff, "RX Failed xQueueCreate.\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
    } else {
    	memset(buff, '0', 255);
        sprintf((char*)buff, "RX Success xQueueCreate.\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
    }

    // Needs to be a better way to do that as opposed to doing a while loop..? We will pretty much always be using CPU cycles for this.
    // Perhaps we should use a while loop & keep polling indefinitely? This still needs to be improved. Interrupt based is better..!

    char data = 0;

    while (1) {
    	vTaskSuspend(NULL);
        myUART4.UART_RX_blocking(uart_rx_queue);

        if (xQueuePeek(uart_rx_queue, &data, 1000)) {
            xQueueReceive(uart_rx_queue, &data, 1000);
            memset(buff, '0', 255);
            sprintf((char*)buff, "Reading: %c\r\n", data);
            HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
        }
    }
}

void UART4_RX_Thread_NonBlocking(void *p) {

    // It appears we may need to initialize clocking for UART peripherals. Refer to RCC_CFGR3 for clock settings.
    RCC->CFGR3 &= ~(3 << 20); // Place 00 at UART4SW[1:0] - Bits 20-21. This will have use use PCLK as UART4 Clock Source.
                              // From examining *.ioc, I see that PCLK / SYSCLK are the same at 8MHz. Likely can be kept as is.

    RCC->APB1ENR |= (1 << 19); // Place 1 at UART4EN - Bit 19. Should enable UART4 CLK.

    //Note: It seems we do not need to configure anything in RCC_APBRSTR register. Seems this is only used for RESET purpose.
    myUART_InitStruct.UART_port = UART4;
    LabUART myUART4(&myUART_InitStruct);  // constructor will take care of configuring UART Rx on PC11 for now.

    uart_rx_queue = xQueueCreate(10, sizeof(uint16_t));
    if (uart_rx_queue == NULL) {
        memset(buff, '0', 255);
        sprintf((char*)buff, "RX Failed xQueueCreate.\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
    } else {
    	memset(buff, '0', 255);
        sprintf((char*)buff, "RX Success xQueueCreate.\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 1000);
    }

    // Needs to be a better way to do that as opposed to doing a while loop..? We will pretty much always be using CPU cycles for this.
    // Perhaps we should use a while loop & keep polling indefinitely? This still needs to be improved. Interrupt based is better..!
    while (1) {
      	vTaskSuspend(NULL);
        myUART4.UART_RX_nonblocking(uart_rx_queue);
    }
}

void AJ_GPIO_Init() {
    // Moving some code here... some issues with Tx/Rx communication.... seems something is up with GPIO init?

    // Lets attempt to initialize our GPIO pin for PC10 as an output (Tx) & PC11 as an input (Rx).

    // First up is initializing the clock for GPIO C port via IOPCEN (I-O Port C Enable..?)
    RCC->AHBENR |= (1 << 19);

    // Initialize our GPIO C Port - Pin 10 as an alternate. For UART Tx.
    LabGPIO_InitStruct myGPIO_Tx_InitStruct;
    myGPIO_Tx_InitStruct.GPIO_port = GPIOC;       // Configure port C.
    myGPIO_Tx_InitStruct.pin       = 10;          // Configure pin 10.
    myGPIO_Tx_InitStruct.pupd      &= ~(3 << 20); // Set GPIOx_PUPDR   --> PUPDR10[1:0] to 00. No PullUp, No PullDown.
    myGPIO_Tx_InitStruct.output    &= ~(1 << 10); // Set GPIOx_OTYPER  --> OT10 to 0. PushPull.
    myGPIO_Tx_InitStruct.speed     |=  (3 << 20); // Set GPIOx_OSPEEDR --> OSPEEDR10[1:0] to 11. High Speed.

    LabGPIO_X myGPIO_Tx(&myGPIO_Tx_InitStruct);
    myGPIO_Tx.setAsAlternate(0x5); //Port C - Pin 10 to AF5 - UART4_TX, 0b0101 == 0x5.

    // Initialize our GPIO C Port - Pin 11 as an alternate. For UART Rx.
    LabGPIO_InitStruct myGPIO_Rx_InitStruct;
    myGPIO_Rx_InitStruct.GPIO_port = GPIOC;       // Configure port C.
    myGPIO_Rx_InitStruct.pin       = 11;          // Configure pin 11.
    myGPIO_Rx_InitStruct.pupd      &= ~(3 << 22); // Set GPIOx_PUPDR   --> PUPDR11[1:0] to 00. No PullUp, No PullDown.
    myGPIO_Rx_InitStruct.output    &= ~(1 << 11); // Set GPIOx_OTYPER  --> OT10 to 0. PushPull.
    myGPIO_Rx_InitStruct.speed     |=  (3 << 22); // Set GPIOx_OSPEEDR --> OSPEEDR10[1:0] to 11. High Speed.

    LabGPIO_X myGPIO_Rx(&myGPIO_Rx_InitStruct);
    myGPIO_Rx.setAsAlternate(0x5); //Port C - Pin 11 to AF5 - UART4_RX, 0b0101 == 0x5.
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
  /* USER CODE BEGIN 2 */

  // AJ - Initialize timer
  HAL_TIM_Base_Start(&htim1);

  // AJ - GPIO Init for UART.
  AJ_GPIO_Init();

  // AJ - Enable UART4_IRQn interrupt/set priority
  NVIC_SetPriority(UART4_IRQn, 5);
  NVIC_EnableIRQ(UART4_IRQn);

  /* USER CODE END 2 */

  /* Init scheduler */
//  osKernelInitialize();

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
  /* creation of defaultTask */
//  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

    /* UART via GPIO simulation task definition */
    //xTaskCreate(UART_GPIO_Thread, "UART_GPIO_Thread", 256, NULL, 1, NULL);

    /* UART4 TX task */
    xTaskCreate(UART4_TX_Thread_NonBlocking, "UART4_TX_Thread_NonBlocking", 256, NULL, 1, &UART_TX_thread);

    xTaskCreate(UART4_RX_Thread_NonBlocking, "UART4_RX_Thread_NonBlocking", 256, NULL, 2, &UART_RX_thread);
    /* Start Scheduler */
    uint8_t start_program[100] = "!!!Program Start!!!\r\n";
    HAL_UART_Transmit(&huart1, start_program, 21, 1000);

    vTaskStartScheduler();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
//  osKernelStart();

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
  huart1.Init.BaudRate = 9600;
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
void StartDefaultTask(void *argument)
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

