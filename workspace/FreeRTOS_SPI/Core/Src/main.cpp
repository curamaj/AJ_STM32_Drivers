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
#include "stm32f3xx_SPI.hpp"
#include "l3gd20.h"

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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
//osThreadId_t defaultTaskHandle;
//const osThreadAttr_t defaultTask_attributes = {
//  .name = "defaultTask",
//  .stack_size = 128 * 4,
//  .priority = (osPriority_t) osPriorityNormal,
//};
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
// AJ - my tasks.

/*-----------------------------------------------------------*/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

xTaskHandle SPI_TX_thread;

xSemaphoreHandle spi_gatekeeper; // Want to ensure only 1 task uses SPI bus at a time. So we will guard access of it.
xSemaphoreHandle spi_interrupt_binary_sem; // For use in our interrupt routine, flag will be raised when Interrupt is detected.
char buff[256]; // Global buffer to handle sprintf / HAL_UART_TRANSMIT calls for basic print functionality.
char local_buff[50];

// Declaring this as a global for use in SPI1_IRQHandler...
LabSPI_InitStruct mySPI1;

// Declaring global for SPI Singal.. This will need to be extern'd to our stm32f3xx_SPI.hpp/.cpp file...
spi_signal spi_current_signal = NO_SIGNAL;


// [2-27-22]: Lets add an extern C block to override our Interrupt Handler.
#ifdef __cplusplus
extern "C" {
#endif
// Override `SPI1_IRQHandler`.. Likely will need to override this.

void SPI1_IRQHandler(void) {

    // Before exiting this routine, we likely need to clear interrupt. Lets try the code below.
    char local_buff[100];

//    memset(local_buff, '0', 100);
//    sprintf((char*)local_buff, "IRQ! mySPI1.SPI_port->CR2:   ");
//    HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 100);
//
//    for (int i = 31 ; i >= 0 ; i--) {
//    	  memset(local_buff, '0', 100);
//        sprintf((char*)local_buff, "%i", ( ( mySPI1.SPI_port->CR2 & (1 << i) ) >> i) );
//        HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 100);
//    }
//
//    memset(local_buff, '0', 100);
//    sprintf((char*)local_buff, "\r\n");
//    HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 100);

    memset(local_buff, '0', 100);
    sprintf((char*)local_buff, "IRQ! TXE: %d TXEIE: %d, RXNE: %d, RXNEIE: %d\r\n", (mySPI1.SPI_port->SR  & (1 << 1)) >> 1,
                                                                                   (mySPI1.SPI_port->CR2 & (1 << 7)) >> 7,
                                                                                   (mySPI1.SPI_port->SR  & (1 << 0)) >> 0,
                                                                                   (mySPI1.SPI_port->CR2 & (1 << 6)) >> 6 );
    HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 100);

    // Lets check if source of interrupt was TXEIE...
    if ( (mySPI1.SPI_port->CR2 & (1 << 7)) >> 7 == 1) { // This will fire when SR - TXE = 1 for TxBuf Empty.
        // Lets disable the source of our current possible interrupts...

        mySPI1.SPI_port->CR2 &= ~(1 << 7); // Clear interrupt for TXEIE.
//        mySPI1.SPI_port->CR2 &= ~(1 << 6); // Clear interrupt for RXNEIE.

        memset(local_buff, '0', 100);
        sprintf((char*)local_buff, "IRQ! TXE: %d TXEIE: %d\r\n", (mySPI1.SPI_port->SR  & (1 << 1)) >> 1,
                                                                 (mySPI1.SPI_port->CR2 & (1 << 7)) >> 7 );
        HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 100);
        
        spi_current_signal = TXE_SIGNAL;
        xSemaphoreGiveFromISR(spi_interrupt_binary_sem, NULL);
    }

    // Lets check if source of interrupt was RXNEIE...
    if ( (mySPI1.SPI_port->CR2 & (1 << 6)) >> 6 == 1) { // This will fire when SR - RXNE = 1 for RxBuf Not Empty.
        // Lets disable the source of our current possible interrupts...

//        mySPI1.SPI_port->CR2 &= ~(1 << 7); // Clear interrupt for TXEIE.
        mySPI1.SPI_port->CR2 &= ~(1 << 6); // Clear interrupt for RXNEIE.

        memset(local_buff, '0', 100);
        sprintf((char*)local_buff, "IRQ! RXE: %d RXNEIE: %d\r\n", (mySPI1.SPI_port->SR  & (1 << 0)) >> 0,
                                                                  (mySPI1.SPI_port->CR2 & (1 << 6)) >> 6 );
        HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 100);
        
        spi_current_signal = RXNE_SIGNAL;
        xSemaphoreGiveFromISR(spi_interrupt_binary_sem, NULL);
    }
}

#ifdef __cplusplus
}
#endif


// [3-14-22]: Lets add typedef's for the data within registers we want to access in out L3GD20 chip if needed. 
// Placing sample bitfield below..
// typedef union {
//     uint8_t flashstatregbyte1;
//     struct {
//         uint8_t pagesize :1;
//         uint8_t protect :1;
//         uint8_t density :4;
//         uint8_t comp :1;
//         uint8_t rdy :1;
//     }__attribute__((packed));
// } flashstatregbyte1;

// [3-20-22]: Perhaps a bitfield mapping laid out like this would let us extract all our angular velocity data a bit easier?
// Realistically, bitfields are better suited for individual bits in a register. 
// So maybe you can practice creating a bitfield map for a certain REG we read from here.
typedef union {
    uint64_t out_data;
    struct {
        uint8_t out_x_l:   8;
        uint8_t out_x_h:   8;
        uint8_t out_y_l:   8;
        uint8_t out_y_h:   8;
        uint8_t out_z_l:   8;
        uint8_t out_z_h:   8;
        uint16_t padding: 16;
    } __attribute__((packed));
} out_data;

void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

void AJ_GPIO_Init() {
    // Moving some code here - Cleaner... Seems something is up with GPIO desconstructor..
    // Lets initialize our GPIO pins for SPI1 functionality. Just Slave Select here...
    // First up is initializing the clock for GPIO E port via IOPEEN (I-O Port E Enable..?)
    RCC->AHBENR |= (1 << 21);

    // AJ - It appears we actually may want to set this up a bit differently, lets set it up as a GPIO Output we can drive Hi/Lo,
    // Initialize our GPIO E Port - Pin 3 as an alternate. For SPI1_SS1 - GPIO Output - Slave Select 1.
    LabGPIO_InitStruct myGPIO_SPI1_SS1_InitStruct;
    myGPIO_SPI1_SS1_InitStruct.GPIO_port = GPIOE;       // Configure port E.
    myGPIO_SPI1_SS1_InitStruct.pin       = 3;           // Configure pin 3.
    myGPIO_SPI1_SS1_InitStruct.pupd      &= ~(3 << 6);  // Set GPIOx_PUPDR
    myGPIO_SPI1_SS1_InitStruct.output    &= ~(1 << 3);  // Set GPIOx_OTYPER
    myGPIO_SPI1_SS1_InitStruct.speed     |=  (3 << 6);  // Set GPIOx_OSPEEDR
    LabGPIO_X myGPIO_SPI1_SS1(&myGPIO_SPI1_SS1_InitStruct);
    myGPIO_SPI1_SS1.setAsOutput();
    myGPIO_SPI1_SS1.setLow(); // Lets set our GPIO connected to SS line on slave device to low to indicate we are communicating with it.
    delay_us(100);
    myGPIO_SPI1_SS1.setHigh(); // Lets set our GPIO connected to SS line on slave device to low to indicate we are communicating with it.
    delay_us(100);
    myGPIO_SPI1_SS1.setLow(); // Lets set our GPIO connected to SS line on slave device to low to indicate we are communicating with it.

    // [3-18-22] It appears it is important to toggle on/off our slave device, else it latches previous data & if in error scenario it may not communicate as expected?
}


void AJ_SPI_Init() {
    // Moving some code here - Cleaner... Seems something is up with GPIO desconstructor..
    // Lets initialize our GPIO pins as ALTERNATE for SPI1 functionality.
    // First up is initializing the clock for GPIO A port via IOPCEN (I-O Port A Enable..?)
    RCC->AHBENR |= (1 << 17);

    // Initialize our GPIO A Port - Pin 5 as an alternate. For SPI1_SCK - Slave Serial Clock
    LabGPIO_InitStruct myGPIO_SPI1_SCK_InitStruct;
    myGPIO_SPI1_SCK_InitStruct.GPIO_port = GPIOA;       // Configure port A.
    myGPIO_SPI1_SCK_InitStruct.pin       = 5;           // Configure pin 5.
    myGPIO_SPI1_SCK_InitStruct.pupd      &= ~(3 << 10); // Set GPIOx_PUPDR
    myGPIO_SPI1_SCK_InitStruct.output    &= ~(1 << 5);  // Set GPIOx_OTYPER
    myGPIO_SPI1_SCK_InitStruct.speed     |=  (3 << 10); // Set GPIOx_OSPEEDR
    LabGPIO_X myGPIO_SPI1_SCK(&myGPIO_SPI1_SCK_InitStruct);
    myGPIO_SPI1_SCK.setAsAlternate(0x5); //Port A - Pin 5 to AF5 - SPI1_SCK, 0b0101 == 0x5.

//    delay_us(500);

    // Initialize our GPIO A Port - Pin 6 as an alternate. For SPI1_MISO - Master In Slave Out 
    LabGPIO_InitStruct myGPIO_SPI1_MISO_InitStruct;
    myGPIO_SPI1_MISO_InitStruct.GPIO_port = GPIOA;       // Configure port A.
    myGPIO_SPI1_MISO_InitStruct.pin       = 6;           // Configure pin 6.
    myGPIO_SPI1_MISO_InitStruct.pupd      &= ~(3 << 12); // Set GPIOx_PUPDR
    myGPIO_SPI1_MISO_InitStruct.output    &= ~(1 << 6);  // Set GPIOx_OTYPER
    myGPIO_SPI1_MISO_InitStruct.speed     |=  (3 << 12); // Set GPIOx_OSPEEDR
    LabGPIO_X myGPIO_SPI1_MISO(&myGPIO_SPI1_MISO_InitStruct);
    myGPIO_SPI1_MISO.setAsAlternate(0x5); //Port A - Pin 6 to AF5 - SPI1_MISO, 0b0101 == 0x5.

//    delay_us(500);

    // Initialize our GPIO A Port - Pin 7 as an alternate. For SPI1_MOSI - Master Out Slave In
    LabGPIO_InitStruct myGPIO_SPI1_MOSI_InitStruct;
    myGPIO_SPI1_MOSI_InitStruct.GPIO_port = GPIOA;       // Configure port A.
    myGPIO_SPI1_MOSI_InitStruct.pin       = 7;           // Configure pin 7.
    myGPIO_SPI1_MOSI_InitStruct.pupd      &= ~(3 << 14); // Set GPIOx_PUPDR
    myGPIO_SPI1_MOSI_InitStruct.output    &= ~(1 << 7);  // Set GPIOx_OTYPER
    myGPIO_SPI1_MOSI_InitStruct.speed     |=  (3 << 14); // Set GPIOx_OSPEEDR
    LabGPIO_X myGPIO_SPI1_MOSI(&myGPIO_SPI1_MOSI_InitStruct);
    myGPIO_SPI1_MOSI.setAsAlternate(0x5); //Port A - Pin 7 to AF5 - SPI1_NSS, 0b0101 == 0x5.
}

void SPI1_TX_Thread_NonBlocking(void* p) {
    
    // Call our LabSPI Constructor. Need to setup SPI peripheral before GPIO_Init.
    mySPI1.SPI_port = SPI1;
    LabSPI myLabSPI1(&mySPI1);

	  // Call our SPI Init, then GPIO Init routine after Constructor.
    // We set our GPIO connected to SS for L3GD20 chip to low in this init routine.
	  AJ_SPI_Init();  // Init's SPI signals as Master... Presumably...
//  AJ_GPIO_Init(); // Slave Select GPIO goes low....


    RCC->AHBENR |= (1 << 21);

    // AJ - It appears we actually may want to set this up a bit differently, lets set it up as a GPIO Output we can drive Hi/Lo,
    // Initialize our GPIO E Port - Pin 3 as an alternate. For SPI1_SS1 - GPIO Output - Slave Select 1.
    LabGPIO_InitStruct myGPIO_SPI1_SS1_InitStruct;
    myGPIO_SPI1_SS1_InitStruct.GPIO_port = GPIOE;       // Configure port E.
    myGPIO_SPI1_SS1_InitStruct.pin       = 3;           // Configure pin 3.
    myGPIO_SPI1_SS1_InitStruct.pupd      &= ~(3 << 6);  // Set GPIOx_PUPDR
    myGPIO_SPI1_SS1_InitStruct.output    &= ~(1 << 3);  // Set GPIOx_OTYPER
    myGPIO_SPI1_SS1_InitStruct.speed     |=  (3 << 6);  // Set GPIOx_OSPEEDR
    LabGPIO_X myGPIO_SPI1_SS1(&myGPIO_SPI1_SS1_InitStruct);
    myGPIO_SPI1_SS1.setAsOutput();
    myGPIO_SPI1_SS1.setLow(); // Lets set our GPIO connected to SS line on slave device to low to indicate we are communicating with it.
    delay_us(100);
    myGPIO_SPI1_SS1.setHigh(); // Lets set our GPIO connected to SS line on slave device to low to indicate we are communicating with it.
    delay_us(100);
    myGPIO_SPI1_SS1.setLow(); // Lets set our GPIO connected to SS line on slave device to low to indicate we are communicating with it.




    // At this point I want to read the L3GD20 chip's data sheet to determine some basic commands to send it...
    // We can start with a basic WHO AM I message..?

    // Let us take out spi_gatekeeper mutex. Will need to return this after task is done. No other task should pre-empt us...
    xSemaphoreTake(spi_gatekeeper, 1000);

    uint8_t data_tx = 0x0F | 0x80; // WHO_AM_I address + Read Bit toggled.

    memset(buff, '0', 256);
    sprintf((char*)buff, "SPI_transfer: 0x%02x\r\n", data_tx);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 256);

    // Lets attempt transfer!
    // uint8_t data = myLabSPI1.SPI_transfer(data_tx);// WHO_AM_I: 0b1000_1111
    data_tx = myLabSPI1.SPI_TxRx_write(data_tx);
    uint8_t data_rx = myLabSPI1.SPI_TxRx_read();

    memset(buff, '0', 256);
    sprintf((char*)buff, "write rx: 0x%02x, read_rx: 0x%02x\r\n", data_tx, data_rx);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 256);

    myGPIO_SPI1_SS1.setHigh(); // Lets set our GPIO connected to SS line on slave device to low to indicate we are communicating with it.
    delay_us(100);
    myGPIO_SPI1_SS1.setLow(); // Lets set our GPIO connected to SS line on slave device to low to indicate we are communicating with it.

    uint8_t data_tx_2 = 0x20 | 0x80; // CTRL_REG1 address + Read Bit toggled.

    memset(buff, '0', 256);
    sprintf((char*)buff, "SPI_transfer: 0x%02x\r\n", data_tx_2);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 256);

    // Lets attempt transfer!
    // uint8_t data_2 = myLabSPI1.SPI_transfer(data_tx_2);// WHO_AM_I: 0b1000_1111
    data_tx_2 = myLabSPI1.SPI_TxRx_write(data_tx_2);
    uint8_t data_rx_2 = myLabSPI1.SPI_TxRx_read();

    memset(buff, '0', 256);
    sprintf((char*)buff, "write rx: 0x%02x, read_rx: 0x%02x\r\n", data_tx_2, data_rx_2);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 256);

    myGPIO_SPI1_SS1.setHigh(); // Lets set our GPIO connected to SS line on slave device to low to indicate we are communicating with it.
    delay_us(100);
    myGPIO_SPI1_SS1.setLow(); // Lets set our GPIO connected to SS line on slave device to low to indicate we are communicating with it.

    char* data_string = "Hello World/r/n";
    uint8_t data_string_rx = myLabSPI1.SPI_TxRx_writeln((uint8_t*)data_string, strlen(data_string));

    memset(buff, '0', 256);
    sprintf((char*)buff, "Printed on SPI: %s | data_string_rx: %d\r\n", data_string, data_string_rx);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 256);

    // After routine is finished, give up the Mutex we took..
    xSemaphoreGive(spi_gatekeeper);

    while(1); // make sure task doesnt end...
}

void SPI1_Init_L3GD20(void* p) {
    
    // Call our LabSPI Constructor. Need to setup SPI peripheral before GPIO_Init.
    mySPI1.SPI_port = SPI1;
    LabSPI myLabSPI1(&mySPI1);

	  // Call our SPI Init, then GPIO Init routine after Constructor.
    // We set our GPIO connected to SS for L3GD20 chip to low in this init routine.
	  AJ_SPI_Init();  // Init's SPI signals as Master... Presumably...

    RCC->AHBENR |= (1 << 21); // Enable Advanced High Speed Bus for GPIOE port.

    // AJ - It appears we actually may want to set this up a bit differently, lets set it up as a GPIO Output we can drive Hi/Lo,
    // Initialize our GPIO E Port - Pin 3 as an alternate. For SPI1_SS1 - GPIO Output - Slave Select 1.
    LabGPIO_InitStruct myGPIO_SPI1_SS1_InitStruct;
    myGPIO_SPI1_SS1_InitStruct.GPIO_port = GPIOE;       // Configure port E.
    myGPIO_SPI1_SS1_InitStruct.pin       = 3;           // Configure pin 3.
    myGPIO_SPI1_SS1_InitStruct.pupd      &= ~(3 << 6);  // Set GPIOx_PUPDR to No pull-up / No pull-down
    myGPIO_SPI1_SS1_InitStruct.output    &= ~(1 << 3);  // Set GPIOx_OTYPER to Output push-pull (reset state)
    myGPIO_SPI1_SS1_InitStruct.speed     |=  (3 << 6);  // Set GPIOx_OSPEEDR to High Speed
    LabGPIO_X myGPIO_SPI1_SS1(&myGPIO_SPI1_SS1_InitStruct);
    myGPIO_SPI1_SS1.setAsOutput();

    // At this point I want to read the L3GD20 chip's data sheet to determine some basic commands to send it...
    // We can start with a basic WHO AM I message..?

    // Let us take out spi_gatekeeper mutex. Will need to return this after task is done. No other task should pre-empt us...
    xSemaphoreTake(spi_gatekeeper, 1000);

    uint8_t data_tx = 0xFF;
    uint8_t data_rx = 0xFF;

    // Send out a WHO_AM_I message & read back data.
    data_tx = 0x0F | 0x80; // WHO_AM_I address + Read Bit toggled.
    myGPIO_SPI1_SS1.setLow();
    myLabSPI1.SPI_TxRx_write(data_tx);
    myLabSPI1.SPI_TxRx_read();
    myGPIO_SPI1_SS1.setHigh();

    /* Goal is to use SPI to configure LG3D20 to the following if possible.
       - Configure Temp Sensor - Read Data.
       - Use CTRL_REG1 for output data rate / bandwidth selection. Say 380Hz / Cut-Off as 20.
       - Use CTRL_REG4 for full scale selection. Say 500 dps.
       - There appears to be only 1 resolution mode so cant configure this...?
    */
 
    /* Some notes...
       - Lets attempt to configure XYZ FIFO to be in bypass mode [4.2.1].. Lets not worry about FIFO mode for now..
       - [4.2.6] Retrieving data from FIFO Section
    */

    // Operations on CTRL_REG1. Want to setup Output Data Rate & Bandwidth selection.
    // Lets read what we currently have.
    data_tx = 0x20 | 0x80; // CTRL_REG1 address + Read Bit toggled.
    myGPIO_SPI1_SS1.setLow();
    myLabSPI1.SPI_TxRx_write(data_tx);
    myLabSPI1.SPI_TxRx_read();
    myGPIO_SPI1_SS1.setHigh();

    // Lets attempt to modify to what we desire.
    data_tx = 0x20 | 0x00; // CTRL_REG1 address + Write Bit toggled.
    myGPIO_SPI1_SS1.setLow();
    myLabSPI1.SPI_TxRx_write(data_tx);
    data_tx = 0xBF; // 0b1011_1111 -> Output Data Rate 380Hz, Bandwidth 20(?), Normal mode, Enable all axis.
    myLabSPI1.SPI_TxRx_write(data_tx);
    myGPIO_SPI1_SS1.setHigh();

    // Lets read again to check if we wrote our data.
    data_tx = 0x20 | 0x80; // CTRL_REG1 address + Read Bit toggled.
    myGPIO_SPI1_SS1.setLow();
    myLabSPI1.SPI_TxRx_write(data_tx);
    myLabSPI1.SPI_TxRx_read();
    myGPIO_SPI1_SS1.setHigh();

    // Assume there is nothing to do in registers CTRL_REG2 & CTRL_REG3...

    // Operations on CTRL_REG4.
    // Lets read CTRL_REG4. Want to setup 
    data_tx = 0x23 | 0x80; // CTRL_REG4 address + Read bit toggled.
    myGPIO_SPI1_SS1.setLow();
    myLabSPI1.SPI_TxRx_write(data_tx);
    myLabSPI1.SPI_TxRx_read();
    myGPIO_SPI1_SS1.setHigh();

    // Lets modify this CTRL_REG4..
    data_tx = 0x23 | 0x00; // CTRL_REG4 address + Write bit toggled.
    myGPIO_SPI1_SS1.setLow();
    myLabSPI1.SPI_TxRx_write(data_tx);
    data_tx = 0x90; // 0b1001000 -> Block Data Update (output regs not updated until MSb & LSb reading), Data LSb @ lower address, 500 dps, 4 wire SPI.
    myLabSPI1.SPI_TxRx_write(data_tx);
    myGPIO_SPI1_SS1.setHigh();

    // Lets read CTRL_REG4...
    data_tx = 0x23 | 0x80; // CTRL_REG4 address + Read bit toggled
    myGPIO_SPI1_SS1.setLow();
    myLabSPI1.SPI_TxRx_write(data_tx);
    myLabSPI1.SPI_TxRx_read();
    myGPIO_SPI1_SS1.setHigh();

    // Operations on CTRL_REG5.
    // Lets read CTRL_REG5.
    data_tx = 0x24 | 0x80; // CTRL_REG5 address + Read bit toggled.
    myGPIO_SPI1_SS1.setLow();
    myLabSPI1.SPI_TxRx_write(data_tx);
    myLabSPI1.SPI_TxRx_read();
    myGPIO_SPI1_SS1.setHigh();    

    data_tx = 0x24 | 0x00; // CTRL_REG5 address + Write bit toggled.
    myGPIO_SPI1_SS1.setLow();
    myLabSPI1.SPI_TxRx_write(data_tx);
    data_tx = 0x042; // 0b01000010 -> No reboot of memory, FIFO enable, High-Pass filter disable. Out_Sel according to path in LG3D20 Figure 18 block diagram. INT disable.
    myLabSPI1.SPI_TxRx_write(data_tx);
    myGPIO_SPI1_SS1.setHigh();
    
    data_tx = 0x24 | 0x80; // CTRL_REG5 address + Read bit toggled.
    myGPIO_SPI1_SS1.setLow();
    myLabSPI1.SPI_TxRx_write(data_tx);
    myLabSPI1.SPI_TxRx_read();
    myGPIO_SPI1_SS1.setHigh();

    for (int i = 0 ; i < 5 ; i++) {
    // Lets attempt to read temperature data. Expressed as a two's complement!
    data_tx = 0x26 | 0x80; // OUT_TEMP address + Read bit toggled.
    myGPIO_SPI1_SS1.setLow();
    myLabSPI1.SPI_TxRx_write(data_tx);
    data_rx = myLabSPI1.SPI_TxRx_read();
    myGPIO_SPI1_SS1.setHigh();

    memset(buff, '0', 256);
    sprintf((char*)buff, "temperature data: %d\r\n", (int8_t)data_rx);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 256);
    }

    // No need to check on STATUS_REG for now... 
    
    // Lets just read the contents of the FIFO_CTRL_REG. We will not modify anything here.
    data_tx = 0x2E | 0x80; // FIFO_CTRL_REG address + Read bit toggled.
    myGPIO_SPI1_SS1.setLow();
    myLabSPI1.SPI_TxRx_write(data_tx);
    myLabSPI1.SPI_TxRx_read();
    myGPIO_SPI1_SS1.setHigh();
    
    // [3-20-22]: Lets attempt to enable STREAM mode. Hopefully we will see values below change as we read gyroscope data.
    data_tx = 0x2E | 0x00; // FIFO_CTRL_REG address + Write bit toggled.
    myGPIO_SPI1_SS1.setLow();
    myLabSPI1.SPI_TxRx_write(data_tx);
    data_tx = 0x40; // 0b01000000 -> Stream Mode enable, No FIFO threshold set as interrupts will not be tracked.
    myLabSPI1.SPI_TxRx_write(data_tx);
    myGPIO_SPI1_SS1.setHigh();

    // Lets just read the contents of the FIFO_CTRL_REG...
    data_tx = 0x2E | 0x80; // FIFO_CTRL_REG address + Read bit toggled.
    myGPIO_SPI1_SS1.setLow();
    myLabSPI1.SPI_TxRx_write(data_tx);
    myLabSPI1.SPI_TxRx_read();
    myGPIO_SPI1_SS1.setHigh();

    for (int i = 0; i < 5; i ++) {
    // Lets attempt to read out angular data! Lets start with OUT_X_L / OUT_X_H --> OUT_Y_L&H --> OUT_Z_L&H.
    data_tx = 0x28 | 0x80 | 0x40; // OUT_X_L address + Read bit toggled + MS bit toggled for multi-byte read.
                                  // No idea, but lets see if this works? Should see all data.
    myGPIO_SPI1_SS1.setLow();
    myLabSPI1.SPI_TxRx_write(data_tx);
    uint8_t x_lo = myLabSPI1.SPI_TxRx_read(); // ADDR 0x28
    uint8_t x_hi = myLabSPI1.SPI_TxRx_read(); // ADDR 0x29
    uint8_t y_lo = myLabSPI1.SPI_TxRx_read(); // ADDR 0x2A
    uint8_t y_hi = myLabSPI1.SPI_TxRx_read(); // ADDR 0x2B
    uint8_t z_lo = myLabSPI1.SPI_TxRx_read(); // ADDR 0x2C
    uint8_t z_hi = myLabSPI1.SPI_TxRx_read(); // ADDR 0x2D
    myGPIO_SPI1_SS1.setHigh();

    int16_t x_data = (x_hi << 8) | x_lo ;
    int16_t y_data = (y_hi << 8) | y_lo ;
    int16_t z_data = (z_hi << 8) | z_lo ;

    memset(buff, '0', 256);
    sprintf((char*)buff, "x_hi: 0x%02x, x_lo: 0x%02x, x_data: %d \r\n", x_lo, x_hi, x_data);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 256);     

    memset(buff, '0', 256);
    sprintf((char*)buff, "y_hi: 0x%02x, y_lo: 0x%02x, y_data: %d \r\n", y_lo, y_hi, y_data);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 256);     

    memset(buff, '0', 256);
    sprintf((char*)buff, "z_hi: 0x%02x, z_lo: 0x%02x, z_data: %d \r\n", z_lo, z_hi, z_data);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 256);     
    }

    // After routine is finished, give up the Mutex we took..
    xSemaphoreGive(spi_gatekeeper);

    while(1); // make sure task doesnt end...
}

void SPI1_HAL_TX (void*) {
// AJ_GPIO_Init(); // Slave Select GPIO goes low.... For HAL driver its not needed...

    uint8_t status = L3GD20_ReadID();

    memset(buff, '0', 256);
    sprintf((char*)buff, "status: %02x", status);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 256);

//     uint8_t spi_buf_tx;
//     char spi_buf_rx[20];

//     for (int i = 0; i <= 0x10; i++) {
//       	spi_buf_tx = i;
//         memset(buff, '0', 256);
//         sprintf((char*)buff, "%02x", spi_buf_tx);
//         HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 256);

//         HAL_SPI_Transmit(&hspi1, &spi_buf_tx, 1, 100); // Transmit.

//         delay_us(100);

// //        memset(buff, '0', 256);
// //        sprintf((char*)buff, "SPI1_HAL_Rx: 0x%08x\r\n", spi_buf_rx[0]);
// //        HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 256);
//     }
    while(1);
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
  //  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // AJ - Initialize timer
  HAL_TIM_Base_Start(&htim1);

  // AJ - GPIO Init for SPI... May not wanna do this here & may need to do this after LabSPI constructor is called.

  // AJ - Setting up global `LabSPI_InitStruct mySPI1` SPI_port to communicate on..

  // AJ - Enable SPI1_IRQn interrupt/set priority
  NVIC_SetPriority(SPI1_IRQn, 5);
  NVIC_EnableIRQ(SPI1_IRQn);

  /* USER CODE END 2 */

  /* Init scheduler */
//  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  spi_gatekeeper = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  spi_interrupt_binary_sem = xSemaphoreCreateBinary();
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

    /* SPI TX task */
    // xTaskCreate(SPI1_TX_Thread_NonBlocking, "SPI1_TX_Thread_NonBlocking", 512, NULL, 1, &SPI_TX_thread);

    /* HAL_SPI Task */
    // xTaskCreate(SPI1_HAL_TX, "SPI1_HAL_TX", 256, NULL, 1, NULL);

    /* LG3D20 task */
    xTaskCreate(SPI1_Init_L3GD20, "SPI1_Init_L3GD20", 512, NULL, 1, NULL);

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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
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

