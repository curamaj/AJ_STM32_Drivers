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
#include "stm32f3xx_I2C.hpp"
#include "l3gd20.h"
#include "stm32f3_discovery_accelerometer.h"

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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
// AJ - my tasks.

/*-----------------------------------------------------------*/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

xTaskHandle I2C_master_thread_handle;
xTaskHandle I2C_slave_thread_handle;
// xTaskHandle I2C_RX_thread; for use on slave..?

xSemaphoreHandle i2c_gatekeeper; // Want to ensure only 1 task uses I2C bus at a time. So we will guard access of it.
xSemaphoreHandle i2c_interrupt_binary_sem; // For use in our interrupt routine, flag will be raised when Interrupt is detected.
char buff[256]; // Global buffer to handle sprintf / HAL_UART_TRANSMIT calls for basic print functionality.

// Declaring this as a global for use in I2C1/2_IRQHandler...
LabI2C_InitStruct myI2C1;
LabI2C_InitStruct myI2C2;

// Declaring global for I2C Singal.. This will need to be extern'd to our stm32f3xx_I2C.hpp/.cpp file...
i2c_signal i2c_current_signal = NO_SIGNAL;

// [3-26-22]: Lets add an extern C block to override our Interrupt Handler.
#ifdef __cplusplus
extern "C" {
#endif
// [3-26-22] AJ - From NXP code - we enable interrupts in our `init`/`initSlave` routines.
// We also will be waiting for the semaphore released in our interrupt handler in our transfer function..

// // Event Interrupt Handler.
// void I2C1_EV_IRQHandler(void) {

// }

// // Error Interrupt Handler.
// void I2C1_ER_IRQHandler(void) {

// }

#ifdef __cplusplus
}
#endif


// [3-14-22]: Lets add typedef's for the data within registers we want to access in out L3GD20 chip if needed. 
// Placing sample bitfield below..
// typedef union {
//     uint64_t out_data;
//     struct {
//         uint8_t out_x_l:   8;
//         uint8_t out_x_h:   8;
//         uint8_t out_y_l:   8;
//         uint8_t out_y_h:   8;
//         uint8_t out_z_l:   8;
//         uint8_t out_z_h:   8;
//         uint16_t padding: 16;
//     } __attribute__((packed));
// } out_data;
// [3-20-22]: Perhaps a bitfield mapping laid out like this would let us extract all our angular velocity data a bit easier?
// Realistically, bitfields are better suited for individual bits in a register. 
// So maybe you can practice creating a bitfield map for a certain REG we read from here.

void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

// Need to modify this function for port/pins mapped to I2C1.
// There appears to be multiple mappings for I2C1 listed - going with this one:
// PA14 - AF4 - I2C1_SDA
// PA15 - AF4 - I2C1_SCL
// SMBA is used for SMBus Alerts.
void AJ_I2C_Init_LSM303LDHC() {
    // First up is initializing the clock for GPIO B port via IOBPEN (I-O Port B Enable..?)
    RCC->AHBENR |= (1 << 18);

    // Initialize our GPIO B Port - Pin 6 as an alternate. For I2C1_SCL - Serial Clock.
    // [3-26-22] Note: Our GPIOs need to be configured as Open Drain! Most likely.
    LabGPIO_InitStruct myGPIO_I2C1_SDA_InitStruct;
    myGPIO_I2C1_SDA_InitStruct.GPIO_port = GPIOB;        // Configure port B.
    myGPIO_I2C1_SDA_InitStruct.pin       = 6;            // Configure pin 6.
    myGPIO_I2C1_SDA_InitStruct.pupd      &= ~(3 << 12);  // Set GPIOx_PUPDR - No pull-up No pull-down as configured.
    myGPIO_I2C1_SDA_InitStruct.output    |=  (1 << 6);   // Set GPIOx_OTYPER - Set as Open Drain.
    myGPIO_I2C1_SDA_InitStruct.speed     |=  (3 << 12);  // Set GPIOx_OSPEEDR - High speed.
    LabGPIO_X myGPIO_I2C1_SDA(&myGPIO_I2C1_SDA_InitStruct);
    myGPIO_I2C1_SDA.setAsAlternate(0x4); //Port A - Pin 14 to AF4 - I2C1_SDA, 0b0100 == 0x4.

    // Initialize our GPIO A Port - Pin 7 as an alternate. For I2C1_SDA - Serial Data. 
    LabGPIO_InitStruct myGPIO_I2C_SCL_InitStruct;
    myGPIO_I2C_SCL_InitStruct.GPIO_port = GPIOB;       // Configure port B.
    myGPIO_I2C_SCL_InitStruct.pin       = 7;           // Configure pin 7.
    myGPIO_I2C_SCL_InitStruct.pupd      &= ~(3 << 14); // Set GPIOx_PUPDR
    myGPIO_I2C_SCL_InitStruct.output    |=  (1 << 7);  // Set GPIOx_OTYPER - Set as Open Drain.
    myGPIO_I2C_SCL_InitStruct.speed     |=  (3 << 14); // Set GPIOx_OSPEEDR
    LabGPIO_X myGPIO_I2C_SCL(&myGPIO_I2C_SCL_InitStruct);
    myGPIO_I2C_SCL.setAsAlternate(0x4); //Port A - Pin 15 to AF5 - I2C1_SCL, 0b0100 == 0x4.
}

// Splitting up init routines so they can be called seperately.
void AJ_I2C1_Init() {
    // First up is initializing the clock for GPIO B port via IOBPEN (I-O Port B Enable..?)
    RCC->AHBENR |= (1 << 18);

    // [3-26-22] Note: Our GPIOs need to be configured as Open Drain! Most likely.
    // Initialize our GPIO B Port - Pin 8 as an alternate. For I2C1_SCL - Serial Clock.
    LabGPIO_InitStruct myGPIO_I2C1_SCL_InitStruct;
    myGPIO_I2C1_SCL_InitStruct.GPIO_port = GPIOB;       // Configure port A.
    myGPIO_I2C1_SCL_InitStruct.pin       = 8;           // Configure pin 15.
    myGPIO_I2C1_SCL_InitStruct.pupd      &= ~(3 << 16); // Set GPIOx_PUPDR
    myGPIO_I2C1_SCL_InitStruct.output    |=  (1 << 8);  // Set GPIOx_OTYPER - Set as Open Drain.
    myGPIO_I2C1_SCL_InitStruct.speed     |=  (3 << 16); // Set GPIOx_OSPEEDR
    LabGPIO_X myGPIO_I2C1_SCL(&myGPIO_I2C1_SCL_InitStruct);
    myGPIO_I2C1_SCL.setAsAlternate(0x04); //Port B - Pin 8 to AF4 - I2C1_SCL, 0b0100 == 0x4.

    // Initialize our GPIO B Port - Pin 9 as an alternate. For I2C1_SDA - Serial Data.
    LabGPIO_InitStruct myGPIO_I2C1_SDA_InitStruct;
    myGPIO_I2C1_SDA_InitStruct.GPIO_port = GPIOB;        // Configure port A.
    myGPIO_I2C1_SDA_InitStruct.pin       = 9;            // Configure pin 14.
    myGPIO_I2C1_SDA_InitStruct.pupd      &= ~(3 << 18);  // Set GPIOx_PUPDR - No pull-up No pull-down as configured.
    myGPIO_I2C1_SDA_InitStruct.output    |=  (1 << 9);   // Set GPIOx_OTYPER - Set as Open Drain.
    myGPIO_I2C1_SDA_InitStruct.speed     |=  (3 << 18);  // Set GPIOx_OSPEEDR - High speed.
    LabGPIO_X myGPIO_I2C1_SDA(&myGPIO_I2C1_SDA_InitStruct);
    myGPIO_I2C1_SDA.setAsAlternate(0x04); //Port B - Pin 9 to AF4 - I2C1_SDA, 0b0100 == 0x4.
}

void AJ_I2C2_Init() {
    // First up is initializing the clock for GPIO A port via IOAPEN (I-O Port A Enable..?)
    RCC->AHBENR |= (1 << 17);

    // Initialize our GPIO A Port - Pin 9 as an alternate. For I2C2_SCL - Serial Clock
    LabGPIO_InitStruct myGPIO_I2C2_SCL_InitStruct;
    myGPIO_I2C2_SCL_InitStruct.GPIO_port = GPIOA;       // Configure port A.
    myGPIO_I2C2_SCL_InitStruct.pin       = 9;           // Configure pin 15.
    myGPIO_I2C2_SCL_InitStruct.pupd      &= ~(3 << 18); // Set GPIOx_PUPDR
    myGPIO_I2C2_SCL_InitStruct.output    |=  (1 << 9);  // Set GPIOx_OTYPER - Set as Open Drain.
    myGPIO_I2C2_SCL_InitStruct.speed     |=  (3 << 18); // Set GPIOx_OSPEEDR
    LabGPIO_X myGPIO_I2C2_SCL(&myGPIO_I2C2_SCL_InitStruct);
    myGPIO_I2C2_SCL.setAsAlternate(0x04); //Port A - Pin 9 to AF5 - I2C2_SCL, 0b0100 == 0x4.

    // Initialize our GPIO A Port - Pin 10 as an alternate. For I2C2_SDA - Serial Data
    LabGPIO_InitStruct myGPIO_I2C2_SDA_InitStruct;
    myGPIO_I2C2_SDA_InitStruct.GPIO_port = GPIOA;       // Configure port A.
    myGPIO_I2C2_SDA_InitStruct.pin       = 10;          // Configure pin 15.
    myGPIO_I2C2_SDA_InitStruct.pupd      &= ~(3 << 20); // Set GPIOx_PUPDR
    myGPIO_I2C2_SDA_InitStruct.output    |=  (1 << 10); // Set GPIOx_OTYPER - Set as Open Drain.
    myGPIO_I2C2_SDA_InitStruct.speed     |=  (3 << 20); // Set GPIOx_OSPEEDR
    LabGPIO_X myGPIO_I2C2_SDA(&myGPIO_I2C2_SDA_InitStruct);
    myGPIO_I2C2_SDA.setAsAlternate(0x04); //Port A - Pin 10 to AF5 - I2C2_SDA, 0b0100 == 0x4.
}

#define WRITE           0
#define WRITE_THEN_READ 1

void I2C1_Master_Thread_LSM303LDHC(void* p) {

    // Let us take out i2c_gatekeeper mutex. Will need to return this after task is done. No other task should pre-empt us...
    xSemaphoreTake(i2c_gatekeeper, 1000);

    // data_buf
	  uint8_t data_buf[5];// Lets write/read from CTRL_REG1_A... Setting MSB to allow Auto-Increment later.. Should be able to use ` | 0x80`

    myI2C1.I2C_port = I2C1;
    LabI2C myLabI2C1(&myI2C1);

    AJ_I2C_Init_LSM303LDHC();
    myLabI2C1.init_master();
 
    // Lets attempt to setup our LSM303LDHC as asked in your question bank.
    /* - Enable Temperature Sensor <-- CRA_REG_M - TEMP_EN to `1`.
       - High Resolution Mode <-- Set REG1 - LPen to `0`, Set REG4 - HR Bit to `1`.
       - Taking samples at 100Hz <-- Set REG1_A - ODR[3:0] = 0b0101.
       - +/- 4G scale <-- Set REG4_A - FS[1:0] to `0b01`.
       - Set BDU to prevent overwriting reads while reading current data. (May be told by interviewer?)
         - Set REG4_A - BDU to `1`.
    */
    
    // We know our slave address is 0x19.
    // Lets follow the pattern of READ - MODIFY - WRITE.

    // -- CTRL_REG1_A Modification --
    // Reading from CTRL_REG1_A.
    memset(data_buf, '0', 5);
    data_buf[0] = 0x20;
    myLabI2C1.master_write(1, 0b0011001, (uint8_t*)&data_buf, WRITE_THEN_READ); // 0b0011001 is our LSM303DLHC slave address.
                                                                                // 1 is last param to indicate write_then_read.
    myLabI2C1.master_read (1, 0b0011001, (uint8_t*)&data_buf);
    memset(buff, '0', 256);
    sprintf((char*)buff, "data_buf[0]: 0x%02x\r\n", data_buf[0]);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 256);

    // Modify step. We have our current data from CTRL_REG1_A in our data buff.
    data_buf[0] &= ~(0b11111000); // Clear Out Data in ODR, Setup LPen to Normal Mode config by clearing. Keep Z-Xen as is.
    data_buf[0] |=  (0b01010000); // Setup ODR to 100 Hz.
    uint8_t write_val = data_buf[0];

    // Writing to CTRL_REG1_A.
    memset(data_buf, '0', 5);
    data_buf[0] = 0x20;
    data_buf[1] = write_val; // Set ODR3-1 to `0b0101`, Set LPen to `0`, Keep Zen-Xen as is (do not modify.) Previously set to 0b111.
    myLabI2C1.master_write(2, 0b0011001, (uint8_t*)&data_buf, WRITE); // Sending 2 bytes, Register Address + data to write.

    // Reading from CTRL_REG1_A. Let see if our new value is stored.
    memset(data_buf, '0', 5);
    data_buf[0] = 0x20;
    myLabI2C1.master_write(1, 0b0011001, (uint8_t*)&data_buf, WRITE_THEN_READ); // 0b0011001 is our LSM303DLHC slave address.
                                                                                // 1 is last param to indicate write_then_read.
    myLabI2C1.master_read (1, 0b0011001, (uint8_t*)&data_buf);
    memset(buff, '0', 256);
    sprintf((char*)buff, "data_buf[0]: 0x%02x\r\n", data_buf[0]);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 256);  

    // -- CTRL_REG4_A Modification --
    // Reading from CTRL_REG4_A.
    memset(data_buf, '0', 5);
    data_buf[0] = 0x23;
    myLabI2C1.master_write(1, 0b0011001, (uint8_t*)&data_buf, WRITE_THEN_READ); // 0b0011001 is our LSM303DLHC slave address.
                                                                                // 1 is last param to indicate write_then_read.
    myLabI2C1.master_read (1, 0b0011001, (uint8_t*)&data_buf);
    memset(buff, '0', 256);
    sprintf((char*)buff, "data_buf[0]: 0x%02x\r\n", data_buf[0]);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 256);

    // Modify step.
    data_buf[0] &= ~(0b10110000); // Clear out Data in BDU & FS[1:0]
    data_buf[0] |=  (0b10011000); // Setup BSU - `1`, FS[1:0] to `01` for +/- 4G.
    write_val = data_buf[0];
    
    // Writing to CTRL_REG4_A.
    memset(data_buf, '0', 5);
    data_buf[0] = 0x23;
    data_buf[1] = write_val; // Set BDU to `1`, FS[1:0] to `0b01` for +/-4G operation, HR to `1` for high res. Do not modify other bits.
    myLabI2C1.master_write(2, 0b0011001, (uint8_t*)&data_buf, WRITE); // Sending 2 bytes, Register Address + data to write.

    // Reading from CTRL_REG4_A.
    memset(data_buf, '0', 5);
    data_buf[0] = 0x23;
    myLabI2C1.master_write(1, 0b0011001, (uint8_t*)&data_buf, WRITE_THEN_READ); // 0b0011001 is our LSM303DLHC slave address.
                                                                                // 1 is last param to indicate write_then_read.
    myLabI2C1.master_read (1, 0b0011001, (uint8_t*)&data_buf);
    memset(buff, '0', 256);
    sprintf((char*)buff, "data_buf[0]: 0x%02x\r\n", data_buf[0]);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 256);

    // -- CRA_REG_M modification --
    // Reading from CRA_REG_M.
    memset(data_buf, '0', 5);
    data_buf[0] = 0x00;
    myLabI2C1.master_write(1, 0b0011001, (uint8_t*)&data_buf, WRITE_THEN_READ); // 0b0011001 is our LSM303DLHC slave address.
                                                                                // 1 is last param to indicate write_then_read.
    myLabI2C1.master_read (1, 0b0011001, (uint8_t*)&data_buf);
    memset(buff, '0', 256);
    sprintf((char*)buff, "data_buf[0]: 0x%02x\r\n", data_buf[0]);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 256);

    // Modify step.
    data_buf[0] &= ~(0b10000000); // Clear out existing data in TEMP_EN.
    data_buf[0] |=  (0b10000000); // Setup TEMP_EN as `1`.
    write_val = data_buf[0];

    // Writing to CRA_REG_M. 
    memset(data_buf, '0', 5);
    data_buf[0] = 0x00;
    data_buf[1] = write_val; // Set TEMPEN to `1`. Do not modify other bits.
    myLabI2C1.master_write(2, 0b0011001, (uint8_t*)&data_buf, WRITE); // Sending 2 bytes, Register Address + data to write.

    // Reading from CRA_REG_M.
    memset(data_buf, '0', 5);
    data_buf[0] = 0x00;
    myLabI2C1.master_write(1, 0b0011001, (uint8_t*)&data_buf, WRITE_THEN_READ); // 0b0011001 is our LSM303DLHC slave address.
                                                                                // 1 is last param to indicate write_then_read.
    myLabI2C1.master_read (1, 0b0011001, (uint8_t*)&data_buf);
    memset(buff, '0', 256);
    sprintf((char*)buff, "data_buf[0]: 0x%02x\r\n", data_buf[0]);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 256);

    // After routine is finished, give up the Mutex we took..
    xSemaphoreGive(i2c_gatekeeper);

    while(1); // make sure task doesnt end...
}

void I2C_HAL_LS303DLHC_Thread(void* p) {
    
    // Call our LabI2C Constructor. Need to setup I2C peripheral before GPIO_Init.
    myI2C1.I2C_port = I2C1;
    LabI2C myLabI2C1(&myI2C1);

    AJ_I2C_Init_LSM303LDHC();
    myLabI2C1.init_master();

    // Let us take out i2c_gatekeeper mutex. Will need to return this after task is done. No other task should pre-empt us...
    xSemaphoreTake(i2c_gatekeeper, 1000);

    // For this function, we will use STM's provided HAL layer to send data to/from our sensor.
    BSP_ACCELERO_Init();
    
    for (int i = 0; i < 100; i++) {
        int16_t pDataXYZ[3];
        BSP_ACCELERO_GetXYZ(pDataXYZ);
        memset(buff, '0', 256);
        sprintf((char*)buff, "pDataXYZ[0]: %d, pDataXYZ[1]: %d, pDataXYZ[2]: %d,\r\n", pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]);
        HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 256);     
    }

    // After routine is finished, give up the Mutex we took..
    xSemaphoreGive(i2c_gatekeeper);

    while(1); // make sure task doesnt end...
}

void I2C1_Master_Thread(void* p) {

    xSemaphoreTake(i2c_gatekeeper, 1000);

    // data_buf as needed for I2C transfer.
	uint8_t data_buf[5];

    myI2C1.I2C_port = I2C1;
    LabI2C myLabI2C1(&myI2C1);

    // Master Thread will need to init I2C1.
    // AJ_I2C1_Init();

    // For now I will use this.
    AJ_I2C_Init_LSM303LDHC();
    myLabI2C1.init_master();

    memset(buff, '0', 256);
    sprintf((char*)buff, "!!Master Init'd!!\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 256);

    // Lets attempt to do a read & write to our slave...

    // Let attempt a read first.
    memset(data_buf, '0', 5);
    myLabI2C1.master_read (1, 0x7E, (uint8_t*)&data_buf);
    sprintf((char*)buff, "data_buf[0]: 0x%02x\r\n", data_buf[0]);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 256);

    // Lets attempt a write now.
    memset(data_buf, '0', 5);
    data_buf[0] = 0x10; // Lets attempt to write 0x20.
    myLabI2C1.master_write(1, 0x7E, (uint8_t*)&data_buf, WRITE); // Sending 2 bytes, Register Address + data to write.

    // Lets attempt a read again & see if our value has changed in slave device.
    memset(data_buf, '0', 5);
    myLabI2C1.master_read (1, 0x7E, (uint8_t*)&data_buf);
    sprintf((char*)buff, "data_buf[0]: 0x%02x\r\n", data_buf[0]);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 256);   

    // Done...

    // After routine is finished, give up the Mutex we took..
    xSemaphoreGive(i2c_gatekeeper);
}

// We will honestly want to run these tasks/threads on seperate uC's. It will be a pain to get it working on 1 uC.
// Just debugging 1 uC at a time will prove to be difficult anyways...
void I2C2_Slave_Thread(void* p) {

    xSemaphoreTake(i2c_gatekeeper, 1000);
    // data_buf as needed.
	  uint8_t data_buf[5];

    // Slave Thread will need to init I2C2.
    // AJ_I2C2_Init();

	  // For now I will use this.
    AJ_I2C_Init_LSM303LDHC();

    // myI2C2.I2C_port = I2C2;
    // LabI2C myLabI2C2(&myI2C2);

    // Lets stick to using I2C1 on slave.
    myI2C1.I2C_port = I2C1;
    LabI2C myLabI2C1(&myI2C1);    

    memset(buff, '0', 256);
    sprintf((char*)buff, "!!Slave about to Init!!\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), 256);

    // myLabI2C2.init_slave(0x7E); // 0x7E will be our slave address.
                                   // init_slave routine will just be a while loop running forever expecting Tx/Rx from master.

    myLabI2C1.init_slave(0x7E); // 0x7E will be our slave address.
                                // init_slave routine will just be a while loop running forever expecting Tx/Rx from master.

    // After routine is finished, give up the Mutex we took..
    xSemaphoreGive(i2c_gatekeeper);
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
  // MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // AJ - Initialize timer
  HAL_TIM_Base_Start(&htim1);

  // AJ - GPIO Init for I2C... May not wanna do this here & may need to do this after LabI2C constructor is called.

  // AJ - Setting up global `LabI2C_InitStruct myI2C1` I2C_port to communicate on..

  // AJ - Enable I2C1/2_IRQn interrupt/set priority. Error interrupts may need higher priority.
  // I2C1_EV_IRQn - I2C1 Event Interrupt & EXTI Line23 Interrupt (I2C1 wakeup)
  // I2C1_ER_IRQn - I2C1 Error Interrupt
  // NVIC_SetPriority(I2C1_EV_IRQn, 6);
  // NVIC_EnableIRQ(I2C1_EV_IRQn);
  // NVIC_SetPriority(I2C1_EV_IRQn, 5);
  // NVIC_EnableIRQ(I2C1_EV_IRQn);

  /* USER CODE END 2 */

  /* Init scheduler */
  // osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  i2c_gatekeeper = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  i2c_interrupt_binary_sem = xSemaphoreCreateBinary();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  // defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
    
    /* I2C LSM303LDHC driver task with AJ HAL*/
    //xTaskCreate(I2C1_Master_Thread_LSM303LDHC, "I2C1_Master_Thread_LSM303LDHC", 512, NULL, 1, &I2C_master_thread_handle);

    // Master & Slave Threads below are to run on seperate uC's.

    /* I2C Master Thread (I2C1) */
    xTaskCreate(I2C1_Master_Thread, "I2C1_Master_Thread", 512, NULL, 1, &I2C_master_thread_handle);

    /* I2C Slave Thread (I2C2) */
    //xTaskCreate(I2C2_Slave_Thread, "I2C2_Slave_Thread", 512, NULL, 1, &I2C_slave_thread_handle);

    /* HAL_I2C Task */
    // xTaskCreate(I2C_HAL_LS303DLHC_Thread, "I2C_HAL_LS303DLHC_Thread", 512, NULL, 1, NULL);

    /* Start Scheduler */
    uint8_t start_program[100] = "!!!Program Start!!!\r\n";
    HAL_UART_Transmit(&huart1, start_program, 21, 1000);

    vTaskStartScheduler();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

