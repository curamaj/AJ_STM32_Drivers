/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F3XX_I2C_H
#define __STM32F3XX_I2C_H

#include "stm32f3xx_hal.h"
#include "stm32f3_discovery.h"
#include "stdint.h"
#include "stm32f303xc.h"

//AJ - FreeRTOS header includes
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

// Grabbing huart1 from main.cpp... access printf in *.cpp...
extern UART_HandleTypeDef huart1;

// Message passing between main.cpp / stm32f3xx_I2C.cpp(.hpp)
extern xSemaphoreHandle i2c_interrupt_binary_sem; 

// To be used by high level functions that user can call.
typedef enum {
    WRITE_MODE = 1,
    READ_MODE  = 2,
    ERROR_MODE = 0
} i2c_mode;

// To be used by interrupt handler as needed.
// Need to rewrite these based off proper interrupts to examine for I2C.
typedef enum {
    TXE_SIGNAL  = 1, // TxFIFO data avail
    RXNE_SIGNAL = 2, // RxFIFO data avail
    NO_SIGNAL   = 0
} i2c_signal;

// To be utilized by state machine functions.
typedef enum {
    BUSY_STATE            = 1,
    READ_COMPLETE_STATE   = 2,
    WRITE_COMPLETE_STATE  = 3,
    DONE_STATE            = 0
} i2c_state;

typedef struct{ // This structure is analogous to `I2C_InitTypeDef` - `stm32f3xx_hal_uart.h`. This is my HAL layer for UART initialization.
    I2C_TypeDef* I2C_port; // ** NEED TO PLACE AN I2C PORT HERE! **
}LabI2C_InitStruct;

typedef struct
{
    uint32_t trxSize;     // # of bytes to transfer.
    uint8_t slaveAddr;    // Slave Device Address
    uint8_t firstReg;     // 1st Register to Read or Write
    uint8_t error;        // Error if any occurred within I2C
    uint8_t *pMasterData; // Buffer of the I2C Read or Write
    
    //Slave operation data
    uint8_t bytesRx;      //bytes received
    uint8_t bytesTx;      //bytes transmitted
}LabI2C_TransactionStruct;

class LabI2C
{
    private:
        LabI2C_InitStruct I2C_init; //private class variable. PASS IN an I2C PORT!  

        // Will want to use this struct during the state machine flow of i2c transaction.
        // During the transaction, we will be reading/writing data from this structure for different phases of state machine.

    public:
        LabI2C(LabI2C_InitStruct* I2C_struct); // init common portions of peripheral for master/slave.

        bool master0slave1;

        uint8_t init_master(); // return a status?
        uint8_t init_slave(uint8_t slave_addr);  // return a status?

        // Master routines:
        uint8_t master_write(uint8_t nbytes, uint8_t slave_addr, uint8_t* data_buff, bool write_then_read);
        uint8_t master_read(uint8_t nbytes, uint8_t slave_addr, uint8_t* data_buff);

        // Slave routines:
        uint8_t slave_write(uint8_t data);
        uint8_t slave_read();

        // Functions that effectively go through a state machine.
        i2c_state i2cMasterStateMachine();
        i2c_state i2cSlaveStateMachine(); 
        

//        ~LabI2C();
};

#ifdef __cplusplus
}
#endif

#endif
