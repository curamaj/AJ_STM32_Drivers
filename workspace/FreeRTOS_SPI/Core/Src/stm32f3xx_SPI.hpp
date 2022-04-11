/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F3XX_SPI_H
#define __STM32F3XX_SPI_H

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

typedef enum {
    PENDING_STATE  = 1,
    TRANSMIT_STATE = 2,
    RECIEVE_STATE  = 3,
    ERROR_STATE    = 4,
    DONE_STATE     = 0
} spi_state;

typedef enum {
    TXE_SIGNAL  = 1, // TxFIFO data avail
    RXNE_SIGNAL = 2, // RxFIFO data avail
    OVR_SIGNAL  = 3, // Lets worry about Error signals a bit later... Have to enable ERRIE control bit anyways before any of these may occur...
    UDR_SIGNAL  = 4,
    FRE_SIGNAL  = 5,
    NO_SIGNAL   = 0  // Potentially an error scenario. We should never be seeing NO_SIGNAL.
} spi_signal;

typedef enum {
    WRITE_MODE = 1,
    READ_MODE  = 2,
    ERROR_MODE = 0
} spi_mode;

typedef struct{ // This structure is analogous to `SPI_InitTypeDef` - `stm32f3xx_hal_uart.h`. This is my HAL layer for UART initialization.
    SPI_TypeDef* SPI_port; // ** NEED TO PLACE AN SPI PORT HERE! **
}LabSPI_InitStruct;

/* We want to `take` this binary semaphore that will be `given` from out ISR based on write/read...
   We will know if out ISR occured for write/read based on a global `signal` enum we need to define...
   I am not sure if our semaphore methods will work on the semaphore extern'd like this.. will have to wait & see.
*/
extern xSemaphoreHandle spi_interrupt_binary_sem; 

/* This `spi_signal` is a global modified by our interrupt service routine.
  Steps:
    1) In Transfer, set spi_state to PENDING. Wait for spi_interrupt_binary_sem - perform xSemaphoreTake(spi_interrupt_binary_sem, 10000)
    2) Upon start, TxFIFO is empty, so TXE = 1 as it has capacity. At this point, given TXEIE is enabled, our ISR will occur for TXE = 1.
    3) In ISR, disable interrupts (all..?) & change current_spi_signal to the desired signal. In this case, we set it to TXE_SIGNAL.
    4) In ISR, perform xSemaphoreGive(spi_interrupt_binary_sem).
    5) In Transfer, check what signal has been recieved. If we obtain TXE_SIGNAL, set spi_state to TRANSMIT & send data to SPI_DR...
    6) In Transfer, go back to waiting for spi_interrupt_binary_sem.
    -- Some Guesses in next behavior ahead... --
    7) At this point, we have sent over the data to our slave... Slave has now sent back some data, which is on RXFIFO.
       Given RXNEIE is enabled, our ISR will occur for RXNE = 1. In Transfer, we are still waiting for binary_semaphore & signal...
    8) ... back to 3) but with RXNE & signals...

  Some questions about this approach...
    a) In transfer routine, should we be waiting for spi_interrupt_binary_sem?
    b) What happens if we enable both transmit/recieve interrupts..? I feel some collision may occur...
    c) What happens if we have the Tx/Rx signal come too quickly? 
    d) After a Binary Semaphore has been given, would we need to wait for it to be taken before a `give` can occur again?
    e) After we take the Binary semaphore in Transfer function, so we need to guard the rest of the code before going back to wait state?
       I am saying this because we may need to ensure that we handle the current interrupt signal.
       This is so before any other signals come in that may occur, we have done what we needed to do for the current signal.....
    e) Anything else............
*/
extern spi_signal spi_current_signal;

// Grabbing huart1 from main.cpp... access printf in *.cpp...
extern UART_HandleTypeDef huart1;

class LabSPI
{
    private:
        LabSPI_InitStruct SPI_init; //private class variable for LabGPIO_InitStruct
        // xSemaphoreHandle spi_transfer_gatekeeper; // should this be public? should this only be used in LabSPI class? 
                                                     // is this even needed?
        
        // For write/read --> uint8_t TxRx function, we are going to be using a `spi_mode` enum to indicate if we are writing/reading.
        // For use internally in SPI_TxRx, setup by read/write.
        spi_mode spi_current_mode;

        uint8_t SPI_TxRx(uint8_t data);


    public:
        LabSPI(LabSPI_InitStruct* SPI_struct);
        uint8_t SPI_transfer(uint8_t data); // assuming a max of 8 bit data.
        // uint16_t SPI_transfer_16bit(uint16_t data, uint8_t data_size);
        
        // Both of the function below will internally use SPI_TxRx for read/write.
        uint8_t SPI_TxRx_read();
        uint8_t SPI_TxRx_write(uint8_t data);

        uint8_t SPI_TxRx_writeln(uint8_t* data, size_t len);

//        ~LabSPI();
};

#ifdef __cplusplus
}
#endif

#endif
