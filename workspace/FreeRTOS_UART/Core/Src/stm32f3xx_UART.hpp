/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F3XX_UART_H
#define __STM32F3XX_UART_H

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


/* USART Functional Description:
  - 2 pins, Tx & Rx. Transmit / Recieve.
  - RX: Serial Data Input
  - TX: When there is no data to transmit, this line is HI.
*/

/* USART frame Composition & relevant info/registers
  - Idle line prior to transmit/recieve
  - a) START bit (LO)
  - b) 7-9 bit data - LSBit first.
  - c) 0.5/1/1.5/2 STOP bit(s) 

  - USART interfaces uses a baud rate generator based on current CPU speed (believe we are at 8MHz.)
  - a) Status Register (USART_ISR)
  - b) Receive / Transmit Data Registers (USART_RDR / USART_TDR)
  - c) Baud Rate Register (USART_BRR)
  - d) Guard-Time Register (USART_GTPR) when in Smartcard mode.

*/

typedef struct{ // This structure is analogous to `UART_InitTypeDef` - `stm32f3xx_hal_uart.h`. This is my HAL layer for UART initialization.
    USART_TypeDef* UART_port; // input USART1/USART2/USART3/UART4/UART5 here.
    uint32_t buad_rate;        // Set bits in BRR regsiter: Use a number like 9600 for baud rate.
    uint32_t word_length;      // Set bits in CR1: Word length per data frame.
    uint32_t stop_bits;        // Set bits in CR2: STOP bit(s) per data frame.
    uint32_t parity;           // Set bits in CR1: Parity bit enable / configuration in data frame.
    uint32_t mode;             // If Rx or Tx mode is enabled/disabled.
    uint32_t hw_flow_ctl;      // Set bits in CR3: CTS hardware flow control enabled/disabled.
    uint32_t oversampling;     // Set bits in CR1: (USART disabled! UE == 0!) Over sampling enabled/disabled (achieve higher speed).
    uint32_t one_bit_sampling; // Set bits in CR3: (USART disabled! UE == 0!) One sample bit method enable (Rx can handle clk deviations with this).
}LabUART_InitStruct;

class LabUART
{
    private:
        LabUART_InitStruct UART_init; //private class variable for LabGPIO_InitStruct

    public:

        LabUART(LabUART_InitStruct* UART_struct);
        // TODO: Fill in methods for init(), transmit(), receive() etc.
        void UART_TX_blocking(uint8_t data);
        void UART_TX_nonblocking(uint16_t data, QueueHandle_t queue);
        void UART_RX_blocking(QueueHandle_t queue);
        void UART_RX_nonblocking(QueueHandle_t queue);

//        ~LabUART();
};

#ifdef __cplusplus
}
#endif

#endif /* __STM32F3XX_GPIO_H */
