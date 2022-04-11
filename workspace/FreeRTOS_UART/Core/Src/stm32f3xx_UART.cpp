#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "stm32f3xx_UART.hpp"

extern UART_HandleTypeDef huart1;

LabUART::LabUART(LabUART_InitStruct* UART_struct){
    // UART initialization code. Bit manipulation goes here based on the UART Port you want to configure.
    // May need to learn about ISR's a bit before diving into this exercise. (RX side)
    // Init routine may need to enable IRQ's for UART port we are working with.
    // How is NVIC configured for STM32?

    // Obtain information from passed LabUART_InitStruct UART_struct & place it in private variable `UART_init`.
    UART_init.UART_port        = UART_struct->UART_port;
    UART_init.buad_rate        = UART_struct->buad_rate;
    UART_init.word_length      = UART_struct->word_length;
    UART_init.stop_bits        = UART_struct->stop_bits;
    UART_init.parity           = UART_struct->parity;
    UART_init.mode             = UART_struct->mode;
    UART_init.hw_flow_ctl      = UART_struct->hw_flow_ctl;
    UART_init.oversampling     = UART_struct->oversampling;
    UART_init.one_bit_sampling = UART_struct->one_bit_sampling;

    // At this point in our `UART_init` private LabUART_InitStruct, we have all of the data we need from caller to setup out UART device.
    
    // Lets assume some parameters and not utilize what we have for now. We can do this later.

    // Setup word_length: M0 & M1 are at different offsets. With 00, we are configured for:
    // 1 START bit, 8 data bits, N STOP bits.
    // UART_init.UART_port->CR1 &= ~(1 << 12);  // Set M0 - Bit 12 to 0
    // UART_init.UART_port->CR1 &= ~(1 << 28);  // Set M1 - Bit 13 to 0

    // [3-12-22]: For 9 bits (8 data bits + parity bit), we will need to this up properly.
    UART_init.UART_port->CR1 |= (1 << 12);   // Set M0 - Bit 12 to 1
    UART_init.UART_port->CR1 &= ~(1 << 28);  // Set M1 - Bit 13 to 0

    // Setup Baud Rate: We know our Frequency is currently 8MHz. With assumption - calculate Baud Rate & write appropriate value to BRR register.
    // Tx/Rx baud = fclk / USARTDIV. Considering we want to operate at 9600 baud rate formula goes as:
    // USARTDIV = fclk/baud. So - USARTDIV = 8000000/9600 = 833.33 onwards. So, we write 833 decimal or 0x0341 hexadecimal.
    UART_init.UART_port->BRR = 0x0341; // Setup Baud Rate as 9600bps based on 8MHz Frequency. I am writing, not setting bits for this one.

    // Disable Over Sampling:
    UART_init.UART_port->CR1 &= ~(1 << 15); //Set OVER8 - Bit 15 to 0. Lets enable oversampling mode by 16.

    // Setup stop_bits: Lets use 1 stop bit
    UART_init.UART_port->CR2 &= ~(3 << 12);  // Set STOP[1:0] - Bits 12-13 to 00

    // Setup one_bit_sampling config: Lets use 1 sample bit bethod.
    UART_init.UART_port->CR3 |=  (1 << 11);  // Set ONEBIT - Bit 11 to 1. 

    // Setup hw_flow_ctl bits.. though it appears this is not available for UART4/5...
    // UART_init.UART_port->CR3 |=  (1 << 8);   // Set RTSE enable for RTS output enable.

    // Setup parity bits.. Disabling it.
    // UART_init.UART_port->CR1 &= ~(1 << 10);

    // [3-12-22]: Lets setup parity bits. Lets do ODD parity.
    UART_init.UART_port->CR1 |=  (1 << 10); // Set PCE - Bit 10 to 1.
    UART_init.UART_port->CR1 &= ~(1 << 9);  // Clr PS - Bit 9 to 0. Set to EVEN parity.
    // When to enable interrupts.. It appears that PE for Parity Error will occur when parity error is detected in RX mode.


    // Setup our given USART. Enabling just TE (Tx pin) for now.
    // This will emit our starting IDLE frame.
    UART_init.UART_port->CR1 |=  (1 << 3);   // Set TE - Bit 3 to 1.

    // [3-2-22]: Rx Enable.
    UART_init.UART_port->CR1 |=  (1 << 2);   // Set RE - Bit 2 to 1.

    // [3-4-22]: Overrun Disable. Disabling this to simplify RXNEIE interrupt handling, the ORE flag may be interferring with my intentions..
    UART_init.UART_port->CR3 |=  (1 << 12);  // Set OVRDIS - Bit 12 to 1. Disable ORE flag.

    // USART Enable! Start up the peripheral bus.
    UART_init.UART_port->CR1 |=  (1 << 0);   // Set UE - Bit 0 to 1.


    // [3-1-22]: Lets try to move out when we enable / disable our interrupt enable toggles. Might be better to do this elsewhere. Think about it...

    // [2-27-22] Setup TXEIE bits to enable interrupts on the UART port we have passed into class constructor
    // UART_init.UART_port->CR1 |= (1 << 7); // USART_CR1->TXEIE (Bit 7) set to `1` for a USART interrupt to be generated upon TXE=1 in USART_ISR reg.
                                             // Given that we will have an interrupt fire when TXE = 1, we can wait for this interrupt before sending next character

    // [3-1-22] Setup TCIE bits to enable interrupts on the UART port passed to class constructor.
    // UART_init.UART_port->CR1 |= (1 << 6); // USART_CR1->TCIE (Bit 6) set to `1` for a USART interrupt to be generated upon TX=1 in USART_ISR reg.
                                          // Lets see what this does.
}

//LabUART::~LabUART(){
//    delete this;
//}

// Notes on relevant registers / details for STM32 USART peripheral:
/*
  - USART_CR1 Register - used to configure character length per UART frame (excluding start/stop bit)
   - This can be 7/8/9
  - Start Bit is LOW. Stop Bit is HIGH.
  - Transmit Enable (TE) bit must be set to activate transmitter function.
    - Data in transmit shift register is output on TX pin, Clock pulses output on CK pin.
    - Data shifts out LSbit first ion TX. USART_TDR reg consists of a buiffer between internal bus &
      transmit shift register. (I believe this is where we our data to be transmitted)
  - Idle frame is sent after TE Bit is enabled (view DS for visual)
  - Control Register 2, bits 13/12 are used to control num STOP bits (0.5/1/1.5/2).
  - Q: What is LBCL - Last bit clock pulse.. (Page 935)
*/

/* Tx: Character Transmission Procedure (Page 893)
  1) Program M bits in USART_CR1 for word length
  2) Select Baud Rate with the USART_BRR register
  3) Program num STOP bits in USART_CR2
  4) Enable USART by writing the UE bit in USART_CR1 register to 1.
  5) Select DMA enable (DMAT) in USART CR3 if multibuffer communication is needed.
     Configure DMA register as explained in multibuffer communication is desired.
  6) Set the TE bit in USART_CR1 to send an idle frame as first frame in transmission.
  7) Write data to transmit in the USART_TDR register (this clears the TXE bit).
     Repeat this for each data to be transmitted in single buffer case.
  8) After writing last data to USART_TDR register, wait until TC=1.
     This indicates that the transmission of last frame is complete.
     This is required when USART is disabled, or enters the HALT mode to avoid corruption in last transmission.
  Notes:
    - TXE bit when SET by HW indicates:
      - Data has been moved from USART_TDR register to the shift register & that data trasmission has started.
      - USART_TDR register is empty.
      - The next data can be written in the USART_TDR register without overwritting previous data written.
    - This flag also generates an interrupt if the TXEIE bit is set (useful for Rx side..? usecase?)
*/

void LabUART :: UART_TX_blocking(uint8_t data){
    // Debug buffer.
	  char func_buff[100] = {0};

	  // Start with polling method for TX - similar to while (`wait for register to be set/clr`);

    // Assuming we have UE enabled & TE enabled + idle frame has been trasmitted...
    // Write our data to trasmit in USART_TDR register.

    // Check on TXE bit before we hit while.
    // for (int i = 0; i <= 100; i++) func_buff[i] = 0;
    // sprintf((char*)func_buff, "Before - TXE: %li | TC: %li \r\n", ((UART_init.UART_port->ISR & (1 << 7) ) >> 7), ((UART_init.UART_port->ISR & (1 << 6) ) >> 6));
    // HAL_UART_Transmit(&huart1, (uint8_t*)func_buff, strlen(func_buff), 1000);

    // Poll for TXE bit.
    while (!((UART_init.UART_port->ISR & (1 << 7) ) >> 7)); // While USART_ISR->TXE (Bit 7) is !(0) --> Block.
                                                             // When we are at !(1), continue code execution, TXE=1 --> data sent to shift reg

    UART_init.UART_port->TDR = data;

    memset(func_buff, '0', 100);
    sprintf((char*)func_buff, "sent: %c\r\n", data);
    HAL_UART_Transmit(&huart1, (uint8_t*)func_buff, strlen(func_buff), 100);

    // for (int i = 0; i <= 100; i++) func_buff[i] = 0;
    // sprintf((char*)func_buff, "After - TXE: %li | TC: %li\r\n", ((UART_init.UART_port->ISR & (1 << 7) ) >> 7), ((UART_init.UART_port->ISR & (1 << 6) ) >> 6));
    // HAL_UART_Transmit(&huart1, (uint8_t*)func_buff, strlen(func_buff), 1000);

    // Need to isolate USART_ISR --> TC bit and while it is 1 we cannot write more data.

    // We can only send data when ISR --> TC bit (Bit 6) is 0! Until then - halt transmission. May want to accomodate for this somewhere.
}

void LabUART :: UART_TX_nonblocking(uint16_t data, QueueHandle_t queue) {
    // Lets attempt to create a nonblocking version of Tx driver.
    
    // Send data to our queue created in Task.
    xQueueSend(queue, &data, portMAX_DELAY);

    char func_buff[30] = {0};
    for (int i = 0; i <= 30; i++) func_buff[i] = 0;
    sprintf((char*)func_buff, "sending: %c | TXE: %d\r\n", data, ((UART_init.UART_port->ISR & (1 << 7) ) >> 7));
    HAL_UART_Transmit(&huart1, (uint8_t*)func_buff, strlen(func_buff), 30);

    // At this point we have our data on the queue. Let us enable TXEIE interrupts for UART_port.
    UART_init.UART_port->CR1 |= (1 << 7); // Enable USART->CR1 TXEIE bit 7 for Enabling TXE = 1 interrupt.
}

/* Rx Character Trasmission Procedure (Page 896)
    - Assume that we have use the class constructor to setup the UART driver as expected - as detailed in `Character reception procedure`.
  Notes: When a character is recieved:
    - RXNE bit is set to indicate that content of shift registers is transferred to the RDR reg.
      - This means that data is recieved & can be read (along w/ associated error flags)
    - If RXNEIE bit is set & interrupt will be generated.
    - The error flags can be set for frame/noise/overrun errors detected upon reception. PE Flag may be set with RXNE
    - In single buffer mode, clearing the RXNE bit is performed by a software read to the USART_RDR reg.
    - RXNE flag can also be cleared by writing 1 to the RXFRQ in the USART_RQR reg. 
    - The RXNE bit must be cleared before the end of the reception of next character to avoid overrun error.

- Overrun error:
  - The RXNE flag is set after every byte received. 
  - An overrun error occurs if RXNE flag is set when the next data is receivevd or the previous DMA request has not been serviced.
  - Rest is on Page 897.

*/ 

void LabUART :: UART_RX_blocking(QueueHandle_t queue){
    // Debug buffer.
	char func_buff[100] = {0};

    // local variable for returning.
    uint8_t data = 0;

    // Check on RXE bit before we hit while.
    // memset(func_buff, '0', 100);
    // sprintf((char*)func_buff, "Before - RXNE: %li \r\n", ((UART_init.UART_port->ISR & (1 << 5)) >> 5) );
    // HAL_UART_Transmit(&huart1, (uint8_t*)func_buff, strlen(func_buff), 1000);

    // Poll for RXE bit.
    while (!((UART_init.UART_port->ISR & (1 << 5) ) >> 5)); // While USART_ISR->RXNE (Bit 5) is !(0) --> Block.
                                                            // When we are at !(1), continue code execution, RXNE=1 --> data from shift reg in RDR reg.

    data = UART_init.UART_port->RDR;

    // memset(func_buff, '0', 100);
    // sprintf((char*)func_buff, "Reading[1]: %c\r\n", data);
    // HAL_UART_Transmit(&huart1, (uint8_t*)func_buff, strlen(func_buff), 1000);

    if ( xQueueSend(queue, &data, portMAX_DELAY) != pdPASS) {
        memset(func_buff, '0', 100);
        sprintf((char*)func_buff, "RX xQueueSend failure.\r\n", data);
        HAL_UART_Transmit(&huart1, (uint8_t*)func_buff, strlen(func_buff), 1000);
    }

    // memset(func_buff, '0', 100);
    // sprintf((char*)func_buff, "After - RXNE: %li \r\n", ((UART_init.UART_port->ISR & (1 << 5)) >> 5) );
    // HAL_UART_Transmit(&huart1, (uint8_t*)func_buff, strlen(func_buff), 1000);

}

void LabUART :: UART_RX_nonblocking(QueueHandle_t queue){ // [3-12-22] This function should be re-written to use the State Machine approach as discussed with Dad 

    // Debug buffer.
	  char func_buff[100] = {0};

    // Variable for input data.
    uint16_t data = 0;

    while (1) {
        // Enable Interrupts for RXNEIE for UART Rx..
        UART_init.UART_port->CR1 |= (1 << 5); // Enable USART->CR1 RXNEIE bit 5 for Enabling RXNE = 1 interrupt.
        UART_init.UART_port->CR1 |= (1 << 8); // Enable USART->CR1 PEIE bit 8 for Enabling PE = 1 interrupt. If PE=1, Pairty Error has occured.
        
        // Receive data from RXNEIE ISR for UART_port->RDR data.
        xQueueReceive(queue, &data, 1000);

        // Lets print out what we recieved on USART1.
        memset(func_buff, '0', 100);
        sprintf((char*)func_buff, "Received: %c.\r\n", data);
        HAL_UART_Transmit(&huart1, (uint8_t*)func_buff, strlen(func_buff), 1000);
    }
}


