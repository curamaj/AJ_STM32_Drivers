#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "stm32f3xx_SPI.hpp"
extern TIM_HandleTypeDef htim1;
void delay_us_spi (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

LabSPI::LabSPI(LabSPI_InitStruct* LabSPI_struct){

    // Lets link our private class variable to the LabSPI_struct passed.
    SPI_init.SPI_port = LabSPI_struct->SPI_port;

    // Assume in AJ_GPIO_Init we have configured out GPIO pins to proper AF values for peripherals to be connected.

    // Lets enable the APB for SPI Peripheral to be active.
    RCC->APB2ENR |= (1 << 12); // RCC APB2ENR Reg - Bit 12 - Set to 1 for SPI1EN for SPI1 Clock Enable.

    // Following Configuration of SPI - Page 962, Section 30.5.7.
    // SPI_CR1 Register:
    // Configure Serial Clock Baud Rate. Currently, PCLK is at 8MHz. Need to determine Speed to communicate to `L3GD20`.
    // It appears the mac speed for SPI Clock Frequency is 10 MHz. Lets attempt to communicate at 4MHz.
    SPI_init.SPI_port->CR1 &= ~(7 << 3); // Setup BR[2:0] to `000`. 8MHz/2 = 4MHz operation.
    // SPI_init.SPI_port->CR1 |=  (3 << 3); // Setup BR[2:0] to `011`. 8MHz/16 = 500KHz operation. Slow it down a bit...

    // Configure CPOL (Clock Polarity) & CPHA (Clock Phase). We will leave these as is.
    SPI_init.SPI_port->CR1 &= ~(1 << 1); // [3-17-22] Setup CPOL to `0`, CK to 0 when idle. Setting it up like this at l3gd20.c appears to need SCLK like this...???
    SPI_init.SPI_port->CR1 &= ~(1 << 0); // Setup CPHA to `0`, The 1st clock transition is the first data capture edge.
    // ^[3-13-22]: AJ - May need to look into what this is.. CPHA...

    // We are going to operate in the normal mode where we have 1 Master to X Slaves. 
    // No need to alter RXONLY (Simplex Mode Toggle) or BIDIMODE (Half-Duplex Mode Toggle) Figure 350/351 for details & pins.
    // To be on safe side, lets setup BIDIMODE to 0 - 2-line unidirectional data mode.
    SPI_init.SPI_port->CR1 &= ~(1 << 15);

    // Configure LSBFIRST bit for frame format. Lets assume LSB first.
    SPI_init.SPI_port->CR1 &= ~(1 << 7); // Setup LSBFIRST to `0`, MSB first. It appears this is what L3GD20 expects...

    // Configure CRCL / CRCEN bits if CRC needed
    // [3-17-22] Configure SSM & SSI as needed: This may be our missing piece?
    // Need to do 1 Master to X Slave config. Lets setup SSM = 1 / SSI = 1. We want to toggle slaves via GPIO, not NSS.
    SPI_init.SPI_port->CR1 |=  (1 << 9); // Setup SSM to `1`. Software Slave Management Enabled...
    SPI_init.SPI_port->CR1 |=  (1 << 8); // Setup SSI to `1`. Given SSM bit is set, I/O value of NSS pin is ignored. Want to use as GPIO.

    // Configure MSTR bit as needed (for MultiMaster config). We need this to indicate we are Master.
    SPI_init.SPI_port->CR1 |=  (1 << 2); // Setup MSTR to `1`, we are Master.

    // SPI_CR2 Register:
    // Configure DS[3:0] bits to select data length of transfer. Need to check length of data from `L3GD20`.
    // I am not sure - but it appears to operate on 8-bits of transfer length at a time. Lets set it up as such.
    // [3-14-22]: It is possible this value needs to be altered based on data to send...
    SPI_init.SPI_port->CR2 |=  (7 << 8); // Setup DS[3:0] to `0111` for 8-bit data length for SPI transfers. It appears this should work with L3GD20...
                                         // Note: I wonder if this value can be changed even if SPE is enabled. So we can do diff data_size Tx/Rx...

    // Configure SSOE as needed.
    // Configure FRF bit is TI protocol is needed.
    // Configure NSSP bit if the NSS pulse mode between 2 data units is required.

    // Configure the FRXTH bit. The RXFIFO threshold must be aligned to read access size for the SPIx_DR register.
    // Considering we are operating in 8-bits of transfer length, we may want to set RXNE event to generate when 32-bit FIFO has 8-bits.
    SPI_init.SPI_port->CR2 |=  (1 << 12); // Setup FRXTH to `1` for RXNE event generated when FIFO lvl >= [32 * (1/4) = 8 bits]

    // [3-17-22]: Ensure that I2SMOD selection indicates that SPI Mode is selected
    SPI_init.SPI_port->I2SCFGR &= ~(1 << 11); // Setup I2SMOD to `0` for SPI Mode Selction.

    // Initialize LDMA_TX / LDMA_RX bits if FMA is used in packed mode.
    
    // Write to SPI_CRCPR register if CRC polynomial is needed.
    // Write proper DMA register as needed (Configure FMA streems for SPI Tx / Rx in DMA registers if the DMA streams are used).

    // I am assuming at this point, we probably need to enable SPI! Lets do this.
    SPI_init.SPI_port->CR1 |=  (1 << 6); // Setup SPE to `1` to enable SPI peripheral.
}

//LabSPI::~LabSPI(){
//    delete this;
//}

// // Debug buffer.
// char func_buff[100] = {0};
// // Lets print out what we recieved on USART1.
// memset(func_buff, '0', 100);
// sprintf((char*)func_buff, "Received: %c.\r\n", data);
// HAL_UART_Transmit(&huart1, (uint8_t*)func_buff, strlen(func_buff), 1000);

// Note the following files as they may be insightful during dev of this driver...
//  - Drivers/BSP/Components/l3gd20/l3gd20.c
//  - Drivers/BSP/Components/l3gd20/l3gd20.h
//  - Drivers/BSP/STM32F3-Discovery/stm32f3_discovery_gyroscope.c
//  - Drivers/BSP/STM32F3-Discovery/stm32f3_discovery_gyroscope.h
//  - Drivers/BSP/STM32F3-Discovery/stm32f3_discovery.c

// The device we will be communicating with is the L3GD20 3-Axis Gyroscope. Will need to take a look at its data sheet.

/* Procedure for enabling SPI (Section 30.5.8)
*/

/* Section 30.5.9: Data transmission & reception procesures notes:
  - Read access to the SPIx_DR reg returns the oldest value stored in the RXFIFO that has not been read yet.
  - Write access to the SPIx_DR reg stores the written data in the TXFIFO  at the end of a send queue.
  - Read access must be aligned with the RXFIFO threshold configured by SPIx_CR2->FRXTH bit.
  - FTLVL[1:0] & FRLVL[1:0] bits indicate current occupany levels of both FIFOs.
  
  - Read access to the SPIx_DR reg must be managed by the RXNE event, which is triggered upon data in RXFIFO reaching FRXTH threshold.
  - When RXNE is cleared, RXFIFO is considered Empty.

  - Write access of a data frame to transmit is managed by the TXE event.
  - TXE event is triggered when TXFIFO level is <= 1/2 its capacity, else TXE is cleared & TXFIFO is considered Full.

  - With this paradigm, RXFIFIO can store 4 data frames. TXFIFO can only store up to 3 data frames, given data format is not greater than 8 bits.
  - This is to prevent corruption of having 3 `8 bit data frames`, then having a `16 bit data frame` come in (FIFO is 32 bits!) 

  - TXE & RXNE can be polled/handled with interrupts.

  - If next data is recieved when RXFIFO is full, an overrun event occurs & is indicated via SPI status flag OVR, can be polled/interrupted when this occurs.

  - BSY bit indicates ongoing transactions of a current data frame. 
  - BSY is set for Master between DF's, but becomes low for a min duration of one SPI clock at slace between each DF transfer.

  - Disable procedrure for transmit/recieve mode
    - Wait until FTLVL[1:0] = `00` (no more data to transfer)
    - Wait until BSY=0 (last frame is processed)
    - Disable SPI (SPE = 0)
    - Read data until FRLVL[1:0] = `00` (read all the recieved data)

[3-14-22] Section 30.6: SPI Interrupts: Noting relevant registers for now... May need to read up on how these interrupts fire/should be cleared.
  - Transmit TXFIFO ready to be loaded. Event Flag: TXE. Enable Control bit: TXEIE.
  - Data recieved in RXFIFO. Event Flag: RXE. Enable Control bit: RXNEIE.

- [3-13-22]: Read up to here. Can pick up from `Data Packing` for completeness on page 965. DMA/Communication Diagrams could be worth examining.
*/


/* Section 30.9: SPI Register notes..
  - 
*/

/* L3GD20 Notes from `l3gd20.pdf`:
  - Section 4.2 FIFO:
    - The L3GD20 embeds 32 slots of 16-bit data FIFO for each of the 3 output channels - yaw, pitch & roll.
    - We will start with either Bypass Mode, then attempt another mode if time permits (FIFO/STREAM/etc..)
    - Each mode is selected by the FIFO_MODE bits in FIFO_CTRL_REG (2Eh).
    - There are Dedicated interrupts that can be generated on DRDY/INT2 pin configured thru CTRL_REG3(22h). This is on another line.
    - Event detection information is available in FIFO_SRC_REG(2Fh).
    - Watermark level can be configured to WTM[4:0] in FIFO_CTRL_REG (2Eh).
  
  - Section 4.6 Retrieve data from FIFO: Note - We will still need to read data from FIFO regs at the first address..?
    - FIFO data is read through OUT_X([L]28h,[H]29h) / OUT_Y([L]2Ah,[H]2Bh) / OUT_Z([L]2Ch,[H]2Dh)
  
  - Section 5.2 SPI bus interface:
    - CS - Serial Port Enable is controlled by SPI master - LO at Tx start, HI at Tx end.
    - SPC - Serial Port Clock is controlled by SPI master.
    - SDI/SDO - Serial Data input/output. They are driven at `FALLING EDGE` of SPC & should be captured at `RISING EDGE`.
    - Both Read/Write Register commands complete in 16 CLK pulses or in multiples of 8 in case of multiple bytes read/write.
    - Frame: [Refer to Figure 12 if needed.]
      - Bit 0 - RW bit: Read/Write.
        - When 0, data DI[7:0] is written to the device. 
        - When 1, data DO[7:0] from the device is read. In read case, the chip will drive `SDO` at the start of bit 8.
      - Bit 1 - MS bit: Address Increment..?
        - When 0, the address remains unchanged in multiple read/write commands. 
        - When 1, the address will be auto-incremented in multiple read/write comamnds.
      - Bit 2-7 - AD bits: Address.
        - This is the address field of the indexed register.
      - Bit 8-15 - Data. Can be Data In / Data Out.
        - DI[7:0] in write mode. This is the data that will be written to device, !MSbit first!. Sent on SDI.
        - DO[7:0] in read mode. This is the data that will be read from device, !MSbit first!. Sent on SDO.
        - For 2-byte - you would have something like D[7:0] - D[15:8] as yout data.
      - In multiple read/write commands, further clocks of 8 clock periods will be added.
        - When MS bit = 0, the address used to read/write data remains the same for every block.
        - When MS bit = 1, the address used to read/write data is incremented at every block.


  - [3-20-22]:
    - Currently have data setup as STREAM. May want to look into this a bit later.
    - BDU bit is quite important for operation here.
      - LG3D20 keeping getting readings, so you might read the LSB from 1 reading and the MSB from another reading if you're not careful about synchronizing to the data-ready flag.
      - The BDU bit freezes the data once you start reading the LSB until you also read the MSB.
      - If you are reading SPI data at a lower frequency than what the LG3D20 is operating at internally, you probably would see the data change quite alot the BDU bit is not set to freeze data once LSB - MSB portion is read

*/

extern char local_buff[50];
spi_state spi_current_state; // Keep track of state within out SPI_transfer function.
// [3-18-22]: I feel this SPI Transfer routine is a bit overly complex.... Keep it Simple Stupid doesnt seem to apply here...
// [3-18-22]: Attempting re-write in a new function below...
uint8_t LabSPI :: SPI_transfer(uint8_t data) {
    // Likely just need to send data... This routine may be used to both transfer/recieve data to our slave, returned value would be data received from slave.

    memset(local_buff, '0', 50);
    sprintf((char*)local_buff, "About to enable interrupts...\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 50);

    spi_current_state = PENDING_STATE; // Pending state.

    memset(local_buff, '0', 50);
    sprintf((char*)local_buff, "PENDING_STATE\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 50);
 
    while (spi_current_state) { // when spi_current_state == 0, we are DONE.
                                // spi_current_state != DONE_STATE for clarity.

        // Lets print out the contents of CR2 before we alter interrupt bits...
    	// memset(local_buff, '0', 50);
        // sprintf((char*)local_buff, "API! SPI_init.SPI_port->CR2: ");
        // HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 50);
        
        // for (int i = 31 ; i >= 0 ; i--) {
        //   	memset(local_buff, '0', 50);
        //     sprintf((char*)local_buff, "%i", ( ( SPI_init.SPI_port->CR2 & (1 << i) ) >> i) );
        //     HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 50);
        // }

        // memset(local_buff, '0', 50);
        // sprintf((char*)local_buff, "\r\n");
        // HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 50);

        // May want to enable our interrupt here for TXE.. Maybe RXNE as well?
        if (spi_current_state == PENDING_STATE) {
            SPI_init.SPI_port->CR2 |= (1 << 7); // Enable interrupt for TXEIE.
        }
        else if (spi_current_state == TRANSMIT_STATE) {
        	SPI_init.SPI_port->CR2 |= (1 << 6); // Enable interrupt for RXNEIE.
        }

        if ( xSemaphoreTake(spi_interrupt_binary_sem, 1000) == pdTRUE) { // If we are able to take the spi_interrupt_binary_sem, it has been given by the ISR.
                                                                         // Need to check `spi_current_signal` for next action to take.

            memset(local_buff, '0', 50);
            sprintf((char*)local_buff, "spi_interrupt_binary_sem Taken!\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 50);

            if (spi_current_signal == TXE_SIGNAL) { // we can send data to TxFIFO that will be delivered to slave.
                spi_current_state = TRANSMIT_STATE; // Transmit state.

                memset(local_buff, '0', 50);
                sprintf((char*)local_buff, "TRANSMIT_STATE - FTLVL: 0x%01x\r\n", (SPI_init.SPI_port->SR & (3 << 11)) >> 11 );
                HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 50);

                *(__IO uint8_t *)&SPI_init.SPI_port->DR = data; // write to TxFIFO.

                memset(local_buff, '0', 50);
                sprintf((char*)local_buff, "Wrote to Data Register: 0x%01x\r\n", data);
                HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 50);

                // It appears that the `*(__IO uint8_t *)&SPI_init.SPI_port->DR` is needed so we can force only 8 bit write to our destination register...
            }
            else if (spi_current_signal == RXNE_SIGNAL) { // we have data from RxFIFO that can be read by master.
                spi_current_state = RECIEVE_STATE; // Recieve state.

                memset(local_buff, '0', 50);
                sprintf((char*)local_buff, "[1]RECIEVE_STATE - FRLVL: 0x%01x\r\n", (SPI_init.SPI_port->SR & (3 << 9)) >> 9 );
                HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 50);

                // Set RXNEIE interrupts. Expecting there to be junk data to collect from RxFIFO after sending over any first byte of data.
                SPI_init.SPI_port->CR2 |= (1 << 6);
                if ( xSemaphoreTake(spi_interrupt_binary_sem, 1000) == pdTRUE ) {
                    if (spi_current_signal == RXNE_SIGNAL) {
                        uint8_t recieved_data = *(__IO uint8_t *)&SPI_init.SPI_port->DR; // lets throw away any data we got in TRANSMIT_STATE. Can be all 0's.

                        memset(local_buff, '0', 50);
                        sprintf((char*)local_buff, "RECIEVE_STATE - RxData: 0x%02x\r\n", recieved_data);
                        HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 50);
                    } else {
                        return 0xF0;
                    }
                } else {
                    return 0xF1;
                }

                memset(local_buff, '0', 50);
                sprintf((char*)local_buff, "[2]RECIEVE_STATE - FRLVL: 0x%01x\r\n", (SPI_init.SPI_port->SR & (3 << 9)) >> 9 );
                HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 50);

                // Set TXE interrupt. We want to ensure we have room on TxFIFO to write our dummy byte.
                SPI_init.SPI_port->CR2 |= (1 << 7);
                if ( xSemaphoreTake(spi_interrupt_binary_sem, 1000) == pdTRUE ) {
                    if (spi_current_signal == TXE_SIGNAL) {                        
                        *(__IO uint8_t *)&SPI_init.SPI_port->DR = 0xFF; // Send over a dummy byte.

                        memset(local_buff, '0', 50);
                        sprintf((char*)local_buff, "Sent out Dummy Byte.\r\n");
                        HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 50);
                    } else {
                        return 0xF2;
                    }
                } else {
                    return 0xF3;
                }

                // Set RXNE interrupt, wait for RxFIFO to contain data we want to read.
                SPI_init.SPI_port->CR2 |= (1 << 6); // Enable interrupt for RXNEIE.
                if ( xSemaphoreTake(spi_interrupt_binary_sem, 1000) == pdTRUE ) {
                    if (spi_current_signal == RXNE_SIGNAL) {
                        memset(local_buff, '0', 50);
                        sprintf((char*)local_buff, "[3]RECIEVE_STATE - FRLVL: 0x%01x\r\n", (SPI_init.SPI_port->SR & (3 << 9)) >> 9 );
                        HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 50);

                        uint8_t recieved_data = *(__IO uint8_t *)&SPI_init.SPI_port->DR;

                        spi_current_state = DONE_STATE; // Done state. We expect to have transmitted, then recieved so presumably at this point we are DONE.

                        memset(local_buff, '0', 50);
                        sprintf((char*)local_buff, "[4]RECIEVE_STATE - FRLVL: 0x%01x\r\n", (SPI_init.SPI_port->SR & (3 << 9)) >> 9 );
                        HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 50);

                        memset(local_buff, '0', 50);
                        sprintf((char*)local_buff, "RECIEVE_STATE - RxData: 0x%02x --> DONE_STATE\r\n", recieved_data);
                        HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 50);

                        return recieved_data;
                    } else {
                        return 0xF4;
                    }
                } else {
                    return 0xF5;
                }

            }
            else if (spi_current_signal == NO_SIGNAL) {
                spi_current_state = ERROR_STATE;
                return 0xF6;
            }
        }
        else {
            spi_current_state = ERROR_STATE;

            //continue;
            return 0xF7;
        }
    }
    return 0xFF; // We shouldnt get here...
}

uint8_t LabSPI :: SPI_TxRx_read() {
    memset(local_buff, '0', 50);
    sprintf((char*)local_buff, "SPI_TxRx_read.\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 50);
    spi_current_mode = READ_MODE;
    // Sending dummy data.
    return SPI_TxRx(0xFF);
}

uint8_t LabSPI :: SPI_TxRx_write(uint8_t data) {
    memset(local_buff, '0', 50);
    sprintf((char*)local_buff, "SPI_TxRx_write: 0x%02x\r\n", data);
    HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 50);
    spi_current_mode = WRITE_MODE;
    // Sending actual data.
    return SPI_TxRx(data);
}

uint8_t LabSPI :: SPI_TxRx_writeln(uint8_t* data, size_t len) {
    for (size_t i = 0 ; i < len; i++) {
        SPI_TxRx_write(data[i]);
    }
    return 0;
}

uint8_t LabSPI :: SPI_TxRx(uint8_t data) {
    // returned data.
    uint8_t return_data = 0;

    if (spi_current_mode == WRITE_MODE || spi_current_mode == READ_MODE) {
        // Enable interrupt for TXEIE.
        SPI_init.SPI_port->CR2 |= (1 << 7);
    } 
    else if (spi_current_mode == ERROR_MODE) {
        // We should never hit this.
        return 0xF0; 
    }

    while ( xSemaphoreTake(spi_interrupt_binary_sem, 1000) == pdTRUE ) {
        
        if (spi_current_signal == TXE_SIGNAL) {
            // Write to DR.
            *(__IO uint8_t *)&SPI_init.SPI_port->DR = data;

            // Enable interrupt for RXNEIE. Expect to obtain an RXNE_SIGNAL after this.
            SPI_init.SPI_port->CR2 |= (1 << 6);
        } 
        
        else if (spi_current_signal == RXNE_SIGNAL) {
            // Read from DR.
            return_data = *(__IO uint8_t *)&SPI_init.SPI_port->DR;

            if (spi_current_mode == WRITE_MODE) {
                // In read mode, we will want to check what the returned data was, it is what we requested from our peripheral.
                memset(local_buff, '0', 50);
                sprintf((char*)local_buff, "TxRx_write returned: 0x%02x\r\n", return_data);
                HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 50);
                return return_data; //0x00; ..
            } else if (spi_current_mode == READ_MODE) {
                // Do not want to return any meaningful data when in write mode. Any recieved data on RxFIFO will be junk.
                memset(local_buff, '0', 50);
                sprintf((char*)local_buff, "TxRx_read returned: 0x%02x\r\n", return_data);
                HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 50);
                return return_data;
            }

            else if (spi_current_mode == ERROR_MODE) {
                // We should never hit this. For now.
                return 0xF1;
            }
        } 

        // We dont have a reason NO_SIGNAL will occur. 
        // When we recieve an interrupt, based on the source we should have a signal to read in spi_current_signal.
        else if (spi_current_signal == NO_SIGNAL) {
            return 0xF2;
        }
    }
    return 0xFF;
}


