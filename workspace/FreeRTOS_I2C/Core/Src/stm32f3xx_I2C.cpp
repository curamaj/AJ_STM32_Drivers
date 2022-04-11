#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "stm32f3xx_I2C.hpp"

extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1; // For use to send data out on UART for debug. To be used in Slave SM.

// For use between main / stm32f3xx_I2C.cpp object...
extern i2c_signal i2c_current_signal;

void delay_us_i2c (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

i2c_state i2c_current_state;
char local_buff[50];

// [3-26-22]: Lets start with our basic init routine. This will configure registers for I2C communication. 
// Specifics to init in slave/master goes in their own routines.

// 3-27-22 - ?? Notes: This might take a while.. Id say probably 2-3 weekend type of days.
/* 28.4.3 Mode Selection
  - Operating Modes include Slave Tx/Rx & Master Tx/Rx.
  - By default it operates in Slave mode.
    - Switches to master when it generates a START condition.
    - Switches from master to slave upon arbitration loss or STOP generation occurs - allows multi-master config.
*/

/* 28.4.4 I2C Initialization
  - Initialization Flow:
    - Clear PE bit in I2C_CR1.
    - Configure ANFOFF (Analog Noise Filter) & DNF (Digital Noise Filter) in I2C_CR1.
    - Configure PRESC / SCLDEL (SCL Delay config) / SDADEL (SDA Delay Config) in I2C_TIMINGR.
      - [SDADEL] Data Hold Time: how long after data appears on SDA line after doing a write to the I2C_TXDR - in case of Transmission.
      - [SCLDEL] Data Setup Time: how long after data is sent on SDA that cloking begins. (for samples to start being taken..?) 
      - Formulas in I2C Timings section will help with this as needed (Figure 294.).
    - Additional Config:
      - In master mode, the PRESC, SCLH & SCLL bits need to be configured before enabling peripheral.
        - [SCLH] - upon rising edge detection, delay before SCL is forced low
        - [SCLL] - upon falling edge detection, delay before releasing SCL output. 
      - In slave mode, the NOSTRETCH bits need to be configured before enabling peripheral.
        - [NOSTRETCH] - for configuration of clock stretching.
    - Enable PE bit for enabling I2C peripheral.
*/

/* 28.4.5 Software Reset
  - Previously configured registers in I2C_CR1 & I2C_TIMINGR are unaffected.
  - Impacted Register bits includes:
    - I2C_CR2: START/STOP/NACK
    - I2C_ISR: BUSY/TXE/TXIS/RXNE/ADDR/NACKF/TCR/TC/STOPF/BERR/ARLO/OVR.
  - Some SMBus register bits are affected too. Not relevant for our driver at the moment.
  - PE needs to be low for 3 APB clock cycles to perform reset.
    - Follow the sequence of Write PE=0. Check PE=0. Write PE=1.
*/

/* 28.4.6 Data Transfer
  - Managed via Tx/Rx Data Registers & a Shift Register.
  - Reception:
    - SDA input fills the shift register.
    - Upon 8th SCL pulse (when complete data byte is recieved), the shift register is copied to I2C_RXDR reg given RXNE = 0 (it is empty)
    - If RXNE=1, the previous data byte has not been read so SCL line is stretched low until I2C_RXDR is read.
      - Stretch is inserted between 8th/9th SCL pulse (before ACK pulse).

  - Transmission:
    - If I2C_TXDR reg is not empty (TXE=0 - i.e. contains data), its content is copied into the shift register after the 9th SCL pulse (ACK pulse).
    - Then the shift reg content is shifted out on SDA line.
    - If TXE=1, no data is written yet on I2C_TXDR, so SCL line is stretched low until I2C_TXDR is written. Stretch is done after 9th SCL pulse.

  - HW Transfer management:
    - The I2C has a byte counter embedded in HW to manage byte transfer & to close communication in the following modes including:
      - NACK/STOP & ReSTART generation in master mode.
      - ACK control when in slave reciever mode.
      - SMBUS related feature too.
    - Byte counter is always enabled in master mode. 
    - Byte counter is disabled in slave mode, but can be enabled via SBC (Slave Byte Control) in I2C_CR2 reg.
    - NumBytes to transfer can be set in NBYTES in I2C_CR2 reg. 
    - If more than 255 bytes needed, RELOAD bit must be selected in I2C_CR2.
      - In this mode, TCR flag is set when the NumBytes in NBYTES has been transferred, and an interrupt is generated if TCIE is set.
      - SCL is stretched as long as TCR flag is set. TCR is cleared by software when NBYTES is written to a non-zero value.
      - When NBYTES counter is reloaded with the last number of bits, the RELOAD bit must be cleared.

    - When RELOAD=0 in master mode, the counter can be used in 2 modes.
      - Automatic End mode (AUTOEND = 1 in I2C_CR2 reg). 
        - Master automatically sends a STOP condition after NBYTES have been transferred.
      - Software End Mode (AUTOEND = 0 in I2C_CR2 reg).
        - Softeare action is expected once NBYTES has been transferred, the TC flag is set & interrupt is generated if TCIE is set.
        - The SCL signal is stretched as long as TC flag is set.
        - The TC flag is cleared by software when START or STOP bit is set in I2C_CR2 register.
        - This mode must be used when master wants to send a RESTART condition.
*/

// 28.4.7 I2C Slave Mode init/Tx/Rx
/* I2C Slave Mode Initialization
  - I2C_OAR1 & I2C_OAR2 registers are used to program slave addresses OA1/OA2.
  - Configurable in 7 or 10 bit addressing via OA1MODE bit in I2C_OAR1 reg.
  - OA1 is enabled via OA1EN bit in I2C_OAR1 reg.
  - OA2 is for a 2nd slave address if needed - OA2MSK is used w/ address comparator to determine if 2nd address may be acknowledged.
  - OA2 can be enabled by setting OA2EN bit in I2C_OAR2 reg.
  - General Call address is enabled via GCEN bit in I2C_CR1 reg.
  - When the I2C is selected by one of its enabled address, the ADDR interrup status flag is set & interrupt is generated if ADDRIE is set.

  - By default, the slave uses clock stretching - which means SCL signal is stretched low as needed.
    - If the master doesnt support clock stretchin, I2C must be configured with NOSTRETCH=1 in I2C_CR1 reg.
  - After recieving an ADDR interrupt - if several addresses are enabled user read the ADDCODE[6:0] bits in I2C_ISR reg for address that matched.
    - DIR flag must also be checked to know transfer direction.

  - Slave Clock Stretching (NOSTRETCH = 0): SCL clock is stretched in following scenarios:
    - When ADDR flag is set - the recieved address matches with one of the enabled slave address.
      - This stretch is released when the ADDR flag is cleared by software by setting the ADDRCF bit.
    - In TX, if the previous data TX is completed and no new data is written to I2C_TXDR reg, or if the 1st data byte is not written when the ADDR flag is cleared (TXE=1).
      - This stretch is released when data is written to the I2C_TXDR reg.
    - In RX, when the I2C_RXDR reg is not read yet & a new data reception is completed.
      - This stretch is released when I2C_RXDR is read.
    - When TCR = 1 in Slave Byte Control, relaod mode (SBC = 1 / RELOAD = 1), meaning that last data byte has been TX.
      - This stretch is released when the TCR is cleared by writing a non-zero value in the NBYTES field.
    - After SCL falling edge detection, the I2C stretches SCL low during a certain duration (formula in DS).

  - Slave Clock without Stretching (NOSTRETCH = 1) Steps omitted.

  - Slave Byte Control Mode: Only compatible with Clock Stretching Enabled. Typically used for SMBus mode.
    - In order to allow byte ACK control in slave reception mode, Slave Byte Control must be enabled in SBC bit - I2C_CR1 reg. Required for SMBus.
    - RELOAD mode must be selected to allow byte ACK control in slave reception mode.
    - To get control of each byte, NBYTES must be initialized to 0x1 in the ADDR interrupt subroutine, and reloaded to 0x1 upon each recieved byte.
    - When Byte is received, the TCR bit is set, stretching the SCL signal low between 8th/9th SCL pulses.
    - The user can read the data from I2C_RXDR reg, then decided to ACK or NACK by configuring the ACK bit in the I2C_CR2 reg. 
    - SCL stretch is released by setting NBYTES to a non-zero value, the ACK or NACK is sent & next byte can be received.
    - NBYTES can be loaded with a value greater than 0x1, and in this case, reception flow is continuous during NBYTES data reception.

  - Slave Init Steps Summary:
    - I2C Initialization Steps
    - Clear OA1EN / OA2EM in I2C_OAR1 / I2C_OAR2.
    - Configure OA1, OA1MODE, OA1EN, OA2, OA2MSK, OA2EN, GCEN
    - Configure SBC in I2C_CR1 (For SMBus)
    - Enable interrupts and/or DMA in I2C_CR1.
*/

/* I2C Slave Transmitter
  - A Transmit interrupt status TXIS is generated when I2C_TXDR reg becomes empty.
  - An interrupt is generated if the TXIE bit is set in the I2C_CR1 reg.
  - TXIS bit is cleared when the I2X_TXDR is written with next data byte to TX.
  - Upon NACK receive, the NACKF bit is set in I2C_ISR register & an interrupt is generated if NACKIE bit is set in I2C_CR1 reg.
    - The slave automatically releases the SCL & SDA lines to let master perform a STOP or RESTART condition.
    - TXIS bit is not set upon NACK.
  - Upon STOP receive, if STOPIE bit is set in ISR_CR1 reg, the STOPF flag is set in I2C_ISR reg & an interrupt is generated.
    - In most cases, the SBC bit is set at `0`.
      - In this case, if TXE = 0 when the slave address is received (ADDR=1), the user either:
        - Send the content of I2C_TXDR reg as the first data byte OR
        - Flush the I2C_TXDR reg by setting TXE bit in order to program a new data byte.
    - If SBC = 1, the number of bytes to be TX must be programmed in NBYTES in the address match interrupt subroutine (ADDR=1). 
      - In this case, the number of TXIS events during the transfer corresponds to the value programmed in NBYTES.

  - Slave Clock without Stretching (NOSTRETCH = 1) Steps omitted. 

  - Figure 299 has Transfer sequence flowchart for I2C slave transmitter, NOSTRETCH=0.
*/

/* I2C Slave Receiver
  - RXNE is set in I2C_ISR when the I2C_RXDR is full & generates an interrupt if RXIE is set in I2C_CR1. 
  - RXNE is cleared when I2C_RXDR is read.
  - Upon STOP receive & if STOPIE is set in I2C_CR1, STOPF is set in I2C_ISR & an interrupt is generated.

  - Figure 302 has Transfer sequence flowchart for I2C slave receiver, NOSTRETCH=0.

*/

// 28.4.8 I2C Master Mode init/Tx/Rx
/* I2C Master Initialization - Clocking Information
  - Before enabling peripheral, I2C Master clock must be configured by setting SCLH/SCLL bits in the I2C_TIMINGR register. 
  - Clock Synchronization is implemented for multi-master & slave clock stretching. To allow Clock Synchronization:
    - The low level of the clock is counted using SCLL counter, starting from the SCL low level internal detection. Delay affected by Analog/Digital noise filters.
    - The high level of the clock is counted using the SCLH counter, starting from the SCL high level internal detection. Delay affected by Analog/Digital noise filters.
  - Formula for master clock period is here. Figure 305 shows more information.
*/

/* I2C Master Communication Initialization - Address Phase
  - To initiate the communication - user must program the following params for the addressed slave in the I2C_CR2 register:
    - Adressing mode (7/10 bit): ADD10.
    - Slave address to be sent: SADD.
    - Transfer direction: RD_WRN
    - In case of 10 bit address read: HEAD10R bit.
      - HEAD10R must be configured to indicate if the complete address sequence must be sent, or only the header in case of direction change.
      - The number of bytes to TX: NBYTES. If the number of bytes is >= 255, NBYTES must be filled with 0xFF.
    
    - The user must then set the START bit in the I2C_CR2 register. Changing all the above bits is not allowed when START bit is set.
      - Then the master automatically send the START condition folloed by the slave address as soon as it detects the bus is free (BUSY=0) & after delay of tBUF.
      - In case of an arbitration loss, the master switches back to slave mode & can acknowledge its own address if it is addressed as a slave.
    
    - START bit is reset by HW when the slave address has been sent on the bus regardless of received ACK/NACK.
    - START bit is reset by HW if an arbitration loss occurs.

    - In 10 bit addressing:
      - When the Slave Address first 7 bits are NACK'd by the slave, the master will re-launch automatically the slave address transmit until ACK received.
      - In this case, ADDRCF must be set if a NACK is received from the slave in order to stop sending the slave address.

    - If I2C is addressed as a slave (ADDR=1) while the START bit is set, the I2C switches to slave mode & the START bit is cleared when ADDRCF bit is set.
      - The same procedure applies for a Repeated Start condition - in this case BUSY=1.
    
    - Master Init steps summary:
      - Initial settings
      - Enable interrupts/DMA in I2C_CR1.
*/

/* Initialization of a master receiver addressing a 10-bit address slave. [Figure 307-308]
  - If the slave address is in 10-bit format, user can choose to send the complete read sequence by clearing the HEAD10R bit in the I2C_CR2 register.
    - In this case the master automatically send the follwoing completed sequence after the START bit is set:
      - (Re)START + Slave address 10 bit header Write + Slave address 2nd byte + ReSTART + Slave address 10 bit header Read.
    
    - If the master addresses a 10-bit address slave, transmits data to this slave, then reads data from the same slave, a master transmission flow must be done first.
      - Then, a repeated start is set with the 10bit slave address configured with HEAD10R=1.
      - In this case master sends this sequence:
        - ReSTART + Slave address 10 bit header Read.
*/

/* I2C Master Transmitter [Figure 309-311]
  - In the case of a write transfer, the TXIS flag is set after each byte transmission, after the 9th SCL pulse when an ACK is received.
  - A TXIS event generates an interrupt if the TXIE bit is set in the I2C_CR1 register.
    - The flag is cleared when the I2C_TXDR register is written with the next data byte to be transmitted.
  - The number of TXIS events during the transfer corresponds to the value in NBYTES. 
    - If the total number of data bytes to be sent is greater than 255, reload mode must be selected via RELOAD bit in I2C_CR2 register.
      - In this case, when NBYTES has been TX, the TCR flag is set & the SCL line is stretched low until NBYTES is written to a non-zero value.

  - The TXIS flag is not set when a NACK is received.
    - When RELOAD=0 & NBYTES has been TX:
      - In automatic end mode (AUTOEND=1), a STOP is automatically sent.
      - In software end mode (AUTOEND=0), the TC flag is set and the SCAL line is stretched low in order to perform software actions:
        - A RESTART condition can be requested by setting the Start bit in the I2C_CR2 register with the proper slave address configuration & number of bytes to TX.
          - Setting the START bit clears the TC flag & the START condition is sent on the bus.
        - A STOP condition can be requested by setting the STOP bit in the I2C_CR2 register. 
          - Setting the STOP bit clears the TC flag & the STOP confition is sent on the bus.
    - If a NACK is recieved: TXIS flag is not set & a STOP condition is automatically sent upon NACK reception.
      - The NACKF flag is set in the I2C_ISR register & an interrupt is generated if the NACKIE bit is set.
*/

/* I2C Master Receiver [Figure 312-314]
  - In the case of a read transfer, the RXNE flag is set after each byte reception, after the 8th SCL pulse.
  - An RXNE event generates an interrupt if the RXIE bit is set in the I2C_CR1 reg.
    - This flag is cleared when I2C_RXDR is read.
  - If the total number of data bytes to be received is greater than 255, RELOAD bit must be set in I2C_CR2 reg.
    - In this case, when NBYTES data has been TX, the TCR flag is set & the SCL line is stretched low until NBYTES is written to a non-zero value.

  - When RELOAD=0 & NBYTES data has been transferred:
    - In automatic end mode (AUTOEND=1), a NACK & a STOP are automatically sent after the last receieved byte.
    - In software end mode (AUTOEND=0), a NACK is automatically sent after the last receieved byte, the TX flag is set & SCL line is stretched low to allow for SW actions:
      - A RESTART condition can be requested by setting the START bit in the I2C_CR2 register with proper slave address config & number of bytes to TX.
        - Setting the Start bit clears the TC flag & the START condition, followed by slave address are sent on the bus.
      - A STOP condition can be request by setting the STOP bit in the I2C_CR2 register. 
        - Setting the STOP bit clears the TC flag & the STOP condition is sent on the bus.
*/

/* 28.4.14 Wakeup from Stop mode on address match
  - I2C is able to wakeup the MCU from STOP mode (APB clock off), when it is addressed. (all addressing modes supported)
  - Wakeup from Stop mode is enabled by setting WUPEN bit in I2C_CR1 reg.
    - HSI oscillator must be clock source for I2CCLK to allow wakup from Stop mode.
  - During Stop mode, HSI is switched off. 
    - When START is detected, the I2C interface switches the HSI on & stretches the SCL low until HSI is woken up.
    - HSI is then used for address reception.
  - In case of an address match, the I2C stretches SCL low during MCU wakeup time.
    - The stretch is released when ADDR flag is cleared by SW, then transfer goes on normally.
  - If address does not match, HSI is switched off & MCU is not woken up.

  - Only an ADDR interrupt can wakeup MCU.
    - Do not enter STOP mode when the I2C is performing a transfer as master or as an addressed slave after ADDR flag is set.
      - This can be managed by clearing SLEEPDEP bit in the ADDR interrupt routine & setting it again only after STOPF flag is set.

  - Some Cautions in this section. Summary:
    - Cannot use digital filter, HSI oscillator must be I2CCLK, Clock stretching must be enabled.
    - If WUPEN is disabled, I2C peripheral must be disabled before entering Stop mode.

*/

/* 28.4.15 Error Conditions
  - Bus Error (BERR):
    - Detected when a START or STOP confition is detected & is not located after a multiple of 9 SCL clock pulses.
    - A START or STOP condition is detected when a SDA edge occurs while SCL is high.
    - The bus error flag is set only if the I2C involved in the transfer as master or addressed slave (not during the address phase in slave mode.)
    - In case of a misplaced START or RESTART detection in slave mode, I2C enteres address recognition state like for a correct START condition.
    - When a bus error is detect, the BERR flag is set in the I2C_ISR reg, and an interrupt is generated if the ERRIE bit is set in the I2C_CR1 reg.

  - Arbitration Lost (ARLO):
    - Detected when a high level is sent on SDA line, but a low level is sampled on the SCL rising edge.
      - In master mode, arbitration los is detected during address phase, data phase & data ACK phase.
        - In this case, SDA/SCL lines are released, the START control bit is cleared by HW & the master switches automatically to slave mode.
      - In slave mode, arbitration loss is detected during data phase & data ACK phase. 
        - In this case, the transfer is stopped & SDA/SCL lines are released.
      - When an arbitration loss is detected, the ARLO flag is set in the I2C_ISR register & an interrupt is generated if the ERRIE bit is set in the I2C_CR1 reg.

  - Overrun/Underrun Error (OVR):
    - Detected in slave mode when NOSTRETCH=1 (Clock Stretching Disabled).
      - Omitting this as we will have Clock stretching enabled.
    
  - Packet Error Checking Error (PECERR)
    - For SMBUS

  - Timeout Error (TIMEOUT)
    - For SMBUS

  - Alter
    - For SMBUS
*/

LabI2C :: LabI2C(LabI2C_InitStruct* LabI2C_struct) {
    I2C_init.I2C_port = LabI2C_struct->I2C_port;
    
    // APBEN for I2C1...
    RCC->APB1ENR |= (1 << 21); // Enable I2C1EN in APB1ENR.

    // Begin Init. Assume we are on I2C1.
    // Disable Analog & Digital Noise Filters...
    I2C_init.I2C_port->CR1 &= ~(1 << 12); // ANFOFF --> Set to `0`.
    I2C_init.I2C_port->CR1 &= ~(15 << 8); // DNF[3:0] --> Set to `0000`

    // Setup PRESC / SCLDEL / SDADEL.. May need to use STM32CubeMX for this.
    I2C_init.I2C_port->TIMINGR |= 0x00100000; // This was pulled from STM32 HAL layer operating at 100 KHz..
                                              // Only configuring bits 31:16 here. 15:0 will be done in Master init.
    
}

uint8_t LabI2C :: init_master(){
    master0slave1 = 0;

    // Setup SCLH / SCLL based on values taken from STM32 HAL layer operating at 100 KHz..
    I2C_init.I2C_port->TIMINGR |= 0x00001D2D;

    // Enable PE bit.
    I2C_init.I2C_port->CR1 |= (1 << 0); // PE --> Set to `1`.
}

// Master routines - Assuming for now that nbytes <= 255. If > 255, will need to program more logic. Worth investigating.
uint8_t LabI2C :: master_write(uint8_t nbytes, uint8_t slave_addr, uint8_t* data_buff, bool write_then_read){
    // Setup NBYTES:
	I2C_init.I2C_port->CR2 &= ~(0xFF << 16);    // Clearing out this data first...
    I2C_init.I2C_port->CR2 |=  (nbytes << 16);  // NBYTES[7:0] = bytes.

    // Setup RELOAD. Setting to 0.
    I2C_init.I2C_port->CR2 &= ~(1 << 24);

    // [3-30-22] Explicitly setting AUTOEND = 0. Want to control STOP bit myself.
    I2C_init.I2C_port->CR2 &= ~(1 << 25);

    // Setup 7-bit address mode.
    I2C_init.I2C_port->CR2 &= ~(1 << 11); // ADD10 --> Set to `0`.

    // Setup slave address to communicate with. Assume 7 bit for now.
    I2C_init.I2C_port->CR2 &= ~(0x7F << 1); // Clearing out this data first...
    I2C_init.I2C_port->CR2 |= (slave_addr << 1); // SADD[7:1] = slave_addr.

    // Setup that Master requests a write.
    I2C_init.I2C_port->CR2 &= ~(1 << 10); // RD_WRN --> Set to `0` to indicate write transfer.

    // Enable interrupts? Probably take a look at I2C_ISR. I2C_ICR may need to be used in interrupt handler as interrupts fire??

    // Send start bit.
    I2C_init.I2C_port->CR2 |= (1 << 13); // START --> Set to `1` to generate START condition.

    // Entry point into state machine.
    // while (i2cMasterStateMachine());

   
    // Lets attempt to follow the flowchart in data sheet here. Lets assume that we have less than 255 bytes to send.
    int i = 0;
    while (1) {
        // If NACK detected, return here.
        if (I2C_init.I2C_port->ISR & (1 << 4)) {
            return 0xF0;
        }

        // If TXIS detected & i <= nbytes, lets write to TxDR.
        // nbytes will be +1 from iterator val... Keep this in mind.
        if (I2C_init.I2C_port->ISR & (1 << 1) && i <= nbytes) {
            // Write the first bit of data from data buff & increment iterator.
            I2C_init.I2C_port->TXDR = data_buff[i++];
        }

        // TC=1 indicates that Transmit is complete - NBYTES have been sent out.
        else if (I2C_init.I2C_port->ISR & (1 << 6) && i == nbytes) {
        	// If we are at this portion of the loop, a subsequent START condition from next transfer will clear our TC ISR.
            if (write_then_read) {
                // We will be doing a read shortly DO NOT WANT TO SEND STOP BIT. 
                // Let `master_read` send another START (ReSTART), allow slave to drive SDA, then we send STOP as master..
                // Realistically NBYTES should be 1 if this happens. May want to check this earlier in function
                return 0xF1; // Indicates we are doing a write_then_read.
            } else {
                // We are done, not expecting write then read. Lets send STOP condition.
                I2C_init.I2C_port->CR2 |= (1 << 14);
            }
        }

        // If we detect a STOPF condition, return here.
        else if (I2C_init.I2C_port->ISR & (1 << 5)) {
            // Clear STOP bit.
            I2C_init.I2C_port->ICR |= (1 << 5); // Need to use ICR for this - Interrupt Clear Register.
            return 0xF2;

        // Go back to top of while loop.
        } else {

        }
    }
    return 0xFF;
    
}

uint8_t LabI2C :: master_read(uint8_t nbytes, uint8_t slave_addr, uint8_t* data_buff){
    // Setup NBYTES:
    I2C_init.I2C_port->CR2 |= (nbytes << 16); // NBYTES[7:0] = bytes.

    // Setup RELOAD. Setting to 0.
    I2C_init.I2C_port->CR2 &= ~(1 << 24);

    // [3-30-22] Explicitly setting AUTOEND = 0. Want to control STOP bit myself.
    I2C_init.I2C_port->CR2 &= ~(1 << 25);

    // Setup 7-bit address mode.
    I2C_init.I2C_port->CR2 &= ~(1 << 11); // ADD10 --> Set to `0`.

    // Setup slave address to communicate with. Assume 7 bit for now.
    I2C_init.I2C_port->CR2 |= (slave_addr << 1); // SADD[7:1] = slave_addr.

    // Setup that Master requests a read.
    I2C_init.I2C_port->CR2 |= (1 << 10); // RD_WRN --> Set to `1` to indicate read transfer.

    // Enable interrupts? Probably take a look at I2C_ISR. I2C_ICR may need to be used in interrupt handler as interrupts fire??

    // Send start bit.
    I2C_init.I2C_port->CR2 |= (1 << 13); // START --> Set to `1` to generate START condition. 
                                         // This will be a ReSTART in the case of write_then_read when master_write is done first.

    int i = 0;
    while (1) {
        // If RXNE == 1, we have data we can recieve from RXDR.
        if (I2C_init.I2C_port->ISR & (1 << 2) && i <= nbytes) {
        	// Recieve data from RXDR.
            data_buff[i++] = I2C_init.I2C_port->RXDR;
        }

        // TC = 1 && i == nbytes implies that all data has been read.
        else if (I2C_init.I2C_port->ISR & (1 << 6) && i == nbytes) {
        	// If we are at this portion of the loop, a subsequent START condition from next transfer will clear our TC ISR.
            // Send out STOP bit.
            I2C_init.I2C_port->CR2 |= (1 << 14);
        }

        // If we detect a STOPF condition, return here.
        else if (I2C_init.I2C_port->ISR & (1 << 5)) {
            // Clear STOP bit.
        	I2C_init.I2C_port->ICR |= (1 << 5); // Need to use ICR for this - Interrupt Clear Register.
            return 0xF0;
        }

        // Go back to top of while loop.
        else {

        }

    }
    return 0xFF;
}

uint8_t LabI2C :: init_slave(uint8_t slave_addr) { // Need a param here to configure our slave address for I2C peripheral when init'ing slave.
    char local_buff[100]; // Local buff to be used for printing.

    master0slave1 = 1;
    
    // Setup NOSTRETCH. We want to enable clock stretching.
    I2C_init.I2C_port->CR1  &= ~(1 << 17); // NOSTRETCH --> Set to `0`.

    // Lets clear out OA1EN / OA2EN.
    I2C_init.I2C_port->OAR1 &= ~(1 << 15); // OA1EN --> Set to 0.
    I2C_init.I2C_port->OAR2 &= ~(1 << 15); // OA1EN --> Set to 0.

    // Lets configure just OA1 address information at this point. Not using OAR2.
    I2C_init.I2C_port->OAR1 &= ~(0x7F << 1); // Lets clear out the current info in OA1 so we can reprogram it in init routine.
    I2C_init.I2C_port->OAR1 |=  (slave_addr << 1); // Setup OA1[7:1] as our slave_addr. Programmable upon init. 
    I2C_init.I2C_port->OAR1 &= ~(1 << 10); // OA1MODE --> Set to `0`. 7 bit addresses.
    I2C_init.I2C_port->OAR1 |=  (1 << 15); // OA1EN --> Set to 1. Enable Slave Address OA1. Must be done after init above.
    
    // Lets Disable General Call Enable. This is likely useful only in case where there are multiple slaves on I2C line.
    I2C_init.I2C_port->CR1  &= ~(1 << 19); // GCEN --> Set to `0`. Address 0b0000000(000) is NACK'd.

    // At this point we could modify OA2 reg... But I just want this addressable via programmed slave_addr by user.
    // Perhaps this can be used as a register address to modify within us as a slave device? may be good to look into this.

    // Lets attempt to Enable Slave Byte Control Mode. May want to review last portion of section `28.4.7` for Slave Byte Control info.
    // We can disable this is it become tough to program against. Apparently this is needed for SMBus so it may not be needed for I2C slave. 
    // I2C_init.I2C_port->CR1  |=  (1 << 16); // Enable SBC - used in Slave Rx mode.
    // I2C_init.I2C_port->CR2  |=  (1 << 24); // Enable Reload. Tx is not compate after NBYTES data Tx (NBYTES Will be reloaded).
    //                                        // TCR flag is set when NBYTES data are Tx, stretching SCL low.
    
    // NOTE: We may opt to not use this for ease of driver creation... Lets see if this is a hassle.
    // Lets Disable Slave Byte Control Mode just for ease of driver creation.
    I2C_init.I2C_port->CR1  &= ~(1 << 16);  // Disable SBC - used in Slave Rx mode.

    // Enable PE bit.
    I2C_init.I2C_port->CR1 |= (1 << 0); // PE --> Set to `1`.

    // Enable interrupts... Should this be done in our state machine? Whats the best way to do this...

    // [4-4-22]: Lets initialize a data_buf to some value. Currently it will just be a variable. We can basically just read or write to this variable (for now).
    // Based on I2C lab, perhaps it might be a good idea to mock up an array that we can read/write to via I2C from certain indexes in our "memory".
    // I am also not sure how multiple byte read/write will be handled. We will need to re-read data sheet & examine what happens with current code.
    uint8_t data_buf = 0xFF;

    // Hm.. Upon looking at our data sheet, we might be able to just call `init_slave` & sit in a while loop waiting for commands..?
    // I am currently not sure if it makes sense to create a slave_write / slave_read routine.
    // Lets try adding a while loop here & implementing logic here for NOSTRETCH = 0 / Tx/Rx modes.

    // Entry point into `state machine`.
    while (1) { 
        // If we detect that Slave Address has matched:
        if (I2C_init.I2C_port->ISR & (1 << 3)) {
            // Lets check for ADDCODE & DIR
            uint8_t addcode = (I2C_init.I2C_port->ISR & (0x7F << 17)) >> 17; // Read what ADDCODE is, shift right by 1 to keep consitent with slave_addr.
            uint8_t dir     = (I2C_init.I2C_port->ISR & (1 << 16)) >> 16;    // Read what DIR is, shift right by 16 to just have a 0/1.

            // dir = 0 --> Write transfer, slave enters reciever mode.
            if (dir == 0) {
                // Lets compare addcode to slave_addr. Should be the same.
                if (addcode == slave_addr) {
                    // Lets set ADDRCF.
                    I2C_init.I2C_port->ICR |= (1 << 3); // ADDRCF --> Set to `1`. Clears ADDR flag in I2C_ISR reg & START bit in I2C_CR2 reg.

                    // Poll until RXNE == 1, then we can read data. Will be cleared upon reading RXDR.
                    while (!(I2C_init.I2C_port->ISR & (1 << 2)));
                    data_buf = I2C_init.I2C_port->RXDR;

                    memset(local_buff, '0', 100);
                    sprintf((char*)local_buff, "Write Transfer - Slave RX mode. data_buf: 0x%02x\r\n", data_buf);
                    HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 100);

                } else {
                    return 0xF1; // Shouldnt happen..
                }
            }
            
            // dir = 1 --> Read transfer, slave enters transmitter mode.
            else if (dir == 1) {
                // Flush out data in I2C_TXDR before transmit.
                I2C_init.I2C_port->ISR |= (1 << 0); // Set TXE to `1` to flush out I2C_TXDR.

                // Lets compare addcode to slave_addr. Should be the same.
                if (addcode == slave_addr) {
                    // Lets set ADDRCF.
                    I2C_init.I2C_port->ICR |= (1 << 3); // ADDRCF --> Set to `1`. Clears ADDR flag in I2C_ISR reg & START bit in I2C_CR2 reg.
                    
                    // Poll until TXIS == 1, then we can write data. Will be cleared upon a write to TXDR.
                    while (!(I2C_init.I2C_port->ISR & (1 << 1)));
                    I2C_init.I2C_port->TXDR = data_buf;

                    memset(local_buff, '0', 100);
                    sprintf((char*)local_buff, "Read Transfer - Slave TX mode. TXDR: 0x%02x / data_buf: 0x%02x\r\n", I2C_init.I2C_port->TXDR, data_buf);
                    HAL_UART_Transmit(&huart1, (uint8_t*)local_buff, strlen(local_buff), 100);

                } else {
                    return 0xF2; // Shouldnt happen..
                }    
            }
        }
    }
}

// Slave routines: [4-3-22] Commenting for now.
/*
uint8_t LabI2C :: slave_write(uint8_t data){
    // Lets attempt to follow the flow chart - assuming SBC = 0.
    while(1) {
    }
}

uint8_t LabI2C :: slave_read(){
    // Lets attempt to follow the flow chart - assuming SBC = 0.
    while(1) {
    }
}
*/

// Functions that effectively go through a state machine.
// [3-26-22]: Attempt to use a switch case if it is helpful rather than if/else constantly.
// May need to obtain a semaphore from our ISR as we go through our states? Not sure yet. Lets think it over.
i2c_state LabI2C :: i2cMasterStateMachine() {
    // enum {
    //     // General States & Master Specific States.

    // }; 

    i2c_state master_current_state = BUSY_STATE;

    // switch () { // Switch on data in register... 
    //     case enum_val:
            
    //         break;
    //     case enum_val2:
            
    //         break;

    //     default:
    //         break;

    return master_current_state;
}

i2c_state LabI2C :: i2cSlaveStateMachine() {

}
