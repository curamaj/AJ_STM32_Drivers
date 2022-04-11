#include "stm32f3xx_GPIO.hpp"
#include "stm32f3xx.h" // Need this to include `stm32f303xc.h` for the `GPIO_TypeDef` we will be setting manually in our class here...

//AJ - Defines go here. 

// Defines below are for reading the GPIOx_MODER register & determining direction.
#define DIRECTION_INPUT     0
#define DIRECTION_OUTPUT    1
#define DIRECTION_ALTERNATE 2
#define DIRECTION_ANALOG    3


/*AJ - Notes
  - 4x 32-bit configuration registers
    - GPIOx_MODER
    - GPIOx_OTYPER
    - GPIOx_OSPEEDR
    - GPIOx_PUPDR

  - 2x 32-bit   data registers
    - GPIOx_IDR - Input Data Register
    - GPIOx_ODR - Output Data Register

  - 1x 32-bit set/reset register
    - GPIOx_BSRR

  - 1x 32-bit locking register 
    - GPIOx_LCKR

  - 2x 32-bit alternate function selection registers
    - GPIOx_AFRH
    - GPIOx_AFRL

  - [12-27-21] Trying to understand the layout of the memory map a bit & structures used in HAL ...
    - `GPIO_TypeDef`: stm32f303xc.h
      - It seems that in stm32f303xc.h there are defines for `GPIOA` - `GPIOF`, all of which point to
      a place in memory where we can directly access the GPIO configuration registers as defined by
      the `GPIO_TypeDef` & the datasheet. 
      - The uint32_t's defined in `GPIO_TypeDef` also are volatile
      as they are not to be optimized by the compiler, they are to be specific offsets in the memory
      map corresponding to registers that can be modified for GPIO configuration desired.
    - `GPIO_InitTypeDef`: stm32f3xx_hal_gpio.h
      - This HAL layer struct allows up to pass information to the device driver in a way such that 
      rgisters do not need to be configured manually by the user. This will be implemented by me.
      - 
*/

// AJ - I will be following a similar convention to `GPIO_InitTypeDef` - `stm32f3xx_hal_gpio.h` for `LabGPIO_InitStruct`.
// Initialization method for GPIO port/struct, will be analogous to `HAL_GPIO_Init` - `smt32f3_discovery.c`.
// [12-27-21]: For now, this constructor will really only apply to the LED as configured by the LabGPIO_InitStruct passed.
LabGPIO_X::LabGPIO_X(LabGPIO_InitStruct* GPIO_struct) {
    // must use setAsInput / setAsOutput for `GPIO_init.mode` to be setup.
    GPIO_init.GPIO_port  = GPIO_struct->GPIO_port;
    GPIO_init.pin        = GPIO_struct->pin;
    GPIO_init.pupd       = GPIO_struct->pupd;
    GPIO_init.speed      = GPIO_struct->speed;
    GPIO_init.alternate  = GPIO_struct->alternate;
    GPIO_init.output     = GPIO_struct->output;

// Init BSRR Mask & set it up based on pin passed.
    GPIO_init.bsrr       |= (1 << GPIO_init.pin);

    GPIO_init.GPIO_port->OTYPER  |= GPIO_init.output;
    GPIO_init.GPIO_port->OSPEEDR |= GPIO_init.speed;
    GPIO_init.GPIO_port->PUPDR   |= GPIO_init.pupd;

}

/*These functions likely will operate on the GPIO_port->MODER register to configure as input/output.
  Based on what I am seeing in `BSP_LED_INIT` - `stm32f3discovery.c`, for an LED I will need to use Push-Pull for MODER & Pull-Up for PUPDR/OTYPER. Speed can be high..
  There are input/output registers GPIOx_IDR & GPIOx_ODR. 
  To set GPIOx_ODR to be set/reset, one should use the control bits in GPIOx_BSRR. There is a BS(i) & BR(i) - Bit Set / Bit Reset.
  GPIOx_BSRR allows for atomic operation for bitwise handling (so if we are interrupted by ISR elsewhere, the operation is not affected.)
  GPIOx_LCKR register can be used to freeze MODER/OTYPER/OSPEEDR/PUPDR/AFR[Hi/Lo].
  (Input-ToDo, look at 11.3.9): GPIOs that are marked as input (switch) can use external interrupt lines to service an ISR.
*/
void LabGPIO_X::setAsInput() {
    uint32_t pin = GPIO_init.pin;
    uint32_t mask = ~(3 << (pin*2));
    GPIO_init.GPIO_port->MODER &= mask;
}

void LabGPIO_X::setAsOutput() {
    uint32_t pin = GPIO_init.pin;
    GPIO_init.GPIO_port->MODER &= ~(1 << ((pin*2)+1) ); //MODERy[1] to a `0`
    GPIO_init.GPIO_port->MODER |= (1 << (pin*2));       //MODERy[0] to a `1`
}

// Add on [2_13_22] lets allow for alternate functions to be configured.
// Refer to `stm32f303vc.pdf` page 43 onwards for mappings per port/pin.
void LabGPIO_X::setAsAlternate(uint8_t AF) { // AF should be specified as a 0x0 - 0xF value. may be able to do 0b0011...
    // We will need to rely on current port/pin to set the AF value correctly.
    // Considering we already know what port we are on, its the pin that matter now.
    uint32_t pin = GPIO_init.pin;

    // Work on MODER register. Setup `10` at MODER0[1:0] to MODER15[1:0] based on pin for offset.
    GPIO_init.GPIO_port->MODER |=  (1 << ((pin*2)+1) ); //MODERy[1] to a `1`
    GPIO_init.GPIO_port->MODER &= ~(1 << pin*2);        //MODERy[0] to a `0`

    // Work on AFR registers
    if (pin >= 0 && pin <= 7) {
        //write data to GPIOx_AFRL
        GPIO_init.GPIO_port->AFR[0] |= (AF << pin*4);
    } else if (pin >= 8 && pin <= 15) {
        pin -= 8; // go back to using 0-7 for 8-15.
        //write data to GPIOx_AFRH
        GPIO_init.GPIO_port->AFR[1] |= (AF << pin*4);
    }
}

void LabGPIO_X::setDirection(bool output) {
    
}

void LabGPIO_X::setHigh() {
//    uint32_t temp = GPIO_init.bsrr;
//    GPIO_init.GPIO_port->BSRR |= temp;
    GPIO_init.GPIO_port->BSRR |= GPIO_init.bsrr;
}

void LabGPIO_X::setLow() {
//    uint32_t temp = GPIO_init.bsrr << 16;
//    GPIO_init.GPIO_port->BSRR |= temp;
	GPIO_init.GPIO_port->BSRR |= GPIO_init.bsrr << 16;
}

void LabGPIO_X::setAs(uint8_t bit) {
    if (bit == 0) {
        this->setLow();
    } else if (bit == 1) {
        this->setHigh();
    }
}

void LabGPIO_X::setLevel(bool high) {
    
}

uint8_t LabGPIO_X::getDirection() {
    uint8_t mask = 0x03;
    uint16_t pin = GPIO_init.pin;
    uint8_t  direction = (GPIO_init.GPIO_port->MODER) >> (pin*2);
    direction = direction & mask;
    return direction;//AJ - Placeholder
}

bool LabGPIO_X::getLevel() {
    bool level;
    if (this->getDirection() == DIRECTION_OUTPUT) { // Read ODR for output data level
        uint16_t mask = 0x0001;
        uint16_t pin = GPIO_init.pin;
        level = ((GPIO_init.GPIO_port->ODR) >> pin) & mask;
    } 
    else if (this->getDirection() == DIRECTION_INPUT) { // Read IDR for input data level
        uint16_t mask = 0x0001;
        uint16_t pin = GPIO_init.pin;
        level = ((GPIO_init.GPIO_port->IDR) >> pin) & mask;
    }
    return level;
}

//LabGPIO_X::~LabGPIO_X() { // destructor
//    delete this;
//}

