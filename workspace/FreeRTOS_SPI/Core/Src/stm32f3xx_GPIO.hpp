/* Define to prevent recursive inclusion -------------------------------------*/
// AJ - Define this class only once even if there are multiple #define __STM32F3XX_GPIO_H scattered throughout code.
#ifndef __STM32F3XX_GPIO_H
#define __STM32F3XX_GPIO_H

#include "stm32f3xx_hal.h"
#include "stm32f3_discovery.h"
#include "stdint.h"
#include "stm32f303xc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{ // This structure is analogous to `GPIO_InitTypeDef` - `stm32f3xx_hal_gpio.h`. This is my HAL layer for GPIO initialization.
    GPIO_TypeDef* GPIO_port;     // GPIO port to configure pin for.
    int pin            = 0;      // pin to configure (0-16.)
    uint16_t bsrr      = 0;      // bsrr offset to use to set a pin HI/LO. Will contain a MASK to be used to set value of GPIO_port->BSRR.
    uint32_t mode      = 0;      // operation mode - input/output/alternate/analog modes
    uint16_t output    = 0;      // output type register - push-pull or pull-up
    uint32_t pupd      = 0;      // pull-up/pull-down/push-pull/open-drain
    uint32_t speed     = 0;      // operating speed
    uint32_t alternate = 0;      // alternate mode select
}LabGPIO_InitStruct;
// AJ - For accessing GPIO_TypeDef structures, we likely can use `GPIOA` - `GPIOF` directly from stm32f303xc.h.
// To begin with, we want to access the LED pins that are on port `GPIOE`. May want to manually use this port.

class LabGPIO_X
{
    private:
      LabGPIO_InitStruct GPIO_init; // private class variable for LabGPIO_InitStruct. 
                                    // this will contain everything peeled off from GPIO_struct passed into constructor for LabGPIO_X.

    public:
      //We will pass in GPIOE for the GPIO_Port param to indicate we are operating on PE pins where LEDs are connected.
      //Using GPIO_struct, we will manually setup how we want our pin on `pin#1/2/3` to behave.
      LabGPIO_X(LabGPIO_InitStruct* GPIO_struct); // Will be analogous to `HAL_GPIO_Init` - `smt32f3_discovery.c`

      void setAsInput();    // private?
      void setAsOutput();   // private?
      void setAsAlternate(uint8_t AF); 
      void setAs(uint8_t bit); // for use in our simple UART_GPIO driver.
      void setDirection(bool output);

      void setHigh(); // private?
      void setLow(); // private?
      void setLevel(bool high);

      uint8_t getDirection();
      bool getLevel();

//      ~LabGPIO_X();
};

#ifdef __cplusplus
}
#endif

#endif /* __STM32F3XX_GPIO_H */
