/* Define to prevent recursive inclusion -------------------------------------*/
// AJ - Define this class only once even if there are multiple #define __STM32F3XX_GPIO_H scattered throughout code.
#ifndef __STM32F3XX_INTERRUPT_H
#define __STM32F3XX_INTERRUPT_H

#include "stm32f3xx_hal.h"
#include "stm32f3_discovery.h"
#include "stdint.h"
#include "stm32f303xc.h"

#ifdef __cplusplus
extern "C" {
#endif

    enum class Edge { // Will need to number these based on how bits are to be configured per edge config.
        None,
        Rising,
        Falling,
        Both          
    };

    void attachInterruptHandler(void* func, Edge edge);
    void enableInterrupts(uint32_t line);

// AJ - Not sure I need a class for this.
/*
class LabInterrupt
{
    private:

    public:
        // 2-20-22 Add On's - Interrupt Support
}
*/

#ifdef __cplusplus
}
#endif

#endif /* __STM32F3XX_GPIO_H */
