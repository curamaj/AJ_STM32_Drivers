# AJ_STM32_Dev
Ajay's STM32 Drivers for GPIO/UART/SPI/I2C using C/C++ &amp; FreeRTOS.

This codebase is targetting the STM32F303VC Discovery board.
  - STM32 Discovery Development board: https://www.st.com/en/evaluation-tools/stm32f3discovery.html#documentation
  - STM32F303VCT6 Reference Manual is here: https://www.st.com/en/microcontrollers-microprocessors/stm32f303vc.html#documentation
    - Direct Link to Reference Manual: https://www.st.com/resource/en/reference_manual/rm0316-stm32f303xbcde-stm32f303x68-stm32f328x8-stm32f358xc-stm32f398xe-advanced-armbased-mcus-stmicroelectronics.pdf 

These are test programs I created so I could re-learn device driver programming & coding against a datasheet.

Project code for each of the folders at this repo are primarily located under the following directory structure:
  - FreeRTOS_`xyz`/
    - sample waveforms taken via Saleae Logic Analyzer will be attached here for each product.
    - likely will be a combo of saleae logic 2 `.sal` files + images + text output.
  - FreeRTOS_`xyz`/Core/Inc/
    - main.hpp: unchanged.
    - FreeRTOSConfig.h: slightly changed to increase stack size while using FreeRTOS.
  - FreeRTOS_`xyz`/Core/Src/
    - main.cpp: per-project main.cpp with test code to test out drivers.
    - stm32fxx_<GPIO/Interrupt/UART/SPI/I2C>.cpp/.hpp: driver code.

This code-base is still a WIP & I will be updating it as time permits, so it may be a little rough around the edges ;)

Thank you for taking the time to look at this repo :)
