################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/ili9328/ili9328.c 

C_DEPS += \
./Drivers/BSP/Components/ili9328/ili9328.d 

OBJS += \
./Drivers/BSP/Components/ili9328/ili9328.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/ili9328/%.o: ../Drivers/BSP/Components/ili9328/%.c Drivers/BSP/Components/ili9328/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"/home/ajaycuram/Desktop/STM32_DEV/workspace/FreeRTOS_UART_test_v1/Drivers/BSP/STM32F3-Discovery" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-ili9328

clean-Drivers-2f-BSP-2f-Components-2f-ili9328:
	-$(RM) ./Drivers/BSP/Components/ili9328/ili9328.d ./Drivers/BSP/Components/ili9328/ili9328.o

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-ili9328

