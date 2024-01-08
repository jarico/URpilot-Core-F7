################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Sistema/system_stm32f7xx.c 

OBJS += \
./Core/Sistema/system_stm32f7xx.o 

C_DEPS += \
./Core/Sistema/system_stm32f7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Sistema/%.o Core/Sistema/%.su: ../Core/Sistema/%.c Core/Sistema/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Sistema

clean-Core-2f-Sistema:
	-$(RM) ./Core/Sistema/system_stm32f7xx.d ./Core/Sistema/system_stm32f7xx.o ./Core/Sistema/system_stm32f7xx.su

.PHONY: clean-Core-2f-Sistema

