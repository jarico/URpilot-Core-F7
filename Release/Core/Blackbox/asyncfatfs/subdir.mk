################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Blackbox/asyncfatfs/asyncfatfs.c \
../Core/Blackbox/asyncfatfs/fat_standard.c 

OBJS += \
./Core/Blackbox/asyncfatfs/asyncfatfs.o \
./Core/Blackbox/asyncfatfs/fat_standard.o 

C_DEPS += \
./Core/Blackbox/asyncfatfs/asyncfatfs.d \
./Core/Blackbox/asyncfatfs/fat_standard.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Blackbox/asyncfatfs/%.o Core/Blackbox/asyncfatfs/%.su: ../Core/Blackbox/asyncfatfs/%.c Core/Blackbox/asyncfatfs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Blackbox-2f-asyncfatfs

clean-Core-2f-Blackbox-2f-asyncfatfs:
	-$(RM) ./Core/Blackbox/asyncfatfs/asyncfatfs.d ./Core/Blackbox/asyncfatfs/asyncfatfs.o ./Core/Blackbox/asyncfatfs/asyncfatfs.su ./Core/Blackbox/asyncfatfs/fat_standard.d ./Core/Blackbox/asyncfatfs/fat_standard.o ./Core/Blackbox/asyncfatfs/fat_standard.su

.PHONY: clean-Core-2f-Blackbox-2f-asyncfatfs

