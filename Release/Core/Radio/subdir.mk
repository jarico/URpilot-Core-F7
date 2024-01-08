################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Radio/ibus.c \
../Core/Radio/ppm.c \
../Core/Radio/radio.c \
../Core/Radio/sbus.c 

OBJS += \
./Core/Radio/ibus.o \
./Core/Radio/ppm.o \
./Core/Radio/radio.o \
./Core/Radio/sbus.o 

C_DEPS += \
./Core/Radio/ibus.d \
./Core/Radio/ppm.d \
./Core/Radio/radio.d \
./Core/Radio/sbus.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Radio/%.o Core/Radio/%.su: ../Core/Radio/%.c Core/Radio/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Radio

clean-Core-2f-Radio:
	-$(RM) ./Core/Radio/ibus.d ./Core/Radio/ibus.o ./Core/Radio/ibus.su ./Core/Radio/ppm.d ./Core/Radio/ppm.o ./Core/Radio/ppm.su ./Core/Radio/radio.d ./Core/Radio/radio.o ./Core/Radio/radio.su ./Core/Radio/sbus.d ./Core/Radio/sbus.o ./Core/Radio/sbus.su

.PHONY: clean-Core-2f-Radio

