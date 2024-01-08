################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Comun/crc.c \
../Core/Comun/localizacion.c \
../Core/Comun/matematicas.c \
../Core/Comun/matriz.c 

OBJS += \
./Core/Comun/crc.o \
./Core/Comun/localizacion.o \
./Core/Comun/matematicas.o \
./Core/Comun/matriz.o 

C_DEPS += \
./Core/Comun/crc.d \
./Core/Comun/localizacion.d \
./Core/Comun/matematicas.d \
./Core/Comun/matriz.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Comun/%.o Core/Comun/%.su: ../Core/Comun/%.c Core/Comun/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Comun

clean-Core-2f-Comun:
	-$(RM) ./Core/Comun/crc.d ./Core/Comun/crc.o ./Core/Comun/crc.su ./Core/Comun/localizacion.d ./Core/Comun/localizacion.o ./Core/Comun/localizacion.su ./Core/Comun/matematicas.d ./Core/Comun/matematicas.o ./Core/Comun/matematicas.su ./Core/Comun/matriz.d ./Core/Comun/matriz.o ./Core/Comun/matriz.su

.PHONY: clean-Core-2f-Comun

