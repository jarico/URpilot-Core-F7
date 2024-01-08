################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Sensores/Barometro/baro_bosch.c \
../Core/Sensores/Barometro/baro_teConectivity.c \
../Core/Sensores/Barometro/barometro.c 

OBJS += \
./Core/Sensores/Barometro/baro_bosch.o \
./Core/Sensores/Barometro/baro_teConectivity.o \
./Core/Sensores/Barometro/barometro.o 

C_DEPS += \
./Core/Sensores/Barometro/baro_bosch.d \
./Core/Sensores/Barometro/baro_teConectivity.d \
./Core/Sensores/Barometro/barometro.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Sensores/Barometro/%.o Core/Sensores/Barometro/%.su: ../Core/Sensores/Barometro/%.c Core/Sensores/Barometro/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Sensores-2f-Barometro

clean-Core-2f-Sensores-2f-Barometro:
	-$(RM) ./Core/Sensores/Barometro/baro_bosch.d ./Core/Sensores/Barometro/baro_bosch.o ./Core/Sensores/Barometro/baro_bosch.su ./Core/Sensores/Barometro/baro_teConectivity.d ./Core/Sensores/Barometro/baro_teConectivity.o ./Core/Sensores/Barometro/baro_teConectivity.su ./Core/Sensores/Barometro/barometro.d ./Core/Sensores/Barometro/barometro.o ./Core/Sensores/Barometro/barometro.su

.PHONY: clean-Core-2f-Sensores-2f-Barometro

