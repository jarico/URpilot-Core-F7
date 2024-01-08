################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Motores/dshot.c \
../Core/Motores/dshot_hal.c \
../Core/Motores/motor.c 

OBJS += \
./Core/Motores/dshot.o \
./Core/Motores/dshot_hal.o \
./Core/Motores/motor.o 

C_DEPS += \
./Core/Motores/dshot.d \
./Core/Motores/dshot_hal.d \
./Core/Motores/motor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Motores/%.o Core/Motores/%.su: ../Core/Motores/%.c Core/Motores/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Motores

clean-Core-2f-Motores:
	-$(RM) ./Core/Motores/dshot.d ./Core/Motores/dshot.o ./Core/Motores/dshot.su ./Core/Motores/dshot_hal.d ./Core/Motores/dshot_hal.o ./Core/Motores/dshot_hal.su ./Core/Motores/motor.d ./Core/Motores/motor.o ./Core/Motores/motor.su

.PHONY: clean-Core-2f-Motores

