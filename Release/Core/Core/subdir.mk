################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Core/fallo_sistema.c \
../Core/Core/inicializacion.c \
../Core/Core/led_estado.c \
../Core/Core/main.c \
../Core/Core/stack.c 

OBJS += \
./Core/Core/fallo_sistema.o \
./Core/Core/inicializacion.o \
./Core/Core/led_estado.o \
./Core/Core/main.o \
./Core/Core/stack.o 

C_DEPS += \
./Core/Core/fallo_sistema.d \
./Core/Core/inicializacion.d \
./Core/Core/led_estado.d \
./Core/Core/main.d \
./Core/Core/stack.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Core/%.o Core/Core/%.su: ../Core/Core/%.c Core/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Core

clean-Core-2f-Core:
	-$(RM) ./Core/Core/fallo_sistema.d ./Core/Core/fallo_sistema.o ./Core/Core/fallo_sistema.su ./Core/Core/inicializacion.d ./Core/Core/inicializacion.o ./Core/Core/inicializacion.su ./Core/Core/led_estado.d ./Core/Core/led_estado.o ./Core/Core/led_estado.su ./Core/Core/main.d ./Core/Core/main.o ./Core/Core/main.su ./Core/Core/stack.d ./Core/Core/stack.o ./Core/Core/stack.su

.PHONY: clean-Core-2f-Core

