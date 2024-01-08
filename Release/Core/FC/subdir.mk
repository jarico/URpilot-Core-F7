################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/FC/control.c \
../Core/FC/fc.c \
../Core/FC/mixer.c \
../Core/FC/rc.c 

OBJS += \
./Core/FC/control.o \
./Core/FC/fc.o \
./Core/FC/mixer.o \
./Core/FC/rc.o 

C_DEPS += \
./Core/FC/control.d \
./Core/FC/fc.d \
./Core/FC/mixer.d \
./Core/FC/rc.d 


# Each subdirectory must supply rules for building sources it contributes
Core/FC/%.o Core/FC/%.su: ../Core/FC/%.c Core/FC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-FC

clean-Core-2f-FC:
	-$(RM) ./Core/FC/control.d ./Core/FC/control.o ./Core/FC/control.su ./Core/FC/fc.d ./Core/FC/fc.o ./Core/FC/fc.su ./Core/FC/mixer.d ./Core/FC/mixer.o ./Core/FC/mixer.su ./Core/FC/rc.d ./Core/FC/rc.o ./Core/FC/rc.su

.PHONY: clean-Core-2f-FC

