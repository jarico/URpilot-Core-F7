################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Sensores/GPS/gps.c \
../Core/Sensores/GPS/gps_ublox.c 

OBJS += \
./Core/Sensores/GPS/gps.o \
./Core/Sensores/GPS/gps_ublox.o 

C_DEPS += \
./Core/Sensores/GPS/gps.d \
./Core/Sensores/GPS/gps_ublox.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Sensores/GPS/%.o Core/Sensores/GPS/%.su: ../Core/Sensores/GPS/%.c Core/Sensores/GPS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Sensores-2f-GPS

clean-Core-2f-Sensores-2f-GPS:
	-$(RM) ./Core/Sensores/GPS/gps.d ./Core/Sensores/GPS/gps.o ./Core/Sensores/GPS/gps.su ./Core/Sensores/GPS/gps_ublox.d ./Core/Sensores/GPS/gps_ublox.o ./Core/Sensores/GPS/gps_ublox.su

.PHONY: clean-Core-2f-Sensores-2f-GPS

