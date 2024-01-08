################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Sensores/Calibrador/calibrador.c \
../Core/Sensores/Calibrador/calibrador_imu.c \
../Core/Sensores/Calibrador/calibrador_mag.c 

OBJS += \
./Core/Sensores/Calibrador/calibrador.o \
./Core/Sensores/Calibrador/calibrador_imu.o \
./Core/Sensores/Calibrador/calibrador_mag.o 

C_DEPS += \
./Core/Sensores/Calibrador/calibrador.d \
./Core/Sensores/Calibrador/calibrador_imu.d \
./Core/Sensores/Calibrador/calibrador_mag.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Sensores/Calibrador/%.o Core/Sensores/Calibrador/%.su: ../Core/Sensores/Calibrador/%.c Core/Sensores/Calibrador/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Sensores-2f-Calibrador

clean-Core-2f-Sensores-2f-Calibrador:
	-$(RM) ./Core/Sensores/Calibrador/calibrador.d ./Core/Sensores/Calibrador/calibrador.o ./Core/Sensores/Calibrador/calibrador.su ./Core/Sensores/Calibrador/calibrador_imu.d ./Core/Sensores/Calibrador/calibrador_imu.o ./Core/Sensores/Calibrador/calibrador_imu.su ./Core/Sensores/Calibrador/calibrador_mag.d ./Core/Sensores/Calibrador/calibrador_mag.o ./Core/Sensores/Calibrador/calibrador_mag.su

.PHONY: clean-Core-2f-Sensores-2f-Calibrador

