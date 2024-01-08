################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Sensores/IMU/imu.c \
../Core/Sensores/IMU/imu_invensense.c 

OBJS += \
./Core/Sensores/IMU/imu.o \
./Core/Sensores/IMU/imu_invensense.o 

C_DEPS += \
./Core/Sensores/IMU/imu.d \
./Core/Sensores/IMU/imu_invensense.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Sensores/IMU/%.o Core/Sensores/IMU/%.su: ../Core/Sensores/IMU/%.c Core/Sensores/IMU/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Sensores-2f-IMU

clean-Core-2f-Sensores-2f-IMU:
	-$(RM) ./Core/Sensores/IMU/imu.d ./Core/Sensores/IMU/imu.o ./Core/Sensores/IMU/imu.su ./Core/Sensores/IMU/imu_invensense.d ./Core/Sensores/IMU/imu_invensense.o ./Core/Sensores/IMU/imu_invensense.su

.PHONY: clean-Core-2f-Sensores-2f-IMU

