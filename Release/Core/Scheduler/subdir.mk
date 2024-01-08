################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Scheduler/scheduler.c \
../Core/Scheduler/tareas.c 

OBJS += \
./Core/Scheduler/scheduler.o \
./Core/Scheduler/tareas.o 

C_DEPS += \
./Core/Scheduler/scheduler.d \
./Core/Scheduler/tareas.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Scheduler/%.o Core/Scheduler/%.su: ../Core/Scheduler/%.c Core/Scheduler/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Scheduler

clean-Core-2f-Scheduler:
	-$(RM) ./Core/Scheduler/scheduler.d ./Core/Scheduler/scheduler.o ./Core/Scheduler/scheduler.su ./Core/Scheduler/tareas.d ./Core/Scheduler/tareas.o ./Core/Scheduler/tareas.su

.PHONY: clean-Core-2f-Scheduler

