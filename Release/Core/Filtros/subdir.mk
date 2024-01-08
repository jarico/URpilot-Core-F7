################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Filtros/filtro_derivada.c \
../Core/Filtros/filtro_media_movil.c \
../Core/Filtros/filtro_notch.c \
../Core/Filtros/filtro_pasa_bajo.c 

OBJS += \
./Core/Filtros/filtro_derivada.o \
./Core/Filtros/filtro_media_movil.o \
./Core/Filtros/filtro_notch.o \
./Core/Filtros/filtro_pasa_bajo.o 

C_DEPS += \
./Core/Filtros/filtro_derivada.d \
./Core/Filtros/filtro_media_movil.d \
./Core/Filtros/filtro_notch.d \
./Core/Filtros/filtro_pasa_bajo.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Filtros/%.o Core/Filtros/%.su: ../Core/Filtros/%.c Core/Filtros/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Filtros

clean-Core-2f-Filtros:
	-$(RM) ./Core/Filtros/filtro_derivada.d ./Core/Filtros/filtro_derivada.o ./Core/Filtros/filtro_derivada.su ./Core/Filtros/filtro_media_movil.d ./Core/Filtros/filtro_media_movil.o ./Core/Filtros/filtro_media_movil.su ./Core/Filtros/filtro_notch.d ./Core/Filtros/filtro_notch.o ./Core/Filtros/filtro_notch.su ./Core/Filtros/filtro_pasa_bajo.d ./Core/Filtros/filtro_pasa_bajo.o ./Core/Filtros/filtro_pasa_bajo.su

.PHONY: clean-Core-2f-Filtros

