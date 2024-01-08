################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Blackbox/blackbox.c \
../Core/Blackbox/blackbox_sd.c \
../Core/Blackbox/sd.c \
../Core/Blackbox/sd_estandar.c \
../Core/Blackbox/sd_sdio.c \
../Core/Blackbox/sd_spi.c 

OBJS += \
./Core/Blackbox/blackbox.o \
./Core/Blackbox/blackbox_sd.o \
./Core/Blackbox/sd.o \
./Core/Blackbox/sd_estandar.o \
./Core/Blackbox/sd_sdio.o \
./Core/Blackbox/sd_spi.o 

C_DEPS += \
./Core/Blackbox/blackbox.d \
./Core/Blackbox/blackbox_sd.d \
./Core/Blackbox/sd.d \
./Core/Blackbox/sd_estandar.d \
./Core/Blackbox/sd_sdio.d \
./Core/Blackbox/sd_spi.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Blackbox/%.o Core/Blackbox/%.su: ../Core/Blackbox/%.c Core/Blackbox/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/STM32_USB_Device_Library/Core/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Blackbox

clean-Core-2f-Blackbox:
	-$(RM) ./Core/Blackbox/blackbox.d ./Core/Blackbox/blackbox.o ./Core/Blackbox/blackbox.su ./Core/Blackbox/blackbox_sd.d ./Core/Blackbox/blackbox_sd.o ./Core/Blackbox/blackbox_sd.su ./Core/Blackbox/sd.d ./Core/Blackbox/sd.o ./Core/Blackbox/sd.su ./Core/Blackbox/sd_estandar.d ./Core/Blackbox/sd_estandar.o ./Core/Blackbox/sd_estandar.su ./Core/Blackbox/sd_sdio.d ./Core/Blackbox/sd_sdio.o ./Core/Blackbox/sd_sdio.su ./Core/Blackbox/sd_spi.d ./Core/Blackbox/sd_spi.o ./Core/Blackbox/sd_spi.su

.PHONY: clean-Core-2f-Blackbox

