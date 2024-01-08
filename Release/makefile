################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/STM32_USB_Device_Library/Core/Src/subdir.mk
-include Middlewares/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
-include Drivers/STM32F7xx_HAL_Driver/Src/subdir.mk
-include Core/Version/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Sistema/subdir.mk
-include Core/Sensores/PM/subdir.mk
-include Core/Sensores/Magnetometro/subdir.mk
-include Core/Sensores/IMU/subdir.mk
-include Core/Sensores/GPS/subdir.mk
-include Core/Sensores/Calibrador/subdir.mk
-include Core/Sensores/Barometro/subdir.mk
-include Core/Sensores/subdir.mk
-include Core/Scheduler/subdir.mk
-include Core/Radio/subdir.mk
-include Core/PID/subdir.mk
-include Core/Motores/subdir.mk
-include Core/GP/subdir.mk
-include Core/Filtros/subdir.mk
-include Core/FC/subdir.mk
-include Core/Drivers/subdir.mk
-include Core/Core/subdir.mk
-include Core/Comun/subdir.mk
-include Core/Blackbox/asyncfatfs/subdir.mk
-include Core/Blackbox/subdir.mk
-include Core/AHRS/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := URpilot_Core_F7
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
URpilot_Core_F7.elf \

MAP_FILES += \
URpilot_Core_F7.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
URpilot_Core_F7.list \


# All Target
all: main-build

# Main-build Target
main-build: URpilot_Core_F7.elf secondary-outputs

# Tool invocations
URpilot_Core_F7.elf URpilot_Core_F7.map: $(OBJS) $(USER_OBJS) C:\Users\ramon\Documents\STM32CubeIde\URpilot_Core_F7\Linker\stm32f767vgtx.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "URpilot_Core_F7.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"C:\Users\ramon\Documents\STM32CubeIde\URpilot_Core_F7\Linker\stm32f767vgtx.ld" --specs=nosys.specs -Wl,-Map="URpilot_Core_F7.map" -Wl,--gc-sections -static -L../Linker --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

URpilot_Core_F7.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "URpilot_Core_F7.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) URpilot_Core_F7.elf URpilot_Core_F7.list URpilot_Core_F7.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
