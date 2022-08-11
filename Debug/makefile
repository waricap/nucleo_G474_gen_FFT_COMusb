################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU ARM Embedded (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/modbus/rtu/subdir.mk
-include Drivers/modbus/port/subdir.mk
-include Drivers/modbus/functions/subdir.mk
-include Drivers/modbus/subdir.mk
-include Drivers/STM32G4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := nucleo_G474_gen_FFT_COMusb
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
nucleo_G474_gen_FFT_COMusb.elf \

MAP_FILES += \
nucleo_G474_gen_FFT_COMusb.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
nucleo_G474_gen_FFT_COMusb.list \


# All Target
all: main-build

# Main-build Target
main-build: nucleo_G474_gen_FFT_COMusb.elf secondary-outputs

# Tool invocations
nucleo_G474_gen_FFT_COMusb.elf nucleo_G474_gen_FFT_COMusb.map: $(OBJS) $(USER_OBJS) E:\Project\STM32_IDE_workspace\nucleo_G474_gen_FFT_COMusb\STM32G474RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "nucleo_G474_gen_FFT_COMusb.elf" @"objects.list" $(USER_OBJS) -Wl,--start-group $(LIBS) -Wl,--end-group -mcpu=cortex-m4 -T"E:\Project\STM32_IDE_workspace\nucleo_G474_gen_FFT_COMusb\STM32G474RETX_FLASH.ld" -Wl,-Map="nucleo_G474_gen_FFT_COMusb.map" -Wl,--gc-sections -static -L"E:\stm32_repositary\STM32Cube_FW_G4_V1.5.0\Drivers\CMSIS\DSP\Lib\ARM" -L"E:\stm32_repositary\STM32Cube_FW_G4_V1.5.0\Drivers\CMSIS\DSP\Lib\GCC" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

nucleo_G474_gen_FFT_COMusb.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "nucleo_G474_gen_FFT_COMusb.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) default.size.stdout nucleo_G474_gen_FFT_COMusb.elf nucleo_G474_gen_FFT_COMusb.list nucleo_G474_gen_FFT_COMusb.map
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
