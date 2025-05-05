################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include drivers/stm32g4xx_hal/Src/subdir.mk
-include drivers/bsp/tft_ili9341/subdir.mk
-include drivers/bsp/epaper/Fonts/subdir.mk
-include drivers/bsp/epaper/subdir.mk
-include drivers/bsp/YX6300/subdir.mk
-include drivers/bsp/WS2812/subdir.mk
-include drivers/bsp/VL53L0X/subdir.mk
-include drivers/bsp/SD/FatFs/src/option/subdir.mk
-include drivers/bsp/SD/FatFs/src/drivers/subdir.mk
-include drivers/bsp/SD/FatFs/src/subdir.mk
-include drivers/bsp/SD/subdir.mk
-include drivers/bsp/NFC03A1/lib_nfc/lib_pcd/subdir.mk
-include drivers/bsp/NFC03A1/lib_nfc/lib_NDEF/subdir.mk
-include drivers/bsp/NFC03A1/lib_nfc/common/subdir.mk
-include drivers/bsp/NFC03A1/CR95HF/subdir.mk
-include drivers/bsp/NFC03A1/subdir.mk
-include drivers/bsp/MotorDC/subdir.mk
-include drivers/bsp/MatrixKeyboard/subdir.mk
-include drivers/bsp/MPU6050/subdir.mk
-include drivers/bsp/MCP23S17/subdir.mk
-include drivers/bsp/MCP23017/subdir.mk
-include drivers/bsp/LD19/subdir.mk
-include drivers/bsp/HC-SR04/subdir.mk
-include drivers/bsp/HC-05/subdir.mk
-include drivers/bsp/GPS/subdir.mk
-include drivers/bsp/DS18B20/subdir.mk
-include drivers/bsp/DHT11/subdir.mk
-include drivers/bsp/BMP180/subdir.mk
-include drivers/bsp/BH1750FVI/subdir.mk
-include drivers/bsp/APDS9960/subdir.mk
-include drivers/bsp/subdir.mk
-include core/Startup/subdir.mk
-include core/Src/subdir.mk
-include app/subdir.mk
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


BUILD_ARTIFACT_NAME := corexy
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
corexy.elf \

MAP_FILES += \
corexy.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
corexy.list \


# All Target
all: main-build

# Main-build Target
main-build: corexy.elf secondary-outputs

# Tool invocations
corexy.elf corexy.map: $(OBJS) $(USER_OBJS) fail-specified-linker-script-missing makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "corexy.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 --specs=nosys.specs -Wl,-Map="corexy.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

corexy.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "corexy.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) corexy.elf corexy.list corexy.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
