################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/bsp/MCP23017/stm32g4_mcp23017.c 

OBJS += \
./drivers/bsp/MCP23017/stm32g4_mcp23017.o 

C_DEPS += \
./drivers/bsp/MCP23017/stm32g4_mcp23017.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/bsp/MCP23017/%.o drivers/bsp/MCP23017/%.su drivers/bsp/MCP23017/%.cyclo: ../drivers/bsp/MCP23017/%.c drivers/bsp/MCP23017/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../core/Inc -I../drivers/stm32g4xx_hal/Inc -I../drivers/stm32g4xx_hal/Inc/Legacy -I../drivers/cmsis/Device/ST/STM32G4xx/Include -I../drivers/cmsis/Include -I../app -I../drivers/bsp -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-bsp-2f-MCP23017

clean-drivers-2f-bsp-2f-MCP23017:
	-$(RM) ./drivers/bsp/MCP23017/stm32g4_mcp23017.cyclo ./drivers/bsp/MCP23017/stm32g4_mcp23017.d ./drivers/bsp/MCP23017/stm32g4_mcp23017.o ./drivers/bsp/MCP23017/stm32g4_mcp23017.su

.PHONY: clean-drivers-2f-bsp-2f-MCP23017

