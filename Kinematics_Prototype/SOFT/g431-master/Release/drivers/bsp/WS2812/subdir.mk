################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../drivers/bsp/WS2812/WS2812_asm.s 

C_SRCS += \
../drivers/bsp/WS2812/stm32g4_ws2812.c 

OBJS += \
./drivers/bsp/WS2812/WS2812_asm.o \
./drivers/bsp/WS2812/stm32g4_ws2812.o 

S_DEPS += \
./drivers/bsp/WS2812/WS2812_asm.d 

C_DEPS += \
./drivers/bsp/WS2812/stm32g4_ws2812.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/bsp/WS2812/%.o: ../drivers/bsp/WS2812/%.s drivers/bsp/WS2812/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"
drivers/bsp/WS2812/%.o drivers/bsp/WS2812/%.su drivers/bsp/WS2812/%.cyclo: ../drivers/bsp/WS2812/%.c drivers/bsp/WS2812/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../core/Inc -I../drivers/stm32g4xx_hal/Inc -I../drivers/stm32g4xx_hal/Inc/Legacy -I../drivers/cmsis/Device/ST/STM32G4xx/Include -I../drivers/cmsis/Include -I../app -I../drivers/bsp -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-bsp-2f-WS2812

clean-drivers-2f-bsp-2f-WS2812:
	-$(RM) ./drivers/bsp/WS2812/WS2812_asm.d ./drivers/bsp/WS2812/WS2812_asm.o ./drivers/bsp/WS2812/stm32g4_ws2812.cyclo ./drivers/bsp/WS2812/stm32g4_ws2812.d ./drivers/bsp/WS2812/stm32g4_ws2812.o ./drivers/bsp/WS2812/stm32g4_ws2812.su

.PHONY: clean-drivers-2f-bsp-2f-WS2812

