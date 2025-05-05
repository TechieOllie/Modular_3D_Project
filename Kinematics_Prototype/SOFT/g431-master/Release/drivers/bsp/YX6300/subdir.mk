################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/bsp/YX6300/stm32g4_yx6300.c 

OBJS += \
./drivers/bsp/YX6300/stm32g4_yx6300.o 

C_DEPS += \
./drivers/bsp/YX6300/stm32g4_yx6300.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/bsp/YX6300/%.o drivers/bsp/YX6300/%.su drivers/bsp/YX6300/%.cyclo: ../drivers/bsp/YX6300/%.c drivers/bsp/YX6300/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../core/Inc -I../drivers/stm32g4xx_hal/Inc -I../drivers/stm32g4xx_hal/Inc/Legacy -I../drivers/cmsis/Device/ST/STM32G4xx/Include -I../drivers/cmsis/Include -I../app -I../drivers/bsp -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-bsp-2f-YX6300

clean-drivers-2f-bsp-2f-YX6300:
	-$(RM) ./drivers/bsp/YX6300/stm32g4_yx6300.cyclo ./drivers/bsp/YX6300/stm32g4_yx6300.d ./drivers/bsp/YX6300/stm32g4_yx6300.o ./drivers/bsp/YX6300/stm32g4_yx6300.su

.PHONY: clean-drivers-2f-bsp-2f-YX6300

