################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/bsp/SD/stm32g4_sd.c 

OBJS += \
./drivers/bsp/SD/stm32g4_sd.o 

C_DEPS += \
./drivers/bsp/SD/stm32g4_sd.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/bsp/SD/%.o drivers/bsp/SD/%.su drivers/bsp/SD/%.cyclo: ../drivers/bsp/SD/%.c drivers/bsp/SD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../core/Inc -I../drivers/stm32g4xx_hal/Inc -I../drivers/stm32g4xx_hal/Inc/Legacy -I../drivers/cmsis/Device/ST/STM32G4xx/Include -I../drivers/cmsis/Include -I../app -I../drivers/bsp -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-bsp-2f-SD

clean-drivers-2f-bsp-2f-SD:
	-$(RM) ./drivers/bsp/SD/stm32g4_sd.cyclo ./drivers/bsp/SD/stm32g4_sd.d ./drivers/bsp/SD/stm32g4_sd.o ./drivers/bsp/SD/stm32g4_sd.su

.PHONY: clean-drivers-2f-bsp-2f-SD

