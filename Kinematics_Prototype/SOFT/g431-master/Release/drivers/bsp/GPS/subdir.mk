################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/bsp/GPS/stm32g4_gps.c 

OBJS += \
./drivers/bsp/GPS/stm32g4_gps.o 

C_DEPS += \
./drivers/bsp/GPS/stm32g4_gps.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/bsp/GPS/%.o drivers/bsp/GPS/%.su drivers/bsp/GPS/%.cyclo: ../drivers/bsp/GPS/%.c drivers/bsp/GPS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../core/Inc -I../drivers/stm32g4xx_hal/Inc -I../drivers/stm32g4xx_hal/Inc/Legacy -I../drivers/cmsis/Device/ST/STM32G4xx/Include -I../drivers/cmsis/Include -I../app -I../drivers/bsp -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-bsp-2f-GPS

clean-drivers-2f-bsp-2f-GPS:
	-$(RM) ./drivers/bsp/GPS/stm32g4_gps.cyclo ./drivers/bsp/GPS/stm32g4_gps.d ./drivers/bsp/GPS/stm32g4_gps.o ./drivers/bsp/GPS/stm32g4_gps.su

.PHONY: clean-drivers-2f-bsp-2f-GPS

