################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/bsp/MotorDC/stm32g4_motorDC.c 

OBJS += \
./drivers/bsp/MotorDC/stm32g4_motorDC.o 

C_DEPS += \
./drivers/bsp/MotorDC/stm32g4_motorDC.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/bsp/MotorDC/%.o drivers/bsp/MotorDC/%.su drivers/bsp/MotorDC/%.cyclo: ../drivers/bsp/MotorDC/%.c drivers/bsp/MotorDC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../core/Inc -I../drivers/stm32g4xx_hal/Inc -I../drivers/stm32g4xx_hal/Inc/Legacy -I../drivers/cmsis/Device/ST/STM32G4xx/Include -I../drivers/cmsis/Include -I../app -I../drivers/bsp -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-bsp-2f-MotorDC

clean-drivers-2f-bsp-2f-MotorDC:
	-$(RM) ./drivers/bsp/MotorDC/stm32g4_motorDC.cyclo ./drivers/bsp/MotorDC/stm32g4_motorDC.d ./drivers/bsp/MotorDC/stm32g4_motorDC.o ./drivers/bsp/MotorDC/stm32g4_motorDC.su

.PHONY: clean-drivers-2f-bsp-2f-MotorDC

