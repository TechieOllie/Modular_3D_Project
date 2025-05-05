################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/bsp/BH1750FVI/stm32g4_bh1750fvi.c 

OBJS += \
./drivers/bsp/BH1750FVI/stm32g4_bh1750fvi.o 

C_DEPS += \
./drivers/bsp/BH1750FVI/stm32g4_bh1750fvi.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/bsp/BH1750FVI/%.o drivers/bsp/BH1750FVI/%.su drivers/bsp/BH1750FVI/%.cyclo: ../drivers/bsp/BH1750FVI/%.c drivers/bsp/BH1750FVI/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../core/Inc -I../drivers/stm32g4xx_hal/Inc -I../drivers/stm32g4xx_hal/Inc/Legacy -I../drivers/cmsis/Device/ST/STM32G4xx/Include -I../drivers/cmsis/Include -I../app -I../drivers/bsp -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-bsp-2f-BH1750FVI

clean-drivers-2f-bsp-2f-BH1750FVI:
	-$(RM) ./drivers/bsp/BH1750FVI/stm32g4_bh1750fvi.cyclo ./drivers/bsp/BH1750FVI/stm32g4_bh1750fvi.d ./drivers/bsp/BH1750FVI/stm32g4_bh1750fvi.o ./drivers/bsp/BH1750FVI/stm32g4_bh1750fvi.su

.PHONY: clean-drivers-2f-bsp-2f-BH1750FVI

