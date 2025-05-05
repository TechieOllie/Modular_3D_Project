################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/bsp/NFC03A1/CR95HF/drv_95hf.c 

OBJS += \
./drivers/bsp/NFC03A1/CR95HF/drv_95hf.o 

C_DEPS += \
./drivers/bsp/NFC03A1/CR95HF/drv_95hf.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/bsp/NFC03A1/CR95HF/%.o drivers/bsp/NFC03A1/CR95HF/%.su drivers/bsp/NFC03A1/CR95HF/%.cyclo: ../drivers/bsp/NFC03A1/CR95HF/%.c drivers/bsp/NFC03A1/CR95HF/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../core/Inc -I../drivers/stm32g4xx_hal/Inc -I../drivers/stm32g4xx_hal/Inc/Legacy -I../drivers/cmsis/Device/ST/STM32G4xx/Include -I../drivers/cmsis/Include -I../app -I../drivers/bsp -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-bsp-2f-NFC03A1-2f-CR95HF

clean-drivers-2f-bsp-2f-NFC03A1-2f-CR95HF:
	-$(RM) ./drivers/bsp/NFC03A1/CR95HF/drv_95hf.cyclo ./drivers/bsp/NFC03A1/CR95HF/drv_95hf.d ./drivers/bsp/NFC03A1/CR95HF/drv_95hf.o ./drivers/bsp/NFC03A1/CR95HF/drv_95hf.su

.PHONY: clean-drivers-2f-bsp-2f-NFC03A1-2f-CR95HF

