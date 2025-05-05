################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/bsp/NFC03A1/lib_nfc/common/lib_95HFConfigManager.c 

OBJS += \
./drivers/bsp/NFC03A1/lib_nfc/common/lib_95HFConfigManager.o 

C_DEPS += \
./drivers/bsp/NFC03A1/lib_nfc/common/lib_95HFConfigManager.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/bsp/NFC03A1/lib_nfc/common/%.o drivers/bsp/NFC03A1/lib_nfc/common/%.su drivers/bsp/NFC03A1/lib_nfc/common/%.cyclo: ../drivers/bsp/NFC03A1/lib_nfc/common/%.c drivers/bsp/NFC03A1/lib_nfc/common/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../core/Inc -I../drivers/stm32g4xx_hal/Inc -I../drivers/stm32g4xx_hal/Inc/Legacy -I../drivers/cmsis/Device/ST/STM32G4xx/Include -I../drivers/cmsis/Include -I../app -I../drivers/bsp -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-bsp-2f-NFC03A1-2f-lib_nfc-2f-common

clean-drivers-2f-bsp-2f-NFC03A1-2f-lib_nfc-2f-common:
	-$(RM) ./drivers/bsp/NFC03A1/lib_nfc/common/lib_95HFConfigManager.cyclo ./drivers/bsp/NFC03A1/lib_nfc/common/lib_95HFConfigManager.d ./drivers/bsp/NFC03A1/lib_nfc/common/lib_95HFConfigManager.o ./drivers/bsp/NFC03A1/lib_nfc/common/lib_95HFConfigManager.su

.PHONY: clean-drivers-2f-bsp-2f-NFC03A1-2f-lib_nfc-2f-common

