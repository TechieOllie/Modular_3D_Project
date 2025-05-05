################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/bsp/SD/FatFs/src/drivers/sd_diskio.c 

OBJS += \
./drivers/bsp/SD/FatFs/src/drivers/sd_diskio.o 

C_DEPS += \
./drivers/bsp/SD/FatFs/src/drivers/sd_diskio.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/bsp/SD/FatFs/src/drivers/%.o drivers/bsp/SD/FatFs/src/drivers/%.su drivers/bsp/SD/FatFs/src/drivers/%.cyclo: ../drivers/bsp/SD/FatFs/src/drivers/%.c drivers/bsp/SD/FatFs/src/drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../core/Inc -I../drivers/stm32g4xx_hal/Inc -I../drivers/stm32g4xx_hal/Inc/Legacy -I../drivers/cmsis/Device/ST/STM32G4xx/Include -I../drivers/cmsis/Include -I../app -I../drivers/bsp -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-bsp-2f-SD-2f-FatFs-2f-src-2f-drivers

clean-drivers-2f-bsp-2f-SD-2f-FatFs-2f-src-2f-drivers:
	-$(RM) ./drivers/bsp/SD/FatFs/src/drivers/sd_diskio.cyclo ./drivers/bsp/SD/FatFs/src/drivers/sd_diskio.d ./drivers/bsp/SD/FatFs/src/drivers/sd_diskio.o ./drivers/bsp/SD/FatFs/src/drivers/sd_diskio.su

.PHONY: clean-drivers-2f-bsp-2f-SD-2f-FatFs-2f-src-2f-drivers

