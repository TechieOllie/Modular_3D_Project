################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/bsp/SD/FatFs/src/option/ccsbcs.c \
../drivers/bsp/SD/FatFs/src/option/syscall.c 

OBJS += \
./drivers/bsp/SD/FatFs/src/option/ccsbcs.o \
./drivers/bsp/SD/FatFs/src/option/syscall.o 

C_DEPS += \
./drivers/bsp/SD/FatFs/src/option/ccsbcs.d \
./drivers/bsp/SD/FatFs/src/option/syscall.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/bsp/SD/FatFs/src/option/%.o drivers/bsp/SD/FatFs/src/option/%.su drivers/bsp/SD/FatFs/src/option/%.cyclo: ../drivers/bsp/SD/FatFs/src/option/%.c drivers/bsp/SD/FatFs/src/option/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../core/Inc -I../drivers/stm32g4xx_hal/Inc -I../drivers/stm32g4xx_hal/Inc/Legacy -I../drivers/cmsis/Device/ST/STM32G4xx/Include -I../drivers/cmsis/Include -I../app -I../drivers/bsp -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-bsp-2f-SD-2f-FatFs-2f-src-2f-option

clean-drivers-2f-bsp-2f-SD-2f-FatFs-2f-src-2f-option:
	-$(RM) ./drivers/bsp/SD/FatFs/src/option/ccsbcs.cyclo ./drivers/bsp/SD/FatFs/src/option/ccsbcs.d ./drivers/bsp/SD/FatFs/src/option/ccsbcs.o ./drivers/bsp/SD/FatFs/src/option/ccsbcs.su ./drivers/bsp/SD/FatFs/src/option/syscall.cyclo ./drivers/bsp/SD/FatFs/src/option/syscall.d ./drivers/bsp/SD/FatFs/src/option/syscall.o ./drivers/bsp/SD/FatFs/src/option/syscall.su

.PHONY: clean-drivers-2f-bsp-2f-SD-2f-FatFs-2f-src-2f-option

