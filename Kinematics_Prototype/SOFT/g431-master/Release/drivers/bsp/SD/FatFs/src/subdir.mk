################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/bsp/SD/FatFs/src/diskio.c \
../drivers/bsp/SD/FatFs/src/ff.c \
../drivers/bsp/SD/FatFs/src/ff_gen_drv.c 

OBJS += \
./drivers/bsp/SD/FatFs/src/diskio.o \
./drivers/bsp/SD/FatFs/src/ff.o \
./drivers/bsp/SD/FatFs/src/ff_gen_drv.o 

C_DEPS += \
./drivers/bsp/SD/FatFs/src/diskio.d \
./drivers/bsp/SD/FatFs/src/ff.d \
./drivers/bsp/SD/FatFs/src/ff_gen_drv.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/bsp/SD/FatFs/src/%.o drivers/bsp/SD/FatFs/src/%.su drivers/bsp/SD/FatFs/src/%.cyclo: ../drivers/bsp/SD/FatFs/src/%.c drivers/bsp/SD/FatFs/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../core/Inc -I../drivers/stm32g4xx_hal/Inc -I../drivers/stm32g4xx_hal/Inc/Legacy -I../drivers/cmsis/Device/ST/STM32G4xx/Include -I../drivers/cmsis/Include -I../app -I../drivers/bsp -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-bsp-2f-SD-2f-FatFs-2f-src

clean-drivers-2f-bsp-2f-SD-2f-FatFs-2f-src:
	-$(RM) ./drivers/bsp/SD/FatFs/src/diskio.cyclo ./drivers/bsp/SD/FatFs/src/diskio.d ./drivers/bsp/SD/FatFs/src/diskio.o ./drivers/bsp/SD/FatFs/src/diskio.su ./drivers/bsp/SD/FatFs/src/ff.cyclo ./drivers/bsp/SD/FatFs/src/ff.d ./drivers/bsp/SD/FatFs/src/ff.o ./drivers/bsp/SD/FatFs/src/ff.su ./drivers/bsp/SD/FatFs/src/ff_gen_drv.cyclo ./drivers/bsp/SD/FatFs/src/ff_gen_drv.d ./drivers/bsp/SD/FatFs/src/ff_gen_drv.o ./drivers/bsp/SD/FatFs/src/ff_gen_drv.su

.PHONY: clean-drivers-2f-bsp-2f-SD-2f-FatFs-2f-src

