################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/bsp/LD19/QS_maths.c \
../drivers/bsp/LD19/stm32g4_ld19.c \
../drivers/bsp/LD19/stm32g4_ld19_display.c 

OBJS += \
./drivers/bsp/LD19/QS_maths.o \
./drivers/bsp/LD19/stm32g4_ld19.o \
./drivers/bsp/LD19/stm32g4_ld19_display.o 

C_DEPS += \
./drivers/bsp/LD19/QS_maths.d \
./drivers/bsp/LD19/stm32g4_ld19.d \
./drivers/bsp/LD19/stm32g4_ld19_display.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/bsp/LD19/%.o drivers/bsp/LD19/%.su drivers/bsp/LD19/%.cyclo: ../drivers/bsp/LD19/%.c drivers/bsp/LD19/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../core/Inc -I../drivers/stm32g4xx_hal/Inc -I../drivers/stm32g4xx_hal/Inc/Legacy -I../drivers/cmsis/Device/ST/STM32G4xx/Include -I../drivers/cmsis/Include -I../app -I../drivers/bsp -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-bsp-2f-LD19

clean-drivers-2f-bsp-2f-LD19:
	-$(RM) ./drivers/bsp/LD19/QS_maths.cyclo ./drivers/bsp/LD19/QS_maths.d ./drivers/bsp/LD19/QS_maths.o ./drivers/bsp/LD19/QS_maths.su ./drivers/bsp/LD19/stm32g4_ld19.cyclo ./drivers/bsp/LD19/stm32g4_ld19.d ./drivers/bsp/LD19/stm32g4_ld19.o ./drivers/bsp/LD19/stm32g4_ld19.su ./drivers/bsp/LD19/stm32g4_ld19_display.cyclo ./drivers/bsp/LD19/stm32g4_ld19_display.d ./drivers/bsp/LD19/stm32g4_ld19_display.o ./drivers/bsp/LD19/stm32g4_ld19_display.su

.PHONY: clean-drivers-2f-bsp-2f-LD19

