################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../app/corexy.c \
../app/main.c \
../app/parser.c \
../app/stepper_motor.c 

OBJS += \
./app/corexy.o \
./app/main.o \
./app/parser.o \
./app/stepper_motor.o 

C_DEPS += \
./app/corexy.d \
./app/main.d \
./app/parser.d \
./app/stepper_motor.d 


# Each subdirectory must supply rules for building sources it contributes
app/%.o app/%.su app/%.cyclo: ../app/%.c app/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../core/Inc -I../drivers/stm32g4xx_hal/Inc -I../drivers/stm32g4xx_hal/Inc/Legacy -I../drivers/cmsis/Device/ST/STM32G4xx/Include -I../drivers/cmsis/Include -I../app -I../drivers/bsp -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-app

clean-app:
	-$(RM) ./app/corexy.cyclo ./app/corexy.d ./app/corexy.o ./app/corexy.su ./app/main.cyclo ./app/main.d ./app/main.o ./app/main.su ./app/parser.cyclo ./app/parser.d ./app/parser.o ./app/parser.su ./app/stepper_motor.cyclo ./app/stepper_motor.d ./app/stepper_motor.o ./app/stepper_motor.su

.PHONY: clean-app

