################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/bsp/epaper/Fonts/stm32g4_epaper_font12.c \
../drivers/bsp/epaper/Fonts/stm32g4_epaper_font16.c \
../drivers/bsp/epaper/Fonts/stm32g4_epaper_font20.c \
../drivers/bsp/epaper/Fonts/stm32g4_epaper_font24.c \
../drivers/bsp/epaper/Fonts/stm32g4_epaper_font8.c 

OBJS += \
./drivers/bsp/epaper/Fonts/stm32g4_epaper_font12.o \
./drivers/bsp/epaper/Fonts/stm32g4_epaper_font16.o \
./drivers/bsp/epaper/Fonts/stm32g4_epaper_font20.o \
./drivers/bsp/epaper/Fonts/stm32g4_epaper_font24.o \
./drivers/bsp/epaper/Fonts/stm32g4_epaper_font8.o 

C_DEPS += \
./drivers/bsp/epaper/Fonts/stm32g4_epaper_font12.d \
./drivers/bsp/epaper/Fonts/stm32g4_epaper_font16.d \
./drivers/bsp/epaper/Fonts/stm32g4_epaper_font20.d \
./drivers/bsp/epaper/Fonts/stm32g4_epaper_font24.d \
./drivers/bsp/epaper/Fonts/stm32g4_epaper_font8.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/bsp/epaper/Fonts/%.o drivers/bsp/epaper/Fonts/%.su drivers/bsp/epaper/Fonts/%.cyclo: ../drivers/bsp/epaper/Fonts/%.c drivers/bsp/epaper/Fonts/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../core/Inc -I../drivers/stm32g4xx_hal/Inc -I../drivers/stm32g4xx_hal/Inc/Legacy -I../drivers/cmsis/Device/ST/STM32G4xx/Include -I../drivers/cmsis/Include -I../app -I../drivers/bsp -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-bsp-2f-epaper-2f-Fonts

clean-drivers-2f-bsp-2f-epaper-2f-Fonts:
	-$(RM) ./drivers/bsp/epaper/Fonts/stm32g4_epaper_font12.cyclo ./drivers/bsp/epaper/Fonts/stm32g4_epaper_font12.d ./drivers/bsp/epaper/Fonts/stm32g4_epaper_font12.o ./drivers/bsp/epaper/Fonts/stm32g4_epaper_font12.su ./drivers/bsp/epaper/Fonts/stm32g4_epaper_font16.cyclo ./drivers/bsp/epaper/Fonts/stm32g4_epaper_font16.d ./drivers/bsp/epaper/Fonts/stm32g4_epaper_font16.o ./drivers/bsp/epaper/Fonts/stm32g4_epaper_font16.su ./drivers/bsp/epaper/Fonts/stm32g4_epaper_font20.cyclo ./drivers/bsp/epaper/Fonts/stm32g4_epaper_font20.d ./drivers/bsp/epaper/Fonts/stm32g4_epaper_font20.o ./drivers/bsp/epaper/Fonts/stm32g4_epaper_font20.su ./drivers/bsp/epaper/Fonts/stm32g4_epaper_font24.cyclo ./drivers/bsp/epaper/Fonts/stm32g4_epaper_font24.d ./drivers/bsp/epaper/Fonts/stm32g4_epaper_font24.o ./drivers/bsp/epaper/Fonts/stm32g4_epaper_font24.su ./drivers/bsp/epaper/Fonts/stm32g4_epaper_font8.cyclo ./drivers/bsp/epaper/Fonts/stm32g4_epaper_font8.d ./drivers/bsp/epaper/Fonts/stm32g4_epaper_font8.o ./drivers/bsp/epaper/Fonts/stm32g4_epaper_font8.su

.PHONY: clean-drivers-2f-bsp-2f-epaper-2f-Fonts

