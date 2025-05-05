################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/bsp/VL53L0X/stm32g4_vl53l0x_demo.c \
../drivers/bsp/VL53L0X/vl53l0x_api.c \
../drivers/bsp/VL53L0X/vl53l0x_api_calibration.c \
../drivers/bsp/VL53L0X/vl53l0x_api_core.c \
../drivers/bsp/VL53L0X/vl53l0x_api_ranging.c \
../drivers/bsp/VL53L0X/vl53l0x_api_strings.c \
../drivers/bsp/VL53L0X/vl53l0x_platform.c \
../drivers/bsp/VL53L0X/vl53l0x_platform_log.c 

OBJS += \
./drivers/bsp/VL53L0X/stm32g4_vl53l0x_demo.o \
./drivers/bsp/VL53L0X/vl53l0x_api.o \
./drivers/bsp/VL53L0X/vl53l0x_api_calibration.o \
./drivers/bsp/VL53L0X/vl53l0x_api_core.o \
./drivers/bsp/VL53L0X/vl53l0x_api_ranging.o \
./drivers/bsp/VL53L0X/vl53l0x_api_strings.o \
./drivers/bsp/VL53L0X/vl53l0x_platform.o \
./drivers/bsp/VL53L0X/vl53l0x_platform_log.o 

C_DEPS += \
./drivers/bsp/VL53L0X/stm32g4_vl53l0x_demo.d \
./drivers/bsp/VL53L0X/vl53l0x_api.d \
./drivers/bsp/VL53L0X/vl53l0x_api_calibration.d \
./drivers/bsp/VL53L0X/vl53l0x_api_core.d \
./drivers/bsp/VL53L0X/vl53l0x_api_ranging.d \
./drivers/bsp/VL53L0X/vl53l0x_api_strings.d \
./drivers/bsp/VL53L0X/vl53l0x_platform.d \
./drivers/bsp/VL53L0X/vl53l0x_platform_log.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/bsp/VL53L0X/%.o drivers/bsp/VL53L0X/%.su drivers/bsp/VL53L0X/%.cyclo: ../drivers/bsp/VL53L0X/%.c drivers/bsp/VL53L0X/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../core/Inc -I../drivers/stm32g4xx_hal/Inc -I../drivers/stm32g4xx_hal/Inc/Legacy -I../drivers/cmsis/Device/ST/STM32G4xx/Include -I../drivers/cmsis/Include -I../app -I../drivers/bsp -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-bsp-2f-VL53L0X

clean-drivers-2f-bsp-2f-VL53L0X:
	-$(RM) ./drivers/bsp/VL53L0X/stm32g4_vl53l0x_demo.cyclo ./drivers/bsp/VL53L0X/stm32g4_vl53l0x_demo.d ./drivers/bsp/VL53L0X/stm32g4_vl53l0x_demo.o ./drivers/bsp/VL53L0X/stm32g4_vl53l0x_demo.su ./drivers/bsp/VL53L0X/vl53l0x_api.cyclo ./drivers/bsp/VL53L0X/vl53l0x_api.d ./drivers/bsp/VL53L0X/vl53l0x_api.o ./drivers/bsp/VL53L0X/vl53l0x_api.su ./drivers/bsp/VL53L0X/vl53l0x_api_calibration.cyclo ./drivers/bsp/VL53L0X/vl53l0x_api_calibration.d ./drivers/bsp/VL53L0X/vl53l0x_api_calibration.o ./drivers/bsp/VL53L0X/vl53l0x_api_calibration.su ./drivers/bsp/VL53L0X/vl53l0x_api_core.cyclo ./drivers/bsp/VL53L0X/vl53l0x_api_core.d ./drivers/bsp/VL53L0X/vl53l0x_api_core.o ./drivers/bsp/VL53L0X/vl53l0x_api_core.su ./drivers/bsp/VL53L0X/vl53l0x_api_ranging.cyclo ./drivers/bsp/VL53L0X/vl53l0x_api_ranging.d ./drivers/bsp/VL53L0X/vl53l0x_api_ranging.o ./drivers/bsp/VL53L0X/vl53l0x_api_ranging.su ./drivers/bsp/VL53L0X/vl53l0x_api_strings.cyclo ./drivers/bsp/VL53L0X/vl53l0x_api_strings.d ./drivers/bsp/VL53L0X/vl53l0x_api_strings.o ./drivers/bsp/VL53L0X/vl53l0x_api_strings.su ./drivers/bsp/VL53L0X/vl53l0x_platform.cyclo ./drivers/bsp/VL53L0X/vl53l0x_platform.d ./drivers/bsp/VL53L0X/vl53l0x_platform.o ./drivers/bsp/VL53L0X/vl53l0x_platform.su ./drivers/bsp/VL53L0X/vl53l0x_platform_log.cyclo ./drivers/bsp/VL53L0X/vl53l0x_platform_log.d ./drivers/bsp/VL53L0X/vl53l0x_platform_log.o ./drivers/bsp/VL53L0X/vl53l0x_platform_log.su

.PHONY: clean-drivers-2f-bsp-2f-VL53L0X

