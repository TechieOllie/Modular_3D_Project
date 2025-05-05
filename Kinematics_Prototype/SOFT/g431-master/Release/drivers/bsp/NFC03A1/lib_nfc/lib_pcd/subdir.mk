################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso14443Apcd.c \
../drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso14443Bpcd.c \
../drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso15693pcd.c \
../drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso18092pcd.c \
../drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso7816pcd.c \
../drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype1pcd.c \
../drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype2pcd.c \
../drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype3pcd.c \
../drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype4pcd.c \
../drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype5pcd.c \
../drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_pcd.c 

OBJS += \
./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso14443Apcd.o \
./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso14443Bpcd.o \
./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso15693pcd.o \
./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso18092pcd.o \
./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso7816pcd.o \
./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype1pcd.o \
./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype2pcd.o \
./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype3pcd.o \
./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype4pcd.o \
./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype5pcd.o \
./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_pcd.o 

C_DEPS += \
./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso14443Apcd.d \
./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso14443Bpcd.d \
./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso15693pcd.d \
./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso18092pcd.d \
./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso7816pcd.d \
./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype1pcd.d \
./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype2pcd.d \
./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype3pcd.d \
./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype4pcd.d \
./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype5pcd.d \
./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_pcd.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/bsp/NFC03A1/lib_nfc/lib_pcd/%.o drivers/bsp/NFC03A1/lib_nfc/lib_pcd/%.su drivers/bsp/NFC03A1/lib_nfc/lib_pcd/%.cyclo: ../drivers/bsp/NFC03A1/lib_nfc/lib_pcd/%.c drivers/bsp/NFC03A1/lib_nfc/lib_pcd/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../core/Inc -I../drivers/stm32g4xx_hal/Inc -I../drivers/stm32g4xx_hal/Inc/Legacy -I../drivers/cmsis/Device/ST/STM32G4xx/Include -I../drivers/cmsis/Include -I../app -I../drivers/bsp -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-bsp-2f-NFC03A1-2f-lib_nfc-2f-lib_pcd

clean-drivers-2f-bsp-2f-NFC03A1-2f-lib_nfc-2f-lib_pcd:
	-$(RM) ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso14443Apcd.cyclo ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso14443Apcd.d ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso14443Apcd.o ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso14443Apcd.su ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso14443Bpcd.cyclo ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso14443Bpcd.d ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso14443Bpcd.o ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso14443Bpcd.su ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso15693pcd.cyclo ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso15693pcd.d ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso15693pcd.o ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso15693pcd.su ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso18092pcd.cyclo ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso18092pcd.d ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso18092pcd.o ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso18092pcd.su ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso7816pcd.cyclo ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso7816pcd.d ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso7816pcd.o ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_iso7816pcd.su ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype1pcd.cyclo ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype1pcd.d ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype1pcd.o ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype1pcd.su ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype2pcd.cyclo ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype2pcd.d ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype2pcd.o ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype2pcd.su ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype3pcd.cyclo ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype3pcd.d ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype3pcd.o ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype3pcd.su ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype4pcd.cyclo ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype4pcd.d ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype4pcd.o ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype4pcd.su ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype5pcd.cyclo ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype5pcd.d ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype5pcd.o ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_nfctype5pcd.su ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_pcd.cyclo ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_pcd.d ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_pcd.o ./drivers/bsp/NFC03A1/lib_nfc/lib_pcd/lib_pcd.su

.PHONY: clean-drivers-2f-bsp-2f-NFC03A1-2f-lib_nfc-2f-lib_pcd

