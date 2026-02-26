################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bsp/ds3231.c 

OBJS += \
./bsp/ds3231.o 

C_DEPS += \
./bsp/ds3231.d 


# Each subdirectory must supply rules for building sources it contributes
bsp/%.o bsp/%.su bsp/%.cyclo: ../bsp/%.c bsp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RBTx -DSTM32 -DSTM32F1 -DNUCLEO_F103RB -c -I../Inc -I"C:/Users/John.DESKTOP-ACCU4EE/Desktop/Embedded-C/My_Workspace/host/stm32f1xx_drivers/bsp" -I"C:/Users/John.DESKTOP-ACCU4EE/Desktop/Embedded-C/My_Workspace/host/stm32f1xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bsp

clean-bsp:
	-$(RM) ./bsp/ds3231.cyclo ./bsp/ds3231.d ./bsp/ds3231.o ./bsp/ds3231.su

.PHONY: clean-bsp

