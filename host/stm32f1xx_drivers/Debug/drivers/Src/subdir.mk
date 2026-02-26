################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f103xx_gpio.c \
../drivers/Src/stm32f103xx_i2c.c \
../drivers/Src/stm32f103xx_rcc.c \
../drivers/Src/stm32f103xx_spi.c \
../drivers/Src/stm32f103xx_usart.c 

OBJS += \
./drivers/Src/stm32f103xx_gpio.o \
./drivers/Src/stm32f103xx_i2c.o \
./drivers/Src/stm32f103xx_rcc.o \
./drivers/Src/stm32f103xx_spi.o \
./drivers/Src/stm32f103xx_usart.o 

C_DEPS += \
./drivers/Src/stm32f103xx_gpio.d \
./drivers/Src/stm32f103xx_i2c.d \
./drivers/Src/stm32f103xx_rcc.d \
./drivers/Src/stm32f103xx_spi.d \
./drivers/Src/stm32f103xx_usart.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su drivers/Src/%.cyclo: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RBTx -DSTM32 -DSTM32F1 -DNUCLEO_F103RB -c -I../Inc -I"C:/Users/John.DESKTOP-ACCU4EE/Desktop/Embedded-C/My_Workspace/host/stm32f1xx_drivers/bsp" -I"C:/Users/John.DESKTOP-ACCU4EE/Desktop/Embedded-C/My_Workspace/host/stm32f1xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/stm32f103xx_gpio.cyclo ./drivers/Src/stm32f103xx_gpio.d ./drivers/Src/stm32f103xx_gpio.o ./drivers/Src/stm32f103xx_gpio.su ./drivers/Src/stm32f103xx_i2c.cyclo ./drivers/Src/stm32f103xx_i2c.d ./drivers/Src/stm32f103xx_i2c.o ./drivers/Src/stm32f103xx_i2c.su ./drivers/Src/stm32f103xx_rcc.cyclo ./drivers/Src/stm32f103xx_rcc.d ./drivers/Src/stm32f103xx_rcc.o ./drivers/Src/stm32f103xx_rcc.su ./drivers/Src/stm32f103xx_spi.cyclo ./drivers/Src/stm32f103xx_spi.d ./drivers/Src/stm32f103xx_spi.o ./drivers/Src/stm32f103xx_spi.su ./drivers/Src/stm32f103xx_usart.cyclo ./drivers/Src/stm32f103xx_usart.d ./drivers/Src/stm32f103xx_usart.o ./drivers/Src/stm32f103xx_usart.su

.PHONY: clean-drivers-2f-Src

