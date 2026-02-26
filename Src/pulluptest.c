#include <stdint.h>
#include <string.h>
#include "stm32f103xx.h"
#include "stm32f103xx_i2c.h"
#include "stm32f103xx_gpio.h"


void GPIOInits()
{
	GPIO_Handle_t I2CPins;

	//SCL Pin Configuration
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = PIN_NUM_6;
	I2CPins.GPIO_PinConfig.GPIO_Pin_Mode_Speed = OP_10MHZ;
	I2CPins.GPIO_PinConfig.GPIO_OPMode = OP_GP_PP;
	GPIO_Init(&I2CPins);

	//SCL Pin Configuration
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = PIN_NUM_7;
	I2CPins.GPIO_PinConfig.GPIO_Pin_Mode_Speed = OP_10MHZ;
	I2CPins.GPIO_PinConfig.GPIO_OPMode = OP_GP_PP;
	GPIO_Init(&I2CPins);
}



int main()
{
	GPIOInits();

	GPIO_WritetoOutputPin(GPIOB, PIN_NUM_7, 1);

	GPIO_WritetoOutputPin(GPIOB, PIN_NUM_6, 1);

	while(1);


}
