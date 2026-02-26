/*
 * 001ToggleLED.c
 *
 *  Created on: Jan 1, 2026
 *      Author: John
 */
#include <stdint.h>
#include "stm32f103xx.h"
#include "stm32f103xx_gpio.h"

void delay()
{
	for (uint32_t i=0; i<500000; i++);
}

int main()
{
	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = PIN_NUM_5;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = OP_10MHZ;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = OP_GP_PP;

	GPIO_Init(&GpioLed);
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, PIN_NUM_5);
		delay();
	}

	return 0;
}
