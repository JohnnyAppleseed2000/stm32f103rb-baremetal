/*
 * 002ButtonLED.c
 *
 *  Created on: Jan 1, 2026
 *      Author: John
 */

#include <stdint.h>
#include "stm32f103xx.h"
#include "stm32f103xx_gpio.h"


int main()
{
	uint8_t flag;

	GPIO_Handle_t GpioLed;
	GPIO_Handle_t GpioB1;

	/*
	 * LED configuration (PA5)
	 */
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = PIN_NUM_5;
	GpioLed.GPIO_PinConfig.GPIO_Pin_Mode_Speed = OP_10MHZ;
	GpioLed.GPIO_PinConfig.GPIO_OPMode = OP_GP_PP;

	/*
	 * Button configuration (PC13)
	 */
	GpioB1.pGPIOx  = GPIOC;
	GpioB1.GPIO_PinConfig.GPIO_PinNumber = PIN_NUM_13;
	GpioB1.GPIO_PinConfig.GPIO_Pin_Mode_Speed = INPUT_MODE;
	GpioB1.GPIO_PinConfig.GPIO_OPMode = IN_PUPD;

	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioB1);

	while(1)
	{
		flag = GPIO_ReadFromInputPin(GPIOC, PIN_NUM_13);
		if (flag)
		{
			GPIO_WritetoOutputPin(GPIOA, PIN_NUM_5, GPIO_PIN_SET);
		}else
		{
			GPIO_WritetoOutputPin(GPIOA, PIN_NUM_5, GPIO_PIN_RESET);
		}

	}

	return 0;
}
