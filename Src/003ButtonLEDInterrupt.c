/*
 * 003ButtonLEDInterrupt.c
 *
 *  Created on: Jan 3, 2026
 *      Author: John
 */


#include <stdint.h>
#include "stm32f103xx.h"
#include "stm32f103xx_gpio.h"

void EXTI15_10_IRQHandler(void);

void delay()
{
	// debouncing 용도
	for(volatile uint32_t i=0; i<20000; i++);
}

int main()
{
	GPIO_Handle_t GpioLed;
	GPIO_Handle_t GpioB1;

	/*
	 * LED configuration (PA5)
	 */
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = PIN_NUM_5;
	GpioLed.GPIO_PinConfig.GPIO_Pin_Mode_Speed = OP_10MHZ;
	GpioLed.GPIO_PinConfig.GPIO_OPMode = OP_GP_PP;
	GpioLed.GPIO_PinConfig.GPIO_EXTI_EN = DISABLE;
	/*
	 * Button configuration (PC13)
	 */
	GpioB1.pGPIOx  = GPIOC;
	GpioB1.GPIO_PinConfig.GPIO_PinNumber = PIN_NUM_13;
	GpioB1.GPIO_PinConfig.GPIO_Pin_Mode_Speed = INPUT_MODE;
	GpioB1.GPIO_PinConfig.GPIO_OPMode = IN_PUPD;
	GpioB1.GPIO_PinConfig.GPIO_EXTI_EN = ENABLE;
	GpioB1.GPIO_PinConfig.GPIO_EXTI_RTFT = EXTI_FT;

	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioB1);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI5);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);
	EXTI15_10_IRQHandler();

	return 0;
}

void EXTI15_10_IRQHandler(void)
{
	//button debounce
	delay();

	//Handle IRQ PR
	GPIO_IRQHandling(PIN_NUM_13);


	GPIO_ToggleOutputPin(GPIOA, PIN_NUM_5);
}
