/*
 * 010USART_Tx_arduino.c
 *
 *  Created on: Jan 20, 2026
 *      Author: John
 */



#include <stdint.h>
#include <string.h>
#include "stm32f103xx.h"
#include "stm32f103xx_usart.h"
#include "stm32f103xx_gpio.h"

/*
 * PA9 USART1 Tx
 * PA10 USART1 Rx
 */

USART_Handle_t USART1handle;

void delay()
{
	for(uint32_t i = 0; i < 50000; i++);
}

void USART_GPIOInits()
{
	GPIO_Handle_t USARTPins;

	//Tx Pin Configuration
	USARTPins.pGPIOx = GPIOA;
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = PIN_NUM_9;
	USARTPins.GPIO_PinConfig.GPIO_Pin_Mode_Speed = OP_50MHZ;
	USARTPins.GPIO_PinConfig.GPIO_OPMode = OP_AF_PP;
	GPIO_Init(&USARTPins);

	/*Rx Pin Configuration
	USARTPins.pGPIOx = GPIOA;
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = PIN_NUM_3;
	USARTPins.GPIO_PinConfig.GPIO_Pin_Mode_Speed = INPUT_MODE;
	USARTPins.GPIO_PinConfig.GPIO_OPMode = IN_PUPD;
	GPIO_Init(&USARTPins);*/

}

void USART1_Inits()
{
	//USART1 configuration
	USART1handle.pUSARTx = USART1;
	USART1handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	USART1handle.USART_Config.USART_Baud = USART_STD_BAUD_9600;
	USART1handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART1handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART1handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART1handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USART_Init(&USART1handle);
}

void Button_Inits()
{
	//Button Configuration
	GPIO_Handle_t GpioB1;
	GpioB1.pGPIOx  = GPIOC;
	GpioB1.GPIO_PinConfig.GPIO_PinNumber = PIN_NUM_13;
	GpioB1.GPIO_PinConfig.GPIO_Pin_Mode_Speed = INPUT_MODE;
	GpioB1.GPIO_PinConfig.GPIO_OPMode = IN_PUPD;

	GPIO_Init(&GpioB1);
}

int main()
{
	char data[32] = "Hello World\n";

	// initialize GPIO pins for USART1
	USART_GPIOInits();

	// initialize B1
	Button_Inits();

	// initialize USART1 configuration
	USART1_Inits();

	USART_PeripheralControl(USART1, ENABLE);

	while(1)
	{
		/*
		 * Only enable USART2 peripheral when button is pressed
		 * Button에 pull up 저항이 달려있어 기본적으로 HIGH인 상태
		 */
		while (GPIO_ReadFromInputPin(GPIOC, PIN_NUM_13));
		// debounce
		delay();

		//Data Send
		USART_SendData(&USART1handle, (uint8_t*)data, strlen(data));


	}


	return 0;

}
