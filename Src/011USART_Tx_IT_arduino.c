/*
 * 011USART_Tx_IT_arduino.c
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

char *msg[3] = {"hihihihihihi123", "Hello How are you ?" , "Today is Monday !"};
//reply from arduino will be stored here
uint8_t rx_buf[1024] ;

//This flag indicates reception completion
uint8_t rxCmplt = RESET;

uint8_t g_data = 0;

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

	//Rx Pin Configuration
	USARTPins.pGPIOx = GPIOA;
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = PIN_NUM_10;
	USARTPins.GPIO_PinConfig.GPIO_Pin_Mode_Speed = INPUT_MODE;
	USARTPins.GPIO_PinConfig.GPIO_OPMode = IN_PUPD;
	GPIO_Init(&USARTPins);

}

void USART1_Inits()
{
	//USART1 configuration
	USART1handle.pUSARTx = USART1;
	USART1handle.USART_Config.USART_Mode = USART_MODE_TXRX;
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
	//Set as Pull up
	GpioB1.pGPIOx->ODR |= (1 << PIN_NUM_13);

	GPIO_Init(&GpioB1);
}

int main(void)
{
	uint32_t cnt = 0;

	USART_GPIOInits();
	Button_Inits();
    USART1_Inits();

    USART_IRQInterruptConfig(IRQ_USART1,ENABLE);
    USART_PeripheralControl(USART1,ENABLE);

    printf("Application is running\n");

    while(1)
    {
		//wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, PIN_NUM_13) );
		//debouncing
		delay();

		// Next message index ; make sure that cnt value doesn't cross 2
		cnt = cnt % 3;

		//this code enables the receive interrupt
		while ( USART_ReceiveDataIT(&USART1handle,rx_buf,strlen(msg[cnt])) != USART_READY );

		//Send the msg indexed by cnt in blocking mode
    	USART_SendData(&USART1handle,(uint8_t*)msg[cnt],strlen(msg[cnt]));

    	printf("Transmitted : %s\n",msg[cnt]);


    	//Now lets wait until all the bytes are received from the arduino .
    	//When all the bytes are received rxCmplt will be SET in application callback
    	while(rxCmplt != SET);

    	//just make sure that last byte should be null otherwise %s fails while printing
    	rx_buf[strlen(msg[cnt])] = '\0';

    	//Print what we received from the arduino
    	printf("Received    : %s\n",rx_buf);

    	//invalidate the flag
    	rxCmplt = RESET;

    	//move on to next message indexed in msg[]
    	cnt ++;

    	//wait till button is pressed
		while(! GPIO_ReadFromInputPin(GPIOC, PIN_NUM_13) );
		//debouncing
		delay();
    }


	return 0;
}


void USART1_IRQHandler(void)
{
	USART_IRQHandling(&USART1handle);
}





void USART_ApplicationEventCallback( USART_Handle_t *pUSARTHandle,uint8_t ApEv)
{
   if(ApEv == USART_EVENT_RX_CMPLT)
   {
			rxCmplt = SET;

   }else if (ApEv == USART_EVENT_TX_CMPLT)
   {
	   ;
   }
}
