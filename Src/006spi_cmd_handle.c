/*
 * 006spi_cmd_handle.c
 *
 *  Created on: Jan 12, 2026
 *      Author: John
 */


#include <stdint.h>
#include <string.h>
#include "stm32f103xx.h"
#include "stm32f103xx_spi.h"
#include "stm32f103xx_gpio.h"

/*
 * PB12 -> SPI2_NSS
 * PB13 -> SPI2_SCK
 * PB14 -> SPI2_MISO
 * PB15 -> SPI2_MOSI
 */

//command codes
#define COMMAND_LED_CTRL      		0x50
#define COMMAND_SENSOR_READ      	0x51
#define COMMAND_LED_READ      		0x52
#define COMMAND_PRINT      			0x53
#define COMMAND_ID_READ      		0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0 	0
#define ANALOG_PIN1 	1
#define ANALOG_PIN2 	2
#define ANALOG_PIN3 	3
#define ANALOG_PIN4 	4

//arduino led

#define LED_PIN  9

void delay()
{
	for(uint32_t i = 0; i < 50000; i++);
}

void SPI_GPIOInit()
{
	GPIO_Handle_t SPIPins;

	//NSS Pin configuration
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = PIN_NUM_12;
	SPIPins.GPIO_PinConfig.GPIO_Pin_Mode_Speed = OP_10MHZ;
	SPIPins.GPIO_PinConfig.GPIO_OPMode = OP_AF_PP;
	GPIO_Init(&SPIPins);

	//SCK Pin Configuration
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = PIN_NUM_13;
	SPIPins.GPIO_PinConfig.GPIO_Pin_Mode_Speed = OP_10MHZ;
	SPIPins.GPIO_PinConfig.GPIO_OPMode = OP_AF_PP;
	GPIO_Init(&SPIPins);

	//MISO Pin Configuration
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = PIN_NUM_14;
	SPIPins.GPIO_PinConfig.GPIO_Pin_Mode_Speed = INPUT_MODE;
	SPIPins.GPIO_PinConfig.GPIO_OPMode = IN_FLOAT;
	GPIO_Init(&SPIPins);

	//MOSI Pin Configuration
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = PIN_NUM_15;
	SPIPins.GPIO_PinConfig.GPIO_Pin_Mode_Speed = OP_10MHZ;
	SPIPins.GPIO_PinConfig.GPIO_OPMode = OP_AF_PP;
	GPIO_Init(&SPIPins);
}

void SPI2_Init()
{
	//SPI2 configuration
	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPI_PinConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPI_PinConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_PinConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32; //sclk = HSI(8MHz) / 32(SPI Prescaler)
	SPI2handle.SPI_PinConfig.SPI_DFF = SPI_DFF_8BITS; // Data frame : 8bits
	SPI2handle.SPI_PinConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_PinConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_PinConfig.SPI_SSM = SPI_SSM_DI; // Hardware slave management

	SPI_Init(&SPI2handle);
}

void Button_Init()
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
	uint8_t dummy_write = 0xFF;
	uint8_t dummy_read;

	// initialize GPIO pins for SPI2
	SPI_GPIOInit();

	// initialize SPI2 configuration
	SPI2_Init();

	// initialize B1
	Button_Init();

	/*
	 * Enable SS output
	 * NSS output = 0 when SPE = 1
	 * NSS output = 1 when SPE = 0
	 */
	SSOEControl(SPI2, ENABLE);

	while(1)
	{
		/*
		 * Only enable SPI2 peripheral when button is pressed
		 * Button에 pull up 저항이 달려있어 기본적으로 HIGH인 상태
		 */
		while (GPIO_ReadFromInputPin(GPIOC, PIN_NUM_13));


		// delay function to prevent button debounce
		delay();

		// Enable SPI2 peripheral
		SPIPeriControl(SPI2, ENABLE);

		//1. CMD_LED_CTRL  	<pin no(1)>     <value(1)>
		uint8_t cmnd_code = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		// send command code
		SPI_SendData(SPI2, &cmnd_code, 1);

		// empty the data recieve buffer
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// send dummy to recieve if ACK or NACK from arduino
		SPI_SendData(SPI2, &dummy_write, 1);

		// send dummy to recieve if ACK or NACK from arduino
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if( ackbyte == (uint8_t)0xF5 )
		{
			args[0] = LED_PIN;
			args[1] = LED_ON;

			// send parameters for cmnd
			SPI_SendData(SPI2, args, 2);
			// empty recieve buffer from arduino
			SPI_ReceiveData(SPI2, args, 2);

		}
	}
}
