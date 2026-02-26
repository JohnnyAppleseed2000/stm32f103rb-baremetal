/*
 * 005Spi_Tx_arduino.c
 *
 *  Created on: Jan 7, 2026
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

	/*MISO Pin Configuration
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = PIN_NUM_14;
	SPIPins.GPIO_PinConfig.GPIO_Pin_Mode_Speed = INPUT_MODE;
	SPIPins.GPIO_PinConfig.GPIO_OPMode = IN_FLOAT;
	GPIO_Init(&SPIPins);*/

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
	char data[] = "Hello World";

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
		// debounce
		delay();

		// Enable SPI2 peripheral
		SPIPeriControl(SPI2, ENABLE);


		//first send length information
		uint8_t dataLen = strlen(data);
		SPI_SendData(SPI2,&dataLen,1);

		//Data Send
		SPI_SendData(SPI2, (uint8_t*)data, strlen(data));

		// confirm that SPI is not busy
		while (SPI_GetFlagStatus(SPI2, BSY_FLAG));

		// Disable SPI peripheral
		SPIPeriControl(SPI2, DISABLE);

		// wait until button is not pressed
		while(GPIO_ReadFromInputPin(GPIOC, PIN_NUM_13) == 0);

		// debounce
		delay();
	}


	return 0;

}

