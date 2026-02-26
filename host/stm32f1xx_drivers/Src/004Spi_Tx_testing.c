/*
 * 004Spi_Tx_testing.c
 *
 *  Created on: Jan 6, 2026
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
	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPI_PinConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPI_PinConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_PinConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; //sclk = HSI(8MHz) / 2(SPI Prescaler) = 4MHz
	SPI2handle.SPI_PinConfig.SPI_DFF = SPI_DFF_8BITS; // Data frame : 8bits
	SPI2handle.SPI_PinConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_PinConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_PinConfig.SPI_SSM = SPI_SSM_EN; // software slave management

	SPI_Init(&SPI2handle);
}


int main()
{
	char data[] = "Hello World";

	// initialize GPIO pins for SPI2
	SPI_GPIOInit();

	// initialize SPI2 configuration
	SPI2_Init();

	// Makes NSS internally high
	SSIControl(SPI2, ENABLE);

	// Enable SPI peripheral
	SPIPeriControl(SPI2, ENABLE);

	//Data Send
	SPI_SendData(SPI2, (uint8_t*)data, strlen(data));

	// Enable SPI peripheral
	SPIPeriControl(SPI2, DISABLE);

	while(1);

	return 0;

}
