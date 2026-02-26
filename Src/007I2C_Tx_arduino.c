/*
 * 007I2C_Tx_arduino.c
 *
 *  Created on: Jan 15, 2026
 *      Author: John
 */


#include <stdint.h>
#include <string.h>
#include "stm32f103xx.h"
#include "stm32f103xx_i2c.h"
#include "stm32f103xx_gpio.h"

#define MY_ADDR		0x52
#define SLAVE_ADDR 	0x68

I2C_Handle_t I2C1handle;

void delay()
{
	for(uint32_t i = 0; i < 50000; i++);
}

/*
 * PB6 -> SCL
 * PB7 -> SDA
 */

void I2C_GPIOInits()
{
	GPIO_Handle_t I2CPins;

	//SCL Pin Configuration
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = PIN_NUM_6;
	I2CPins.GPIO_PinConfig.GPIO_Pin_Mode_Speed = OP_10MHZ;
	I2CPins.GPIO_PinConfig.GPIO_OPMode = OP_AF_OPDR;
	GPIO_Init(&I2CPins);

	//SDA Pin Configuration
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = PIN_NUM_7;
	I2CPins.GPIO_PinConfig.GPIO_Pin_Mode_Speed = OP_10MHZ;
	I2CPins.GPIO_PinConfig.GPIO_OPMode = OP_AF_OPDR;
	GPIO_Init(&I2CPins);
}

void I2C1_Inits()
{
	//I2C1 configuration
	I2C1handle.pI2Cx = I2C1;
	I2C1handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
	I2C1handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C_Init(&I2C1handle);
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
	uint8_t data[] = "Hello World\n";

	// Enable peripheral clock
	I2C_PeriClockControl(I2C1, ENABLE);

	// initialize GPIO pins for I2C1
	I2C_GPIOInits();

	// initialize B1
	Button_Init();

	// initialize I2C1 configuration
	I2C1_Inits();


	while(1)
	{

		/*
		 * Only enable I2C2 peripheral when button is pressed
		 * Button에 pull up 저항이 달려있어 기본적으로 HIGH인 상태
		 */
		while (GPIO_ReadFromInputPin(GPIOC, PIN_NUM_13));
		// debouncing
		delay();

		I2C_MasterSendData(&I2C1handle, data, strlen((char*)data), SLAVE_ADDR, I2C_SR_DISABLE);


	}


	return 0;

}
