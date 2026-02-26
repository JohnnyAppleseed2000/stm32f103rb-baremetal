/*
 * stm32f103xx_i2c.c
 *
 *  Created on: Jan 14, 2026
 *      Author: John
 */

#include "stm32f103xx_i2c.h"
#include "stm32f103xx_rcc.h"

#include <stdio.h>

#define PCLK_SPEED 		8000000

static uint8_t I2C_GetStatusFlag(I2C_RegDef_t *pI2Cx, uint8_t status_flag);
static void I2C_SendAddressWrite(I2C_RegDef_t *pI2Cx, uint8_t slave_addr);
static void I2C_SendAddressRead(I2C_RegDef_t *pI2Cx, uint8_t slave_addr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_ACKControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);



/*
 * I2C related status flags definitions
 */
#define I2C_FLAG_TXE   		( 1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE   	( 1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB			( 1 << I2C_SR1_SB)
#define I2C_FLAG_OVR  		( 1 << I2C_SR1_OVR)
#define I2C_FLAG_AF   		( 1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO 		( 1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR 		( 1 << I2C_SR1_BERR)
#define I2C_FLAG_STOPF 		( 1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10 		( 1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF  		( 1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR 		( 1 << I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT 	( 1 << I2C_SR1_TIMEOUT)

/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		}
	} else {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_DI();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - I2C_PeripheralControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);

	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}



/*********************************************************************
 * @fn      		  - I2C_Init
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void I2C_Init(I2C_Handle_t *pI2CHandle) {
	uint32_t tempreg = 0;

	// configure the frequency by APB1 Bus speed
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / (1000000U);
	pI2CHandle->pI2Cx->CR2 = tempreg;

	// Device Address configuration
	tempreg = 0;
	tempreg |= ((pI2CHandle->I2C_Config.I2C_DeviceAddress << 1));
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	uint16_t ccr_value;
	tempreg = 0;

	// CCR calculation
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		// configure CCR value
		ccr_value = RCC_GetPCLK1Value()
				/ (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= ccr_value & 0xFFF;
	} else {
		tempreg |= (1 << I2C_CCR_FS);
		tempreg |= pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY;
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
			ccr_value = RCC_GetPCLK1Value()
					/ (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		} else if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_16_9) {
			ccr_value = RCC_GetPCLK1Value()
					/ (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		tempreg |= ccr_value & 0xFFF;
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		tempreg = (RCC_GetPCLK1Value() /1000000U) + 1 ;

	}else
	{
		//mode is fast mode
		tempreg = ( (RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1;

	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

	// enable Peripheral
	I2C_PeripheralControl(pI2CHandle->pI2Cx, ENABLE);

	// ack configuration
	tempreg=0;
	tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << I2C_CR1_ACK;
	tempreg |= (1 << 0);
	pI2CHandle->pI2Cx->CR1 = tempreg;
}

static uint8_t I2C_GetStatusFlag(I2C_RegDef_t *pI2Cx, uint8_t status_flag) {
	if (pI2Cx->SR1 & status_flag) {
		return FLAG_SET;
	} else {
		return FLAG_RESET;
	}
}

static void I2C_SendAddressWrite(I2C_RegDef_t *pI2Cx, uint8_t slave_addr) {
	slave_addr = slave_addr << 1;
	slave_addr &= ~(1 << 0);
	pI2Cx->DR = slave_addr;
}

static void I2C_SendAddressRead(I2C_RegDef_t *pI2Cx, uint8_t slave_addr) {
	slave_addr = slave_addr << 1;
	slave_addr |= (1 << 0);
	pI2Cx->DR = slave_addr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;
	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
	{
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize  == 1)
			{
				//first disable the ack
				I2C_ACKControl(pI2CHandle->pI2Cx,DISABLE);

				//clear the ADDR flag ( read SR1 , read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}

		}
		else
		{
			//clear the ADDR flag ( read SR1 , read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;

		}

	}
	else
	{
		//device is in slave mode
		//clear the ADDR flag ( read SR1 , read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}
}

void I2C_StopGeneration(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

static void I2C_ACKControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	} else if (EnorDi == DISABLE) {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	//6. Send Data until Len = 0
	while (pI2CHandle->TxLen > 0)
	{
		// wait until data register is empty
		while (!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
		pI2CHandle->TxLen--;
		pI2CHandle->pTxBuffer++;
	}
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if (pI2CHandle->RxSize == 1) {
		//Read data in the buffer DR

		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}

	if (pI2CHandle->RxSize > 1) {
		//read multiple bytes
		if (pI2CHandle->RxLen == 2) {
			// last 2 bytes remaining
			//clear the ack bit
			I2C_ACKControl(pI2CHandle->pI2Cx, DISABLE);
		}
		//read the data from data register DR into buffer
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
		// increment the buffer address
		pI2CHandle->pRxBuffer++;
		//decrement len
		pI2CHandle->RxLen--;
		}
	if(pI2CHandle->RxLen == 0 )
	{
		//close the I2C data reception and notify the application

		//1. generate the stop condition
		if(pI2CHandle->Sr == I2C_SR_DISABLE)
			I2C_StopGeneration(pI2CHandle->pI2Cx);

		//2 . Close the I2C rx
		I2C_CloseReceiveData(pI2CHandle);

		//3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
	}
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ACKControl(pI2CHandle->pI2Cx,ENABLE);
	}
}
/*********************************************************************
 * @fn      		  - I2C_MasterSendData
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer,
		uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {


	//1.Generate START condition
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);
	//2.confirm that start generation was completed
	while (!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_SB));
	//3. send the address of the slave with r/nw bit set to Write(0) (8bits)
	I2C_SendAddressWrite(pI2CHandle->pI2Cx, SlaveAddr);
	//4. wait until address is complete by checking ADDR flag in SR1
	while (!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_ADDR))
		;
	//5. Clear ADDR Flag by reading SR1 and SR2
	I2C_ClearADDRFlag(pI2CHandle);

	//6. Send Data until Len = 0
	while (Len > 0) {
		// wait until data register is empty
		while (!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		Len--;
		pTxbuffer++;
	}

	//7. wait until data register and shift register empty (TXE = 1, BTF = 1)
	while (!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

	while (!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//8. Generate Stop Condition
	if (Sr == I2C_SR_DISABLE)
	{
		I2C_StopGeneration(pI2CHandle->pI2Cx);
	}
}

/*********************************************************************
 * @fn      		  - I2C_MasterReceiveData
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr) {

	//1.Generate START condition
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);
	//2.confirm that start generation was completed
	while (!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_SB))
		;
	//3. send the address of the slave with r/nw bit set to Read(1) (8bits)
	I2C_SendAddressRead(pI2CHandle->pI2Cx, SlaveAddr);
	//4. wait until address is complete by checking ADDR flag in SR1
	while (!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_ADDR))
		;
	//5. Clear ADDR Flag by reading SR1 and SR2
	I2C_ClearADDRFlag(pI2CHandle);

	if (Len == 1) {
		// Read a single byte from Slave

		//1. Disable Acking
		I2C_ACKControl(pI2CHandle->pI2Cx, DISABLE);
		//2.Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);
		//3.wait until RXNE = 1
		while (! I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
		//4.Generate STOP condition
		if (Sr == I2C_SR_DISABLE)
		{
			I2C_StopGeneration(pI2CHandle->pI2Cx);
		}

		//.5Read data in the buffer DR
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	if (Len > 1) {
		//read multiple bytes
		// clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);
		//read the data until Len is zero
		for (uint32_t i = Len; i > 0; i--) {
			//wait until RXNE = 1
			while (! I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
			if (i == 2) {
				// last 2 bytes remaining

				//clear the ack bit
				I2C_ACKControl(pI2CHandle->pI2Cx, DISABLE);
				//generate STOP condition
				if (Sr == I2C_SR_DISABLE)
				{
				I2C_StopGeneration(pI2CHandle->pI2Cx);
				}
			}
			//read the data from data register DR into buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			// increment the buffer address
			pRxBuffer++;
		}
	}

	//re-enable ACKing
	I2C_ACKControl(pI2CHandle->pI2Cx, ENABLE);
}

/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		//1.Generate START condition
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;

}

/*********************************************************************
 * @fn      		  - I2C_MasterReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the below code . Also include the fn prototype in header file

 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->RxSize = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

/*********************************************************************
 * @fn      		  - I2C_MasterReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the below code . Also include the fn prototype in header file

 */

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1, temp2, temp3;
	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

	// 1. Handle for interrupt when SB is set
	if (temp2 && temp3)
	{
		/*
		 * Cleared by software by reading the SR1 register followed by writing the DR register,
		 * or by hardware when PE=0
		 */
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_SendAddressWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}else if( pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
		{
			I2C_SendAddressRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);

	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent and ACK is received
	//		 When Slave mode   : Address matched with own address
	if(temp2 && temp3)
	{
		/*
		 * This bit is cleared by software reading SR1 register followed reading SR2,
		 * or by hardware when PE=0.
		 */
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);

	//3. Handle for interrupt generated by BTF event
	//Note: Tx mode: Data transmission is finished
	//		Rx mode:

	if (temp2 && temp3)
	{
		/*
		 * Cleared by software reading SR1 followed by either a read or write in the DR register
		 * or by hardware after a start or a stop condition in transmission or when PE=0.
		 */
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			// make sure data register is empty
			if (pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE) )
			{
				//BTF, TXE = 1
				if(pI2CHandle->TxLen == 0 )
				{
					// 1. generate stop condition
					if(pI2CHandle->Sr == I2C_SR_DISABLE)
						I2C_StopGeneration(pI2CHandle->pI2Cx);
					// 2. reset all elements of handle structure
					I2C_CloseSendData(pI2CHandle);
					// 3. notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);
				}
			}
		}else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
		{
			;
		}
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF);
	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode

	if(temp2 && temp3)
	{
		//STOF flag is set
		//Clear the STOPF ( i.e 1) read SR1 2) Write to CR1 )

		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP);
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE);
	//5. Handle For interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		//Check for device mode (Master Mode)
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			//TXE flag is set
			//We have to do the data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}else
		{
			//slave
			//make sure that the slave is really in transmitter mode
			if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
			}
		}
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RXNE);
	//6. Handle For interrupt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{
		//check device mode .
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			//RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);

			}

		}else
		{
			//slave
			//make sure that the slave is really in receiver mode
			if(!(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_RCV);
			}
		}
	}

}

/*********************************************************************
 * @fn      		  - I2C_IRQInterruptConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
	}
}

/*********************************************************************
 * @fn      		  - I2C_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - __NVIC_PRIO_BITS) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}

/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the code also define these macros in the driver
						header file
						#define I2C_ERROR_BERR  3
						#define I2C_ERROR_ARLO  4
						#define I2C_ERROR_AF    5
						#define I2C_ERROR_OVR   6
						#define I2C_ERROR_TIMEOUT 7

 */

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}







