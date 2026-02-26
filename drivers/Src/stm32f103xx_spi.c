/*
 * stm32f103xx_spi.c
 *
 *  Created on: Jan 5, 2026
 *      Author: John
 */


#include "stm32f103xx_spi.h"
#include <stddef.h>

static void spi_txe_interrupt_handler();
static void spi_rxne_interrupt_handler();
static void spi_ovr_interrupt_handler();

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
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

void SPIPeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}else
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - This function initializes SPI configuration
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t tempreg = 0;

	//enable peripheral clock
	SPIPeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//1. Device Mode
	tempreg |= pSPIHandle->SPI_PinConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Bus Configuration
	if (pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// Clear BiDi mode
		pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// Set BiDi Mode
		pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// clear bidi mode
		pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIMODE);
		// Output disabled (Recieve only)
		pSPIHandle->pSPIx->CR1 &= (1 << SPI_CR1_BIDIOE);
	}else if (pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_TXONLY)
	{
		// clear bidi mode
		pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIMODE);
		// Output enabled (transmit only)
		pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_BIDIOE);
	}

	//3. configure spi clock speed (baud rate)
	tempreg |= pSPIHandle->SPI_PinConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. configure the date frame format
	tempreg |= pSPIHandle->SPI_PinConfig.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL & CPHA
	tempreg |= pSPIHandle->SPI_PinConfig.SPI_CPOL << SPI_CR1_CPOL;
	tempreg |= pSPIHandle->SPI_PinConfig.SPI_CPHA << SPI_CR1_CPHA;

	//6. configure the software slave management
	tempreg |= pSPIHandle->SPI_PinConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

/*********************************************************************
 * @fn      		  - SPIPeriControl
 *
 * @brief             - This function initializes SPI configuration
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void SPIPeriControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*********************************************************************
 * @fn      		  - SSIControl
 *
 * @brief             - This function initializes SPI configuration
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void SSIControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/*********************************************************************
 * @fn      		  - SSOEControl
 *
 * @brief             - This function initializes SPI configuration
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void SSOEControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}

}
/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             - This function de-initializes SPI configuration
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}

/*********************************************************************
 * @fn      		  - SPI_GetFlagStatus
 *
 * @brief             - This function de-initializes SPI configuration
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t Flagname)
{
	if(pSPIx->SR & Flagname)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - This function de-initializes SPI configuration
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		//1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, TXE_FLAG) == FLAG_RESET);

		//2. Tx_buffer written to Date register
		if ((pSPIx->CR1) & (1 << SPI_CR1_DFF))
		{
			// 16bit transmission
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			pTxBuffer += 2;
			Len -= 2;
		}else
		{
			//8-bit transmission
			pSPIx->DR = *pTxBuffer;
			pTxBuffer += 1;
			Len -= 1;
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_RecieveData
 *
 * @brief             - This function de-initializes SPI configuration
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		//1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, RXNE_FLAG) == FLAG_RESET);

		//2. Tx_buffer written to Date register
		if ((pSPIx->CR1) & (1 << SPI_CR1_DFF))
		{
			// 16bit transmission
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			pRxBuffer += 2;
			Len -= 2;
		}else
		{
			//8-bit transmission
			*pRxBuffer = pSPIx->DR;
			pRxBuffer += 1;
			Len -= 1;
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_SendDataIT
 *
 * @brief             - This function de-initializes SPI configuration
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if ( state !=  SPI_BUSY_IN_TX)
	{
		// 1. Save the TxBuffer and Length
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		// 2. Save the SPI state as busy
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		// 3. TXEIE to 1
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	// 4. Data transmission handled in ISR code

	return state;
}

/*********************************************************************
 * @fn      		  - SPI_RecieveDataIT
 *
 * @brief             - This function de-initializes SPI configuration
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if ( state != SPI_BUSY_IN_RX)
	{
		// 1. Save the TxBuffer and Length
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		// 2. Save the SPI state as busy
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		// 3. RXNEIE to 1
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	// 4. Data transmission handled in ISR code

	return state;
}

/*********************************************************************
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             - This function de-initializes SPI configuration
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	// check for txe
	if( temp1 && temp2 )
	{
		// TXE handle function
		spi_txe_interrupt_handler(pSPIHandle);
	}

	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	//check for rxne
	if( temp1 && temp2 )
	{
		// RXNE handle function
		spi_rxne_interrupt_handler(pSPIHandle);
	}

	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	// check for ovr flag
	if( temp1 && temp2 )
	{
		// TXE handle function
		spi_ovr_interrupt_handler(pSPIHandle);
	}
}

// helper functions
static void spi_txe_interrupt_handler(SPI_Handle_t *pSPIHandle)
{
	//1. Tx_buffer written to Data register
	if ((pSPIHandle->pSPIx->CR1) & (1 << SPI_CR1_DFF))
	{
		// 16bit transmission
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->pTxBuffer += 2;
		pSPIHandle->TxLen -= 2;
	}else
	{
		//8-bit transmission
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->pTxBuffer += 1;
		pSPIHandle->TxLen -= 1;
	}

	if (! pSPIHandle->TxLen)
	{
		//turn off transmission
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handler(SPI_Handle_t *pSPIHandle)
{
	//1. Rx_buffer written to Data register
	if ((pSPIHandle->pSPIx->CR1) & (1 << SPI_CR1_DFF))
	{
		// 16bit transmission
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->pRxBuffer += 2;
		pSPIHandle->RxLen -= 2;
	}else
	{
		//8-bit transmission
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->pRxBuffer += 1;
		pSPIHandle->RxLen -= 1;
	}

	if (! pSPIHandle->RxLen)
	{
		//turn off reception
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_interrupt_handler(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	// 1. clear the ovr flag
	if ( pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	// 2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEvent)
{
	// weak implementation
}




