/*
 * stm32f103xx_usart.c
 *
 *  Created on: Jan 17, 2026
 *      Author: John
 */


#include "stm32f103xx_usart.h"
#include "stm32f103xx_rcc.h"
#include <stdio.h>


/*********************************************************************
 * @fn      		  - USART_PeriClockControl
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
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}else if (pUSARTx == USART1)
		{
			USART2_PCLK_EN();
		}
	}else if(EnorDi == DISABLE)
	{
		if (pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}else if (pUSARTx == USART1)
		{
			USART2_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - USART_GetFlagStatus
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

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
	if (pUSARTx->SR & FlagName)
	{
		return FLAG_SET;
	}else
	{
		return FLAG_RESET;
	}
}

/*********************************************************************
 * @fn      		  - USART_PeripheralControl
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

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}else if (EnorDi == DISABLE)
	{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}

/*********************************************************************
 * @fn      		  - USART_SetBaudRate
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
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	//Variable to hold the APB clock
	uint32_t PCLKx;
	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

	uint32_t tempreg=0;

	//Get the value of APB bus clock in to the variable PCLKx
	if(pUSARTx == USART1)
	{
	   PCLKx = RCC_GetPCLK2Value();
	}else
	{
	   PCLKx = RCC_GetPCLK1Value();
	}

	//over sampling by 16 (stm32f1 시리즈는 고정으로 over16)
	usartdiv = ((25 * PCLKx) / (4 *BaudRate));

	//Calculate the Mantissa part
	M_part = usartdiv/100;

	//Place the Mantissa part in appropriate bit position . refer USART_BRR
	tempreg |= M_part << 4;

	//Extract the fraction part
	F_part = (usartdiv - (M_part * 100));
	//round up F part
	F_part = (((F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);
	tempreg |= F_part;

	//copy the value of tempreg in to BRR register
	pUSARTx->BRR = tempreg;
}

/*********************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{

	//Temporary variable
	uint32_t tempreg=0;

/******************************** Configuration of CR1******************************************/

	//Implement the code to enable the Clock for given USART peripheral
	 USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Implement the code to enable the Receiver bit field
		tempreg|= (1 << USART_CR1_RE);
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//Implement the code to enable the Transmitter bit field
		tempreg |= ( 1 << USART_CR1_TE );

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//Implement the code to enable the both Transmitter and Receiver bit fields
		tempreg |= ( ( 1 << USART_CR1_TE) | ( 1 << USART_CR1_RE) );
	}

    //Implement the code to configure the Word length configuration item
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M ;


    //Configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//Implement the code to enale the parity control
		tempreg |= ( 1 << USART_CR1_PCE);

		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		//Implement the code to enable the parity control
	    tempreg |= ( 1 << USART_CR1_PCE);

	    //Implement the code to enable ODD parity
	    tempreg |= ( 1 << USART_CR1_PS);

	}

   //Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg=0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//Implement the code to enable CTS flow control
		tempreg |= ( 1 << USART_CR3_CTSE);


	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//Implement the code to enable RTS flow control
		tempreg |= ( 1 << USART_CR3_RTSE);

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control
		tempreg |= (( 1 << USART_CR3_CTSE) | ( 1 << USART_CR3_RTSE));
	}


	pUSARTHandle->pUSARTx->CR3 = tempreg;

	//set baud rate
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}

/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			pUSARTHandle->pUSARTx->DR = *(uint16_t*)pTxBuffer & 0x01FF;
			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				pTxBuffer++;
			}

		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);
			pTxBuffer++;
		}
	}

	//wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}

/*********************************************************************
 * @fn      		  - USART_ReceiveData
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

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_RXNE));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//9bits will be user data
				*(uint16_t*)pRxBuffer = pUSARTHandle->pUSARTx->DR & 0x1FF;
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
				pRxBuffer++;
			}
		}
		else
		{
			//8bit data in a frame

			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
			}

			else
			{
				//7 bits will be of user data and 1 bit is parity

				//read only 7 bits
				 *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & 0x7F);
			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}
}

/*********************************************************************
 * @fn      		  - USART_SendDataWithIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t tx_state = pUSARTHandle->TxBusyState;

	if (tx_state != USART_BUSY_IN_TX)
	{
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		// TXE 와 TC 의 interrupt enable
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);

	}

	return tx_state;
}

/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rx_state = pUSARTHandle->RxBusyState;

	if (rx_state != USART_BUSY_IN_RX)
	{
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		// RXNEIE interrupt mode enable
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);

	}

	return rx_state;
}



/*********************************************************************
 * @fn      		  - USART_IRQHandling
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
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t temp1, temp2;
	uint16_t pdata;

	/***********************************************************************
	 * interrupt for TCIE
	 ************************************************************************/
	temp1 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCIE);
	temp2 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC);

	if(temp1 && temp2 )
	{
		//close transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(! pUSARTHandle->TxLen )
			{
				pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC);
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TCIE);
				pUSARTHandle->pTxBuffer = NULL;
				pUSARTHandle->TxLen = 0;
				pUSARTHandle->TxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
			}
		}
	}

	/***********************************************************************
	 * interrupt for TXEIE
	 ************************************************************************/
	temp1 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);
	temp2 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE);
	if (temp1 && temp2)
	{
		if (pUSARTHandle->TxLen > 0)
		{
			//check the word length
			if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
			{
				// 9 bits, check parity
				pdata = *(uint16_t*)(pUSARTHandle->pTxBuffer);
				pUSARTHandle->pUSARTx->DR = pdata & (uint16_t)0x1FF;
				if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					// parity disabled, 9bits of data
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen--;
				}else
				{
					// parity enabled, 8bits of data and 1 parity
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen--;

				}
			}else
			{
				//8bits
				pUSARTHandle->pUSARTx->DR = *(pUSARTHandle->pTxBuffer) & (uint8_t)0xFF;
				pUSARTHandle->pTxBuffer++;
				pUSARTHandle->TxLen--;
			}
		}
		if (! pUSARTHandle->TxLen)
		{
			//disable txe interrupt flag
			pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
		}
	}

	/***********************************************************************
	 * interrupt for RXNEIE
	 ************************************************************************/
	temp1 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);
	temp2 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE);
	if(temp1 && temp2)
	{
		if (pUSARTHandle->RxLen > 0)
		{
			//check the word length
			if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
			{
				if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					// parity disabled, 9bits of data
					*(uint16_t*)(pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x1FF);
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen--;
				}else
				{
					// parity enabled, 8bits of data and 1 parity
					*(uint16_t*)(pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0xFF);
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen--;

				}
			}else
			{
				//8bits of data
				*(uint16_t*)(pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0xFF);
				pUSARTHandle->pRxBuffer++;
				pUSARTHandle->RxLen--;
			}
		}
		if(! pUSARTHandle->RxLen)
		{
			//disable rxne interrupt flag
			pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
			pUSARTHandle->RxBusyState = USART_READY;
			USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
		}
	}


}


/*********************************************************************
 * @fn      		  - USART_IRQInterruptConfig
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
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if ( IRQNumber < 32 ) // 0~31
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if ( IRQNumber >= 32 && IRQNumber < 64 ) // 32~63
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
	}else
	{
		if ( IRQNumber < 32 ) // 0~31
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if ( IRQNumber >= 32 && IRQNumber < 64 ) // 32~63
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));

		}
	}
}

/*********************************************************************
 * @fn      		  - USART_IRQPriorityConfig
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
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section  = IRQNumber %4;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - __NVIC_PRIO_BITS);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}





















