/*
 * stm32f103xx_spi.h
 *
 *  Created on: Jan 5, 2026
 *      Author: John
 */

#ifndef INC_STM32F103XX_SPI_H_
#define INC_STM32F103XX_SPI_H_
#define __weak __attribute__((weak))

#include "stm32f103xx.h"
#include <stdint.h>

/*
 * This is a configuration struct for SPI pin
 */

typedef struct
 {
	uint8_t SPI_DeviceMode;		//Master or Slave mode selection
	uint8_t SPI_BusConfig;		// Bus configuration
	uint8_t SPI_CPHA;			// Clock phase (0: data capture at first edge, 1: data capture at second edge)
	uint8_t SPI_CPOL;			// Clock polarity (0: Clk 0 when idle, 1: Clk 1 when idle)
	uint8_t SPI_SclkSpeed;		// Baud rate control
	uint8_t SPI_DFF;			//  Data frame format
	uint8_t SPI_SSM;
}SPI_PinConfig_t;

/*
 * Handle Structure for SPI Pin
 */

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_PinConfig_t SPI_PinConfig;
	uint8_t 		*pTxBuffer; /* !< To store the app. Tx buffer address > */
	uint8_t 		*pRxBuffer;	/* !< To store the app. Rx buffer address > */
	uint32_t 		TxLen;		/* !< To store Tx len > */
	uint32_t 		RxLen;		/* !< To store Tx len > */
	uint8_t 		TxState;	/* !< To store Tx state > */
	uint8_t 		RxState;	/* !< To store Rx state > */
}SPI_Handle_t;


/*
 * SPI application states
 */
#define SPI_READY 					0
#define SPI_BUSY_IN_RX 				1
#define SPI_BUSY_IN_TX 				2

/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT   1
#define SPI_EVENT_RX_CMPLT   2
#define SPI_EVENT_OVR_ERR    3

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER    1
#define SPI_DEVICE_MODE_SLAVE     0


/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD                1
#define SPI_BUS_CONFIG_HD                2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY    3
#define SPI_BUS_CONFIG_SIMPLEX_TXONLY    4

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2             	0
#define SPI_SCLK_SPEED_DIV4             	1
#define SPI_SCLK_SPEED_DIV8             	2
#define SPI_SCLK_SPEED_DIV16             	3
#define SPI_SCLK_SPEED_DIV32             	4
#define SPI_SCLK_SPEED_DIV64             	5
#define SPI_SCLK_SPEED_DIV128             	6
#define SPI_SCLK_SPEED_DIV256             	7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS 	0
#define SPI_DFF_16BITS  1

/*
 * @CPOL
 */
#define SPI_CPOL_HIGH 1
#define SPI_CPOL_LOW 0

/*
 * @CPHA
 */
#define SPI_CPHA_HIGH 1
#define SPI_CPHA_LOW 0

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN     1
#define SPI_SSM_DI     0

/*
 * @SPI_TX
 */
#define SPI_TX_EMPTY    1
#define SPI_TX_NEMPTY   0

/*
 * @SPI_RX
 */
#define SPI_RX_EMPTY    0
#define SPI_RX_NEMPTY   1





/*****************************************************************************************
 * 								SPI APIs supported by this driver
 *****************************************************************************************/


/*
 * Peripheral Clock setup
 */
void SPIPeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init De-init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Check the Flag from Status regiser
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t Flagname);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 *  IRQ Configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * Other peripheral control APIs
 */
void SPIPeriControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SSIControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SSOEControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 *  Apllication callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEvent);

#endif /* INC_STM32F103XX_SPI_H_ */
