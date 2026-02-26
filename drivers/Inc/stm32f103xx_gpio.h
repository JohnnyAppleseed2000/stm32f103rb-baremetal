/*
 * stm32f103xx_gpio.h
 *
 *  Created on: Dec 31, 2025
 *      Author: John
 */

#ifndef INC_STM32F103XX_GPIO_H_
#define INC_STM32F103XX_GPIO_H_

#include "stm32f103xx.h"



/*
 * This is a configuration struct for GPIO pin
 */

typedef struct
 {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_Pin_Mode_Speed;
	uint8_t GPIO_OPMode;
	uint8_t GPIO_EXTI_EN;
	uint8_t GPIO_EXTI_RTFT;
}GPIO_PinConfig_t;


/*
 * Handle Structure for GPIO Pin
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;


// macros for GPIO input Operating mode
#define IN_ANALOG		0
#define IN_FLOAT		1
#define IN_PUPD			2
#define IN_RESERVED		3

// macros for GPIO output Operating mode
#define OP_GP_PP		0
#define OP_GP_OPDR		1
#define OP_AF_PP		2
#define OP_AF_OPDR		3

//macros for EXTI Trigger mode
#define EXTI_FT			0
#define EXTI_RT			1
#define EXTI_RTFT		2

// macros for mode&speed of GPIO
#define INPUT_MODE		0
#define OP_10MHZ		1
#define OP_2MHZ			2
#define OP_50MHZ		3

//macros for EXTI
#define EXTI_EN			ENABLE
#define EXTI_DI			DISABLE


/*****************************************************************************************
 * 									GPIO APIs supported by this driver
 *****************************************************************************************/

/*
 * Peripheral Clock setup
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init De-init
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data Read-Write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WritetoOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WritetoOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 *  IRQ Configuration and ISR Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);








#endif /* INC_STM32F103XX_GPIO_H_ */




