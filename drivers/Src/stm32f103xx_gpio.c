/*
 * stm32f103xx_gpio.c
 *
 *  Created on: Dec 31, 2025
 *      Author: John
 */


#include "stm32f103xx_gpio.h"
#define GPIO_BASEADDR_TO_CODE(x)   ( (x == GPIOA) ? 0 : \
									 (x == GPIOB) ? 1 : \
									 (x == GPIOC) ? 2 : \
									 (x == GPIOD) ? 3 : \
									 (x == GPIOE) ? 4 : \
									 (x == GPIOF) ? 5 : \
									 (x == GPIOG) ? 6 : 0 )

// GPIO 클럭 제어
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
	}else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
	}
}


// GPIO 초기화
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0; 
	uint8_t pin =  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

	// GPIO 클럭 활성화

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. Interrupt
	if (pGPIOHandle->GPIO_PinConfig.GPIO_EXTI_EN)
	{
		//AFIO Clock Enable
		AFIO_PCLK_EN();

		//Interrupt mode
		uint8_t exticr_idx = pin / 4;
		uint8_t exticr_shift = (pin % 4) * 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		// a. AFIO Register Configuration
		AFIO->EXTICR[exticr_idx] &= ~(0xF << exticr_shift);
		AFIO->EXTICR[exticr_idx] |=  (portcode << exticr_shift);

		// b. Interrupt Unmask
		EXTI->IMR |= (1 << pin);

		// c. Trigger Selection
		if (pGPIOHandle->GPIO_PinConfig.GPIO_EXTI_RTFT == EXTI_RT)
		{
			//Rising Trigger
			EXTI->RTSR |= (1 << pin);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_EXTI_RTFT == EXTI_FT)
		{
			//Falling Trigger
			EXTI->FTSR |= (1 << pin);
		}else
		{
			EXTI->RTSR |= (1 << pin);
			EXTI->FTSR |= (1 << pin);
		}
	}


	//2. [I/O & Speed & PullUp PullDown] configuration of GPIO
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < PIN_NUM_8) // Pin Number 0~7
	{
		//Output mode configuration
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_OPMode << (2 + (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->CRL &= ~(0x3 << (2 + (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
		pGPIOHandle->pGPIOx->CRL |= temp;
		temp = 0;
		//Speed Configuration
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_Pin_Mode_Speed << (4* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->CRL &= ~(0x3 << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->CRL |= temp;
		temp = 0;
	}else 	// Pin Number 8~15
	{
		//Output mode configuration
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_OPMode << (2 + (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8)));
		pGPIOHandle->pGPIOx->CRH &= ~(0x3 << (2 + (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8))));
		pGPIOHandle->pGPIOx->CRH |= temp;
		temp = 0;
		//Speed Configuration
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_Pin_Mode_Speed << (4* (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8));
		pGPIOHandle->pGPIOx->CRH &= ~(0x3 << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8)));
		pGPIOHandle->pGPIOx->CRH |= temp;
		temp = 0;
	}
}


// GPIO 
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){ 
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
}


// GPIO Pin 입력 함수
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x1);

	return value;
}


// GPIO Port 입력 함수
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)(pGPIOx->IDR);

	return value;
}


// GPIO Pin 출력 함수
void GPIO_WritetoOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
	if (Value == GPIO_PIN_SET)
	{
		// 1 출력
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{
		// 0출력 
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}


// GPIO Port 출력 함수
void GPIO_WritetoOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	// 값 출력
	pGPIOx->ODR = Value;
}


// GPIO pin 토글 함수
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


/*
 *  IRQ Configuration과 인터럽트 핸들 함수
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi)
	{
		// Cortex-M3 칩의 NVIC_ISER 레지스터를 통해 인터럽트 활성화
		if (IRQNumber < 32)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber >= 32 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 65 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}else
	{
		// 인터럽트 비활성화
		if (IRQNumber < 32)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber >= 32 && IRQNumber < 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 65 && IRQNumber < 96)
		{
			//program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

// 인터럽트 우선순위
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
		uint8_t iprx = IRQNumber / 4;
		uint8_t iprx_section  = IRQNumber %4 ;

		uint8_t shift_amount =  (8 * iprx_section) + (8 - __NVIC_PRIO_BITS);
		*(NVIC_PR_BASE_ADDR + iprx) |= IRQPriority << shift_amount;
}


void GPIO_IRQHandling(uint8_t PinNumber){
	// pending 레지스터의 해당 핀 값 삭제
	if(EXTI->PR & ( 1 << PinNumber))
	{
		//clear
		EXTI->PR |= ( 1 << PinNumber);
	}
}
