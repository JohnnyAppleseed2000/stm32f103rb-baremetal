/*
 * stm32f103xx.h
 *
 *  Created on: Dec 31, 2025
 *      Author: John
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

#include <stddef.h>
#include <stdint.h>

#define _vo volatile

/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex M3 Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ( (_vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (_vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (_vo uint32_t*)0xE000E108 )


/*
 * ARM Cortex M3 Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((_vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((_vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((_vo uint32_t*)0XE000E188)

/*
 * ARM Cortex Mx Processor NVIC IPRx register Addresses
 */
#define NVIC_IPR0 			((_vo uint32_t*)0XE000E400)
#define NVIC_IPR1			((_vo uint32_t*)0XE000E404)
#define NVIC_IPR2  			((_vo uint32_t*)0XE000E408)

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((_vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor priority offset bits
 */
#define __NVIC_PRIO_BITS	4

/*
 * ARM Cortex Mx Processor priority numbers
 */
#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15

/*
 * base address of flash and sram
 */

#define FLASH_BASE_ADDR					0x08000000UL
#define SRAM_BASE_ADDR					0x20000000UL
#define ROM								0x1FFFE000UL


/*
 * base address of RCC, APB1, APB2, AHB
 */

#define RCC_BASE_ADDR					0x40021000UL
#define APB1_BASE_ADDR 					0x40000000UL
#define APB2_BASE_ADDR 					0x40010000UL
#define AHB_BASE_ADDR 					0x40018000UL

/*
 * base address of peripherals connected to APB2 bus
 */
#define AFIO_BASE_ADDR					(APB2_BASE_ADDR + 0x0UL)
#define EXTI_BASE_ADDR					(APB2_BASE_ADDR + 0x400UL)

#define GPIOA_BASE_ADDR					(APB2_BASE_ADDR + 0x800UL)
#define GPIOB_BASE_ADDR					(APB2_BASE_ADDR + 0xC00UL)
#define GPIOC_BASE_ADDR					(APB2_BASE_ADDR + 0x1000UL)
#define GPIOD_BASE_ADDR					(APB2_BASE_ADDR + 0x1400UL)
#define GPIOE_BASE_ADDR					(APB2_BASE_ADDR + 0x1800UL)
#define GPIOF_BASE_ADDR					(APB2_BASE_ADDR + 0x1C00UL)
#define GPIOG_BASE_ADDR					(APB2_BASE_ADDR + 0x2000UL)

#define ADC1_BASE_ADDR					(APB2_BASE_ADDR + 0x2400UL)
#define ADC2_BASE_ADDR					(APB2_BASE_ADDR + 0x2800UL)
#define TIM1_BASE_ADDR					(APB2_BASE_ADDR + 0x2C00UL)
#define SPI1_BASE_ADDR					(APB2_BASE_ADDR + 0x3000UL)
#define TIM8_BASE_ADDR					(APB2_BASE_ADDR + 0x3400UL)
#define USART1_BASE_ADDR				(APB2_BASE_ADDR + 0x3800UL)
#define ADC3_BASE_ADDR					(APB2_BASE_ADDR + 0x3C00UL)
//reserved
#define TIM9_BASE_ADDR					(APB2_BASE_ADDR + 0x4C00UL)
#define TIM10_BASE_ADDR					(APB2_BASE_ADDR + 0x5000UL)
#define TIM11_BASE_ADDR					(APB2_BASE_ADDR + 0x5400UL)

/*
 * base address of peripherals connected to APB1 bus
 */
#define SPI2_BASE_ADDR					(APB1_BASE_ADDR + 0x3800UL)
#define SPI3_BASE_ADDR					(APB1_BASE_ADDR + 0x3C00UL)
#define I2C1_BASE_ADDR					(APB1_BASE_ADDR + 0x5400UL)
#define I2C2_BASE_ADDR					(APB1_BASE_ADDR + 0x5800UL)
#define USART2_BASE_ADDR				(APB1_BASE_ADDR + 0x4400UL)
#define USART3_BASE_ADDR				(APB1_BASE_ADDR + 0x4800UL)

/*
 * peripheral register definition structure of GPIO
 */

typedef struct
{
	_vo uint32_t CRL;		/*Port configuration register low								*/
	_vo uint32_t CRH;		/*Port configuration register high 								*/
	_vo uint32_t IDR;		/*Port input data register										*/
	_vo uint32_t ODR;		/*Port output data register										*/
	_vo uint32_t BSRR;		/*Port bit set/reset register									*/
	_vo uint32_t BRR;		/*Port bit reset register										*/
	_vo uint32_t LCKR;		/*Port configuration lock register								*/
}GPIO_RegDef_t;

/*
 * peripheral register definition structure of RCC
 */

typedef struct
{
	_vo uint32_t CR;			//Clock control register
	_vo uint32_t CFGR;			//Clock configuration register
	_vo uint32_t CIR;			//Clock interrupt register
	_vo uint32_t APB2RSTR;		//APB2 peripheral reset register
	_vo uint32_t APB1RSTR;		//APB1 peripheral reset register
	_vo uint32_t AHBENR;		//AHB Peripheral Clock enable register
	_vo uint32_t APB2ENR;		//APB2 peripheral clock enable register
	_vo uint32_t APB1ENR;		//APB1 peripheral clock enable register
	_vo uint32_t BDCR;			//Port configuration lock register
	_vo uint32_t CSR;			//Port configuration lock register
}RCC_RegDef_t;

/*
 * peripheral register definition structure of EXTI
 */

typedef struct
{
	_vo uint32_t IMR;			//Interrupt mask register
	_vo uint32_t EMR;			//Event mask register
	_vo uint32_t RTSR;			//Rising trigger selection register
	_vo uint32_t FTSR;			//Falling trigger selection register
	_vo uint32_t SWIER;			//Software interrupt event registerr
	_vo uint32_t PR;			//Pending register
}EXTI_RegDef_t;

/*
 * peripheral register definition structure of AFIO
 */

typedef struct
{
	_vo uint32_t EVCR;			//Event control register
	_vo uint32_t MAPR;			//AF remap and debug I/O configuration register
	_vo uint32_t EXTICR[4];		//External interrupt configuration register 1
	_vo uint32_t MAPR2;			//AF remap and debug I/O configuration register2
}AFIO_RegDef_t;

/*
 * peripheral register definition structure of SPI
 */

typedef struct
{
	_vo uint32_t CR1;			//SPI control register 1
	_vo uint32_t CR2;			//SPI control register 2
	_vo uint32_t SR;			//SPI status register
	_vo uint32_t DR;			//SPI data register
	_vo uint32_t CRCPR;			//SPI CRC polynomial register
	_vo uint32_t RXCRCR;		//SPI RX CRC register
	_vo uint32_t TXCRCR;		//SPI TX CRC register
	_vo uint32_t I2SCFGR;		//SPI_I2S configuration register
	_vo uint32_t I2SPR;			//SPI_I2S prescaler register
}SPI_RegDef_t;

/*
 * peripheral register definition structure of I2C
 */

typedef struct
{
	_vo uint32_t CR1;			//I2C Control register 1
	_vo uint32_t CR2;			//I2C Control register 2
	_vo uint32_t OAR1;			//I2C Own address register 1
	_vo uint32_t OAR2;			//I2C Own address register 2
	_vo uint32_t DR;			//I2C Data register
	_vo uint32_t SR1;			//I2C Status register 1
	_vo uint32_t SR2;			//I2C Status register 2
	_vo uint32_t CCR;			//I2C Clock control register
	_vo uint32_t TRISE;			//I2C TRISE register
}I2C_RegDef_t;

/*
 * peripheral register definition structure of USART
 */

typedef struct
{
	_vo uint32_t SR;			//USART Status register
	_vo uint32_t DR;			//USART Data register
	_vo uint32_t BRR;			//USART Baud Rate register 1
	_vo uint32_t CR1;			//USART Control register 1
	_vo uint32_t CR2;			//USART Control register 2
	_vo uint32_t CR3;			//USART Control register 3
	_vo uint32_t GTPR;			//USART Guard time and prescaler register
}USART_RegDef_t;

/*
 * peripheral definitions
 */

#define GPIOA							((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB							((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC							((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD							((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE							((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF							((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define GPIOG							((GPIO_RegDef_t*)GPIOG_BASE_ADDR)

#define RCC								((RCC_RegDef_t*)RCC_BASE_ADDR)
#define EXTI							((EXTI_RegDef_t*)EXTI_BASE_ADDR)
#define AFIO							((AFIO_RegDef_t*)AFIO_BASE_ADDR)
#define SPI1							((SPI_RegDef_t*)SPI1_BASE_ADDR)
#define SPI2							((SPI_RegDef_t*)SPI2_BASE_ADDR)
#define SPI3							((SPI_RegDef_t*)SPI3_BASE_ADDR)
#define I2C1							((I2C_RegDef_t*)I2C1_BASE_ADDR)
#define I2C2							((I2C_RegDef_t*)I2C2_BASE_ADDR)
#define USART1							((USART_RegDef_t*)USART1_BASE_ADDR)
#define USART2							((USART_RegDef_t*)USART2_BASE_ADDR)
#define USART3							((USART_RegDef_t*)USART3_BASE_ADDR)

/*
 * Clock Enable Macros for GPIO peripherals
 */

#define GPIOA_PCLK_EN()					(RCC -> APB2ENR |= (1 << 2))
#define GPIOB_PCLK_EN()					(RCC -> APB2ENR |= (1 << 3))
#define GPIOC_PCLK_EN()					(RCC -> APB2ENR |= (1 << 4))
#define GPIOD_PCLK_EN()					(RCC -> APB2ENR |= (1 << 5))
#define GPIOE_PCLK_EN()					(RCC -> APB2ENR |= (1 << 6))
#define GPIOF_PCLK_EN()					(RCC -> APB2ENR |= (1 << 7))
#define GPIOG_PCLK_EN()					(RCC -> APB2ENR |= (1 << 8))

/*
 * Clock Disable Macros for GPIO
 */

#define GPIOA_PCLK_DI()					(RCC -> APB2ENR &= ~(1 << 2))
#define GPIOB_PCLK_DI()					(RCC -> APB2ENR &= ~(1 << 3))
#define GPIOC_PCLK_DI()					(RCC -> APB2ENR &= ~(1 << 4))
#define GPIOD_PCLK_DI()					(RCC -> APB2ENR &= ~(1 << 5))
#define GPIOE_PCLK_DI()					(RCC -> APB2ENR &= ~(1 << 6))
#define GPIOF_PCLK_DI()					(RCC -> APB2ENR &= ~(1 << 7))
#define GPIOG_PCLK_DI()					(RCC -> APB2ENR &= ~(1 << 8))

/*
 * Clock Enable & Disable Macro for AFIO
 */
#define AFIO_PCLK_EN()					(RCC -> APB2ENR |= 1)
#define AFIO_PCLK_DI()					(RCC -> APB2ENR &= ~1)

/*
 * Clock Enable Macro for SPI
 */
#define SPI1_PCLK_EN()					(RCC -> APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()					(RCC -> APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()					(RCC -> APB1ENR |= (1 << 15))

/*
 * Clock Disable Macro for SPI
 */
#define SPI1_PCLK_DI()					(RCC -> APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()					(RCC -> APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()					(RCC -> APB1ENR &= ~(1 << 15))

/*
 * Clock Enable Macro for I2C
 */
#define I2C1_PCLK_EN()					(RCC -> APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()					(RCC -> APB1ENR |= (1 << 22))

/*
 * Clock Disable Macro for I2C
 */
#define I2C1_PCLK_DI()					(RCC -> APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()					(RCC -> APB1ENR &= ~(1 << 22))

/*
 * Clock Enable Macro for USART
 */
#define USART1_PCLK_EN()					(RCC -> APB2ENR |= (1 << 14))
#define USART2_PCLK_EN()					(RCC -> APB1ENR |= (1 << 17))

/*
 * Clock Disable Macro for USART
 */
#define USART1_PCLK_DI()					(RCC -> APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI()					(RCC -> APB1ENR &= ~(1 << 17))

//generic macros

#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET
#define FLAG_SET		SET
#define FLAG_RESET		RESET



// macros for GPIO Pin Number
#define PIN_NUM_0		0
#define PIN_NUM_1		1
#define PIN_NUM_2		2
#define PIN_NUM_3		3
#define PIN_NUM_4		4
#define PIN_NUM_5		5
#define PIN_NUM_6		6
#define PIN_NUM_7		7
#define PIN_NUM_8		8
#define PIN_NUM_9		9
#define PIN_NUM_10		10
#define PIN_NUM_11		11
#define PIN_NUM_12		12
#define PIN_NUM_13		13
#define PIN_NUM_14		14
#define PIN_NUM_15		15

//macros to reset GPIOx Peripheral
#define GPIOA_REG_RESET()				do{(RCC->APB2RSTR |= (1<<2)); (RCC->APB2RSTR &= ~(1<<2)); }while(0)
#define GPIOB_REG_RESET()				do{(RCC->APB2RSTR |= (1<<3)); (RCC->APB2RSTR &= ~(1<<3)); }while(0)
#define GPIOC_REG_RESET()				do{(RCC->APB2RSTR |= (1<<4)); (RCC->APB2RSTR &= ~(1<<4)); }while(0)
#define GPIOD_REG_RESET()				do{(RCC->APB2RSTR |= (1<<5)); (RCC->APB2RSTR &= ~(1<<5)); }while(0)
#define GPIOE_REG_RESET()				do{(RCC->APB2RSTR |= (1<<6)); (RCC->APB2RSTR &= ~(1<<6)); }while(0)
#define GPIOF_REG_RESET()				do{(RCC->APB2RSTR |= (1<<7)); (RCC->APB2RSTR &= ~(1<<7)); }while(0)
#define GPIOG_REG_RESET()				do{(RCC->APB2RSTR |= (1<<8)); (RCC->APB2RSTR &= ~(1<<8)); }while(0)

//macros to reset SPIx Peripheral
#define SPI1_REG_RESET()				do{(RCC->APB2RSTR |= (1<<12)); (RCC->APB2RSTR &= ~(1<<12)); }while(0)
#define SPI2_REG_RESET()				do{(RCC->APB1RSTR |= (1<<14)); (RCC->APB1RSTR &= ~(1<<14)); }while(0)
#define SPI3_REG_RESET()				do{(RCC->APB1RSTR |= (1<<15)); (RCC->APB1RSTR &= ~(1<<15)); }while(0)

//GPIO IRQ(Interrupt Request) Numbers of STM32F103x MCU
#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40

//SPI IRQ(Interrupt Request) Numbers of STM32F103x MCU
#define IRQ_SPI1 			35
#define IRQ_SPI2 			36
#define IRQ_SPI3 			51

//I2C IRQ(Interrupt Request) Numbers of STM32F103x MCU
#define IRQ_I2C1_EV 		31
#define IRQ_I2C1_ER 		32
#define IRQ_I2C2_EV 		33
#define IRQ_I2C2_ER 		34

//USART IRQ(Interrupt Request) Numbers of STM32F103x MCU
#define IRQ_USART1 			37
#define IRQ_USART2 			38
#define IRQ_USART3 			39

/*****************************************************************************************
 * Bit postition definitions of SPI peripheral
 *****************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7

/*****************************************************************************************
 * Bit postition definitions of I2C peripheral
 *****************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15

/*****************************************************************************************
 * Bit postition definitions of USART peripheral
 *****************************************************************************************/
/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9





/*
 * Flags for SPI Status register
 */
#define RXNE_FLAG						(1 << SPI_SR_RXNE)
#define TXE_FLAG						(1 << SPI_SR_TXE)
#define CHSIDE_FLAG						(1 << SPI_SR_CHSIDE)
#define UDE_FLAG						(1 << SPI_SR_UDR)
#define CRCERR_FLAG						(1 << SPI_SR_CRCERR)
#define MODF_FLAG						(1 << SPI_SR_MODF)
#define OVR_FLAG						(1 << SPI_SR_OVR)
#define BSY_FLAG						(1 << SPI_SR_BSY)

#endif /* INC_STM32F103XX_H_ */
