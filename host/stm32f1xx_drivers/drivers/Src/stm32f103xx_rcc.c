/*
 * stm32f103xx_rcc.c
 *
 *  Created on: Jan 19, 2026
 *      Author: John
 */


#include "stm32f103xx_rcc.h"
static uint32_t RCC_GetHSEOutputClock();
static uint32_t RCC_GetPLLOutputClock();

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = { 2, 4 , 8, 16};


uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1,SystemClk;
	uint8_t clksrc,temp,ahbp,apb1p;

	clksrc = (RCC->CFGR >> 2) & 0x3;

	if (clksrc == 0)
	{
		//HSI Selected
		SystemClk = 8000000U;
	}else if (clksrc == 1)
	{
		//HSE clock
		SystemClk = RCC_GetHSEOutputClock();
	} else if (clksrc == 2) {
		//PLL
		SystemClk = RCC_GetPLLOutputClock();

	}

	//for ahb
	temp = (RCC->CFGR >> 4) & 0xF;
	if (temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[8 - temp];
	}

	//for apb1
	temp = (RCC->CFGR >> 8) & 0x7;
	if(temp < 4)
	{
		apb1p = 1;
	}else
	{
		apb1p = APB1_PreScaler[temp - 4];
	}

	pclk1 = (SystemClk/ahbp) / apb1p;

	return pclk1;
}

uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t SystemClock,tmp,pclk2;
	uint8_t clk_src = ( RCC->CFGR >> 2) & 0X3;
	uint8_t ahbp,apb2p;

	clk_src = (RCC->CFGR >> 2) & 0x3;

	if (clk_src == 0)
	{
		//HSI Selected
		SystemClock = 8000000U;
	}else if (clk_src == 1)
	{
		//HSE clock
		SystemClock = RCC_GetHSEOutputClock();
	} else if (clk_src == 2) {
		//PLL
		SystemClock = RCC_GetPLLOutputClock();
	}

	//for ahb
	tmp = (RCC->CFGR >> 4) & 0xF;
	if (tmp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[8 - tmp];
	}

	//for apb2
	tmp = (RCC->CFGR >> 11) & 0x7;
	if(tmp < 4)
	{
		apb2p = 1;
	}else
	{
		apb2p = APB1_PreScaler[tmp - 4];
	}

	pclk2 = (SystemClock/ahbp) / apb2p;

	return pclk2;
}

static uint32_t RCC_GetHSEOutputClock(){
	return 0;
}

static uint32_t RCC_GetPLLOutputClock(){
	return 0;
}
