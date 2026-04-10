/*
 * stm32f103xx_usart.c
 * 생성일: 2026. 1. 17.
 * 작성자: John
 */

#include "stm32f103xx_usart.h"
#include "stm32f103xx_rcc.h"
#include <stdio.h>

// USART 주변장치 클럭 제어
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE) {
        if (pUSARTx == USART1) USART1_PCLK_EN();
        else if (pUSARTx == USART2) USART2_PCLK_EN(); // (기존 코드의 USART1 중복 체크 수정)
    } else {
        if (pUSARTx == USART1) USART1_PCLK_DI();
        else if (pUSARTx == USART2) USART2_PCLK_DI();
    }
}

// 상태 플래그 확인
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
    return (pUSARTx->SR & FlagName) ? FLAG_SET : FLAG_RESET;
}

// USART 활성화/비활성화 (UE 비트)
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE) pUSARTx->CR1 |= (1 << USART_CR1_UE);
    else pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
}

// 보레이트 설정 (BRR 레지스터 계산)
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
    uint32_t PCLKx, usartdiv, M_part, F_part, tempreg = 0;

    // 1. 해당 USART의 버스 클럭(PCLK) 가져오기
    PCLKx = (pUSARTx == USART1) ? RCC_GetPCLK2Value() : RCC_GetPCLK1Value();

    // 2. USARTDIV 계산 (Over-sampling by 16 기준)
    usartdiv = ((25 * PCLKx) / (4 * BaudRate));

    // 3. Mantissa(정수부) 추출
    M_part = usartdiv / 100;
    tempreg |= M_part << 4;

    // 4. Fraction(소수부) 추출 및 반올림
    F_part = (usartdiv - (M_part * 100));
    F_part = (((F_part * 16) + 50) / 100) & ((uint8_t)0x0F);
    tempreg |= F_part;

    pUSARTx->BRR = tempreg;
}

// USART 초기화 (CR1, CR2, CR3 설정)
void USART_Init(USART_Handle_t *pUSARTHandle)
{
    uint32_t tempreg = 0;
    USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

    // 1. 모드 설정 (TX/RX)
    if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX) tempreg |= (1 << USART_CR1_RE);
    else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX) tempreg |= (1 << USART_CR1_TE);
    else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX) tempreg |= (1 << USART_CR1_RE) | (1 << USART_CR1_TE);

    // 2. 데이터 길이(8bit/9bit) 및 패리티 설정
    tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;
    if (pUSARTHandle->USART_Config.USART_ParityControl != USART_PARITY_DISABLE) {
        tempreg |= (1 << USART_CR1_PCE);
        if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD) tempreg |= (1 << USART_CR1_PS);
    }
    pUSARTHandle->pUSARTx->CR1 = tempreg;

    // 3. 정지 비트(Stop Bits) 설정 (CR2)
    pUSARTHandle->pUSARTx->CR2 = (pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP);

    // 4. 하드웨어 흐름 제어(CTS/RTS) 설정 (CR3)
    tempreg = 0;
    if (pUSARTHandle->USART_Config.USART_HWFlowControl != USART_HW_FLOW_CTRL_NONE) {
        if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS) tempreg |= (1 << USART_CR3_CTSE);
        else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS) tempreg |= (1 << USART_CR3_RTSE);
        else tempreg |= (1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE);
    }
    pUSARTHandle->pUSARTx->CR3 = tempreg;

    USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}

// 데이터 송신 (폴링 방식)
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    for(uint32_t i = 0 ; i < Len; i++) {
        while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

        if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
            pUSARTHandle->pUSARTx->DR = (*(uint16_t*)pTxBuffer & 0x01FF);
            pTxBuffer += (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) ? 2 : 1;
        } else {
            pUSARTHandle->pUSARTx->DR = (*pTxBuffer & 0xFF);
            pTxBuffer++;
        }
    }
    while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC)); // 전송 완료 대기
}

// 데이터 수신 (폴링 방식)
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    for(uint32_t i = 0 ; i < Len; i++) {
        while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

        if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
            if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
                *(uint16_t*)pRxBuffer = (pUSARTHandle->pUSARTx->DR & 0x1FF);
                pRxBuffer += 2;
            } else {
                *pRxBuffer = (pUSARTHandle->pUSARTx->DR & 0xFF);
                pRxBuffer++;
            }
        } else {
            if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) *pRxBuffer = (pUSARTHandle->pUSARTx->DR & 0xFF);
            else *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & 0x7F);
            pRxBuffer++;
        }
    }
}

// 인터럽트 기반 송신 시작
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    uint8_t tx_state = pUSARTHandle->TxBusyState;
    if (tx_state != USART_BUSY_IN_TX) {
        pUSARTHandle->pTxBuffer = pTxBuffer;
        pUSARTHandle->TxLen = Len;
        pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;
        pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE) | (1 << USART_CR1_TCIE);
    }
    return tx_state;
}

// 인터럽트 기반 수신 시작
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    uint8_t rx_state = pUSARTHandle->RxBusyState;
    if (rx_state != USART_BUSY_IN_RX) {
        pUSARTHandle->pRxBuffer = pRxBuffer;
        pUSARTHandle->RxLen = Len;
        pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;
        pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
    }
    return rx_state;
}

// USART 인터럽트 핸들러 (TC, TXE, RXNE 처리)
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
    uint32_t temp1, temp2;

    // 1. TC 처리
    temp1 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCIE);
    temp2 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC);
    if(temp1 && temp2) {
        if (pUSARTHandle->TxBusyState == USART_BUSY_IN_TX && !pUSARTHandle->TxLen) {
            pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_TC);
            pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TCIE);
            pUSARTHandle->TxBusyState = USART_READY;
            USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
        }
    }

    // 2. TXE 처리
    temp1 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);
    temp2 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE);
    if (temp1 && temp2 && pUSARTHandle->TxLen > 0) {
        if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
            pUSARTHandle->pUSARTx->DR = (*(uint16_t*)pUSARTHandle->pTxBuffer & 0x1FF);
            pUSARTHandle->pTxBuffer += (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) ? 2 : 1;
        } else {
            pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer & 0xFF);
            pUSARTHandle->pTxBuffer++;
        }
        if (!(--pUSARTHandle->TxLen)) pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
    }

    // 3. RXNE 처리
    temp1 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);
    temp2 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE);
    if(temp1 && temp2 && pUSARTHandle->RxLen > 0) {
        if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
            if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
                *(uint16_t*)pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & 0x1FF);
                pUSARTHandle->pRxBuffer += 2;
            } else {
                *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & 0xFF);
                pUSARTHandle->pRxBuffer++;
            }
        } else {
            if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & 0xFF);
            else *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & 0x7F);
            pRxBuffer++;
        }
        if (!(--pUSARTHandle->RxLen)) {
            pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
            pUSARTHandle->RxBusyState = USART_READY;
            USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
        }
    }
}

// NVIC 설정
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if (EnorDi == ENABLE) {
        if (IRQNumber < 32) *NVIC_ISER0 |= (1 << IRQNumber);
        else *NVIC_ISER1 |= (1 << (IRQNumber % 32));
    } else {
        if (IRQNumber < 32) *NVIC_ICER0 |= (1 << IRQNumber);
        else *NVIC_ICER1 |= (1 << (IRQNumber % 32));
    }
}

// NVIC 우선순위 설정
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t section = IRQNumber % 4;
    uint8_t shift = (8 * section) + (8 - __NVIC_PRIO_BITS);
    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift);
}