/*
 * stm32f103xx_spi.c
 * 생성일: 2026. 1. 5.
 * 작성자: John
 */

#include "stm32f103xx_spi.h"
#include <stddef.h>

// 내부 인터럽트 핸들러 함수 선언
static void spi_txe_interrupt_handler();
static void spi_rxne_interrupt_handler();
static void spi_ovr_interrupt_handler();

// SPI 주변장치 클럭 제어
void SPIPeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pSPIx == SPI1) SPI1_PCLK_EN();
        else if (pSPIx == SPI2) SPI2_PCLK_EN();
        else if (pSPIx == SPI3) SPI3_PCLK_EN();
    } else {
        if (pSPIx == SPI1) SPI1_PCLK_DI();
        else if (pSPIx == SPI2) SPI2_PCLK_DI();
        else if (pSPIx == SPI3) SPI3_PCLK_DI();
    }
}

// SPI 초기화
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
    uint32_t tempreg = 0;
    SPIPeriClockControl(pSPIHandle->pSPIx, ENABLE); // 클럭 활성화

    // 1. (Master/Slave) 모드 설정
    tempreg |= pSPIHandle->SPI_PinConfig.SPI_DeviceMode << SPI_CR1_MSTR;

    // 2. 버스 구성 (Full-duplex, Half-duplex, Simplex)
    if (pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
        pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIMODE); // 양방향 모드 해제
    } else if (pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
        pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_BIDIMODE);  // 양방향 모드 설정
    } else if (pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
        pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIMODE);
        pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_RXONLY);    // 수신 전용 설정
    }

    // 3. Baud rate 설정
    tempreg |= pSPIHandle->SPI_PinConfig.SPI_SclkSpeed << SPI_CR1_BR;

    // 4. 데이터 프레임 포맷(8bit/16bit) 설정
    tempreg |= pSPIHandle->SPI_PinConfig.SPI_DFF << SPI_CR1_DFF;

    // 5. CPOL & CPHA 설정
    tempreg |= pSPIHandle->SPI_PinConfig.SPI_CPOL << SPI_CR1_CPOL;
    tempreg |= pSPIHandle->SPI_PinConfig.SPI_CPHA << SPI_CR1_CPHA;

    // 6. 소프트웨어 슬레이브 관리(SSM) 설정
    tempreg |= pSPIHandle->SPI_PinConfig.SPI_SSM << SPI_CR1_SSM;

    pSPIHandle->pSPIx->CR1 = tempreg;
}

// SPI 주변장치 활성화 (SPE 비트)
void SPIPeriControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE) pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    else pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
}

// SSI 설정 (Internal Slave Select)
void SSIControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE) pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    else pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
}

// SSOE 설정 (Slave Select Output Enable)
void SSOEControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE) pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
    else pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
}

// SPI 레지스터 초기화
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
    if (pSPIx == SPI1) SPI1_REG_RESET();
    else if (pSPIx == SPI2) SPI2_REG_RESET();
    else if (pSPIx == SPI3) SPI3_REG_RESET();
}

// 상태 플래그 확인
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t Flagname)
{
    return (pSPIx->SR & Flagname) ? FLAG_SET : FLAG_RESET;
}

// 데이터 송신 (폴링 방식)
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
    while (Len > 0)
    {
        while(SPI_GetFlagStatus(pSPIx, TXE_FLAG) == FLAG_RESET); // TXE 대기

        if ((pSPIx->CR1) & (1 << SPI_CR1_DFF)) { // 16비트 모드
            pSPIx->DR = *((uint16_t*)pTxBuffer);
            pTxBuffer += 2; Len -= 2;
        } else { // 8비트 모드
            pSPIx->DR = *pTxBuffer;
            pTxBuffer += 1; Len -= 1;
        }
    }
}

// 데이터 수신 (폴링 방식)
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
    while (Len > 0)
    {
        while(SPI_GetFlagStatus(pSPIx, RXNE_FLAG) == FLAG_RESET); // RXNE 대기

        if ((pSPIx->CR1) & (1 << SPI_CR1_DFF)) { // 16비트 모드
            *((uint16_t*)pRxBuffer) = pSPIx->DR;
            pRxBuffer += 2; Len -= 2;
        } else { // 8비트 모드
            *pRxBuffer = pSPIx->DR;
            pRxBuffer += 1; Len -= 1;
        }
    }
}

// 인터럽트 기반 송신 시작
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    uint8_t state = pSPIHandle->TxState;
    if (state != SPI_BUSY_IN_TX) {
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = Len;
        pSPIHandle->TxState = SPI_BUSY_IN_TX;
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE); // TXE 인터럽트 활성화
    }
    return state;
}

// 인터럽트 기반 수신 시작
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    uint8_t state = pSPIHandle->RxState;
    if (state != SPI_BUSY_IN_RX) {
        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLen = Len;
        pSPIHandle->RxState = SPI_BUSY_IN_RX;
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE); // RXNE 인터럽트 활성화
    }
    return state;
}

// SPI 인터럽트 통합 핸들러 (TXE, RXNE, OVR 처리)
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
    uint8_t temp1, temp2;
    // TXE 체크
    temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
    temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
    if(temp1 && temp2) spi_txe_interrupt_handler(pSPIHandle);

    // RXNE 체크
    temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
    temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
    if(temp1 && temp2) spi_rxne_interrupt_handler(pSPIHandle);

    // OVR(오버런) 체크
    temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
    temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
    if(temp1 && temp2) spi_ovr_interrupt_handler(pSPIHandle);
}

// 도움 함수: 실제 TX 인터럽트 처리
static void spi_txe_interrupt_handler(SPI_Handle_t *pSPIHandle)
{
    if ((pSPIHandle->pSPIx->CR1) & (1 << SPI_CR1_DFF)) { // 16비트 전송
        pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
        pSPIHandle->pTxBuffer += 2; pSPIHandle->TxLen -= 2;
    } else { // 8비트 전송
        pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
        pSPIHandle->pTxBuffer += 1; pSPIHandle->TxLen -= 1;
    }

    if (!pSPIHandle->TxLen) { // 전송 완료 시 종료
        SPI_CloseTransmission(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
    }
}

// 도움 함수: 실제 RX 인터럽트 처리
static void spi_rxne_interrupt_handler(SPI_Handle_t *pSPIHandle)
{
    if ((pSPIHandle->pSPIx->CR1) & (1 << SPI_CR1_DFF)) { // 16비트 수신
        *((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
        pSPIHandle->pRxBuffer += 2; pSPIHandle->RxLen -= 2;
    } else { // 8비트 수신
        *(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
        pSPIHandle->pRxBuffer += 1; pSPIHandle->RxLen -= 1;
    }

    if (!pSPIHandle->RxLen) { // 수신 완료 시 종료
        SPI_CloseReception(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
    }
}

// 도움 함수: 오버런 에러 처리
static void spi_ovr_interrupt_handler(SPI_Handle_t *pSPIHandle)
{
    uint8_t temp;
    if (pSPIHandle->TxState != SPI_BUSY_IN_TX) { // 플래그 클리어 (DR, SR 읽기)
        temp = pSPIHandle->pSPIx->DR;
        temp = pSPIHandle->pSPIx->SR;
    }
    (void)temp;
    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

// 송신 인터럽트 비활성화 및 상태 초기화
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
    pSPIHandle->pTxBuffer = NULL;
    pSPIHandle->TxLen = 0;
    pSPIHandle->TxState = SPI_READY;
}

// 수신 인터럽트 비활성화 및 상태 초기화
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
    pSPIHandle->pRxBuffer = NULL;
    pSPIHandle->RxLen = 0;
    pSPIHandle->RxState = SPI_READY;
}

// 애플리케이션 콜백 (사용자 구현용)
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{
    //재정의 필요
}