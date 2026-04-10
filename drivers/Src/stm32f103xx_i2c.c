/*
 * stm32f103xx_i2c.c
 * 생성일: 2026. 1. 14.
 * 작성자: John
 */

#include "stm32f103xx_i2c.h"
#include "stm32f103xx_rcc.h"
#include <stdio.h>

#define PCLK_SPEED      8000000

// 내부 정적 함수 선언
static uint8_t I2C_GetStatusFlag(I2C_RegDef_t *pI2Cx, uint8_t status_flag);
static void I2C_SendAddressWrite(I2C_RegDef_t *pI2Cx, uint8_t slave_addr);
static void I2C_SendAddressRead(I2C_RegDef_t *pI2Cx, uint8_t slave_addr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_ACKControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

// I2C 상태 플래그 정의
#define I2C_FLAG_TXE         ( 1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE        ( 1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB          ( 1 << I2C_SR1_SB)
#define I2C_FLAG_OVR         ( 1 << I2C_SR1_OVR)
#define I2C_FLAG_AF          ( 1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO        ( 1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR        ( 1 << I2C_SR1_BERR)
#define I2C_FLAG_STOPF       ( 1 << I2C_SR1_STOPF)
#define I2C_FLAG_BTF         ( 1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR        ( 1 << I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT     ( 1 << I2C_SR1_TIMEOUT)

// I2C 주변장치 클럭 제어
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        if (pI2Cx == I2C1) I2C1_PCLK_EN();
        else if (pI2Cx == I2C2) I2C2_PCLK_EN();
    } else {
        if (pI2Cx == I2C1) I2C1_PCLK_DI();
        else if (pI2Cx == I2C2) I2C2_PCLK_DI();
    }
}

// I2C 활성화/비활성화 (PE 비트)
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) pI2Cx->CR1 |= (1 << I2C_CR1_PE);
    else pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
}

// I2C 초기화 (속도, 주소, 타이밍 설정)
void I2C_Init(I2C_Handle_t *pI2CHandle) {
    uint32_t tempreg = 0;

    // 1. PCLK 주파수 설정 (CR2)
    tempreg = RCC_GetPCLK1Value() / 1000000U;
    pI2CHandle->pI2Cx->CR2 = tempreg;

    // 2. 장치 고유 주소 설정 (OAR1)
    tempreg = (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1) | (1 << 14);
    pI2CHandle->pI2Cx->OAR1 = tempreg;

    // 3. CCR 계산 (통신 속도)
    uint16_t ccr_value;
    tempreg = 0;
    if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
        ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
    } else {
        tempreg |= (1 << I2C_CCR_FS) | (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);
        uint8_t factor = (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) ? 3 : 25;
        ccr_value = RCC_GetPCLK1Value() / (factor * pI2CHandle->I2C_Config.I2C_SCLSpeed);
    }
    pI2CHandle->pI2Cx->CCR = tempreg | (ccr_value & 0xFFF);

    // 4. TRISE 설정 (최대 상승 시간)
    if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
        tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
    else
        tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
    pI2CHandle->pI2Cx->TRISE = tempreg & 0x3F;

    I2C_PeripheralControl(pI2CHandle->pI2Cx, ENABLE);
    
    // ACK 설정
    I2C_ACKControl(pI2CHandle->pI2Cx, pI2CHandle->I2C_Config.I2C_AckControl);
}

// 상태 플래그 확인
static uint8_t I2C_GetStatusFlag(I2C_RegDef_t *pI2Cx, uint8_t status_flag) {
    return (pI2Cx->SR1 & status_flag) ? FLAG_SET : FLAG_RESET;
}

// 쓰기 주소 송신
static void I2C_SendAddressWrite(I2C_RegDef_t *pI2Cx, uint8_t slave_addr) {
    pI2Cx->DR = (slave_addr << 1) & ~(1 << 0);
}

// 읽기 주소 송신
static void I2C_SendAddressRead(I2C_RegDef_t *pI2Cx, uint8_t slave_addr) {
    pI2Cx->DR = (slave_addr << 1) | (1 << 0);
}

// ADDR 플래그 클리어 (SR1, SR2 읽기)
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle) {
    uint32_t dummy_read;
    if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) { // 마스터 모드
        if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX && pI2CHandle->RxSize == 1) {
            I2C_ACKControl(pI2CHandle->pI2Cx, DISABLE);
        }
    }
    dummy_read = pI2CHandle->pI2Cx->SR1;
    dummy_read = pI2CHandle->pI2Cx->SR2;
    (void)dummy_read;
}

// STOP 조건 생성
void I2C_StopGeneration(I2C_RegDef_t *pI2Cx) {
    pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

// ACK 활성/비활성 제어
static void I2C_ACKControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
    else pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
}

// 인터럽트 기반 마스터 송신 처리
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle) {
    if (pI2CHandle->TxLen > 0) {
        pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
        pI2CHandle->TxLen--;
        pI2CHandle->pTxBuffer++;
    }
}

// 인터럽트 기반 마스터 수신 처리
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle) {
    if (pI2CHandle->RxSize == 1) {
        *(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
        pI2CHandle->RxLen--;
    } else if (pI2CHandle->RxSize > 1) {
        if (pI2CHandle->RxLen == 2) I2C_ACKControl(pI2CHandle->pI2Cx, DISABLE);
        *(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
        pI2CHandle->pRxBuffer++;
        pI2CHandle->RxLen--;
    }

    if(pI2CHandle->RxLen == 0) {
        if(pI2CHandle->Sr == I2C_SR_DISABLE) I2C_StopGeneration(pI2CHandle->pI2Cx);
        I2C_CloseReceiveData(pI2CHandle);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
    }
}

// 송신 종료 및 인터럽트 비활성화
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle) {
    pI2CHandle->pI2Cx->CR2 &= ~((1 << I2C_CR2_ITBUFEN) | (1 << I2C_CR2_ITEVTEN));
    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pTxBuffer = NULL;
}

// 수신 종료 및 인터럽트 비활성화
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle) {
    pI2CHandle->pI2Cx->CR2 &= ~((1 << I2C_CR2_ITBUFEN) | (1 << I2C_CR2_ITEVTEN));
    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pRxBuffer = NULL;
    if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE) I2C_ACKControl(pI2CHandle->pI2Cx, ENABLE);
}

// 마스터 데이터 송신 (폴링)
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {
    pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START); // START 생성
    while (!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_SB));
    
    I2C_SendAddressWrite(pI2CHandle->pI2Cx, SlaveAddr); // 주소 송신
    while (!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));
    I2C_ClearADDRFlag(pI2CHandle);

    while (Len > 0) { // 데이터 송신 루프
        while (!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
        pI2CHandle->pI2Cx->DR = *pTxbuffer;
        Len--; pTxbuffer++;
    }

    while (!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
    while (!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_BTF));
    
    if (Sr == I2C_SR_DISABLE) I2C_StopGeneration(pI2CHandle->pI2Cx); // STOP 생성
}

// 마스터 데이터 수신 (폴링)
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr) {
    pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);
    while (!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_SB));

    I2C_SendAddressRead(pI2CHandle->pI2Cx, SlaveAddr);
    while (!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

    if (Len == 1) { // 1바이트 수신
        I2C_ACKControl(pI2CHandle->pI2Cx, DISABLE);
        I2C_ClearADDRFlag(pI2CHandle);
        while (!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
        if (Sr == I2C_SR_DISABLE) I2C_StopGeneration(pI2CHandle->pI2Cx);
        *pRxBuffer = pI2CHandle->pI2Cx->DR;
    } else if (Len > 1) { // 다중 바이트 수신
        I2C_ClearADDRFlag(pI2CHandle);
        for (uint32_t i = Len; i > 0; i--) {
            while (!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
            if (i == 2) {
                I2C_ACKControl(pI2CHandle->pI2Cx, DISABLE);
                if (Sr == I2C_SR_DISABLE) I2C_StopGeneration(pI2CHandle->pI2Cx);
            }
            *pRxBuffer = pI2CHandle->pI2Cx->DR;
            pRxBuffer++;
        }
    }
    I2C_ACKControl(pI2CHandle->pI2Cx, ENABLE);
}

// 인터럽트 기반 마스터 송신 시작
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {
    uint8_t busystate = pI2CHandle->TxRxState;
    if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) {
        pI2CHandle->pTxBuffer = pTxBuffer;
        pI2CHandle->TxLen = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
        pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;

        pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN) | (1 << I2C_CR2_ITEVTEN) | (1 << I2C_CR2_ITERREN);
    }
    return busystate;
}

// 인터럽트 기반 마스터 수신 시작
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {
    uint8_t busystate = pI2CHandle->TxRxState;
    if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) {
        pI2CHandle->pRxBuffer = pRxBuffer;
        pI2CHandle->RxLen = Len;
        pI2CHandle->RxSize = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
        pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;

        pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN) | (1 << I2C_CR2_ITEVTEN) | (1 << I2C_CR2_ITERREN);
    }
    return busystate;
}

// I2C 이벤트 인터럽트 핸들러
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle) {
    uint32_t temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
    uint32_t temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
    uint32_t sr1 = pI2CHandle->pI2Cx->SR1;

    // 1. SB (Start Bit) 발생 처리
    if (temp2 && (sr1 & (1 << I2C_SR1_SB))) {
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) I2C_SendAddressWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
        else I2C_SendAddressRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
    }

    // 2. ADDR (주소 송신 완료) 처리
    if (temp2 && (sr1 & (1 << I2C_SR1_ADDR))) I2C_ClearADDRFlag(pI2CHandle);

    // 3. BTF (바이트 전송 완료) 처리
    if (temp2 && (sr1 & (1 << I2C_SR1_BTF))) {
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX && pI2CHandle->TxLen == 0) {
            if (pI2CHandle->Sr == I2C_SR_DISABLE) I2C_StopGeneration(pI2CHandle->pI2Cx);
            I2C_CloseSendData(pI2CHandle);
            I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
        }
    }

    // 4. STOPF (스톱 비트 감지) 처리 (슬레이브 전용)
    if (temp2 && (sr1 & (1 << I2C_SR1_STOPF))) {
        pI2CHandle->pI2Cx->CR1 |= 0x0000;
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
    }

    // 5. TXE (데이터 레지스터 비었음) 처리
    if (temp1 && temp2 && (sr1 & (1 << I2C_SR1_TXE))) {
        if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) { // 마스터
            if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) I2C_MasterHandleTXEInterrupt(pI2CHandle);
        } else if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)) { // 슬레이브 송신
            I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
        }
    }

    // 6. RXNE (데이터 수신됨) 처리
    if (temp1 && temp2 && (sr1 & (1 << I2C_SR1_RXNE))) {
        if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) { // 마스터
            if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) I2C_MasterHandleRXNEInterrupt(pI2CHandle);
        } else if (!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))) { // 슬레이브 수신
            I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
        }
    }
}

// NVIC 인터럽트 활성/비활성 설정
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        if (IRQNumber <= 31) *NVIC_ISER0 |= (1 << IRQNumber);
        else *NVIC_ISER1 |= (1 << (IRQNumber % 32));
    } else {
        if (IRQNumber <= 31) *NVIC_ICER0 |= (1 << IRQNumber);
        else *NVIC_ICER1 |= (1 << (IRQNumber % 32));
    }
}

// NVIC 인터럽트 우선순위 설정
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
    uint8_t iprx = IRQNumber / 4;
    uint8_t section = IRQNumber % 4;
    uint8_t shift = (8 * section) + (8 - __NVIC_PRIO_BITS);
    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift);
}

// I2C 에러 인터럽트 핸들러
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle) {
    uint32_t temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITERREN);
    uint32_t sr1 = pI2CHandle->pI2Cx->SR1;

    // 버스 에러, 중재 손실, ACK 실패, 오버런, 타임아웃 체크 및 처리
    if (temp2 && (sr1 & (1 << I2C_SR1_BERR))) {
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
    }
    if (temp2 && (sr1 & (1 << I2C_SR1_ARLO))) {
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
    }
    if (temp2 && (sr1 & (1 << I2C_SR1_AF))) {
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
    }
    if (temp2 && (sr1 & (1 << I2C_SR1_OVR))) {
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
    }
    if (temp2 && (sr1 & (1 << I2C_SR1_TIMEOUT))) {
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
    }
}