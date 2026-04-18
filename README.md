#  STM32 Bare-metal Peripheral Drivers

> **HAL 라이브러리를 사용하지 않고, 오직 데이터시트 분석을 통해 구현한 레지스터 레벨 드라이버 라이브러리입니다.**

이 프로젝트는 MCU의 내부 동작 원리를 깊이 있게 이해하고, 자원이 제한된 임베디드 환경에서 최적화된 드라이버를 설계하는 역량을 증명하기 위해 진행되었습니다.

---

##  핵심 역량
* **Register-level Programming:** 주소 기반 포인터 조작을 통한 하드웨어 직접 제어.
* **Datasheet Analysis:** STM32 Reference Manual(RM0008) 분석을 통한 레지스터 구조 및 제어 시퀀스 설계.
* **Clock Tree Configuration:** RCC 설계를 통한 시스템 클록 및 주변장치 활성화.

---

##  구현된 주변장치

### 1. GPIO (General Purpose I/O)
* 모드 설정(Input, Output, Alternate Function) 및 비트 마스킹을 통한 고속 제어 구현.

### 2. UART (Universal Asynchronous RX/TX)
* 시스템 클록 기반 **Baud Rate 직접 계산** 및 ISR(Interrupt Service Routine)을 활용한 비동기 데이터 수신.
  <img width="785" height="303" alt="006I2C_tx_testing" src="https://github.com/user-attachments/assets/6e69d529-d0de-481b-b6cb-85811c9fbbd4" />

### 3. SPI (Serial Peripheral Interface)
* Full-duplex Master 모드 구현 및 통신 모드(CPOL, CPHA) 수동 설정을 통한 외부 센서 동기화.
  <img width="885" height="484" alt="004Spi_Tx_testing" src="https://github.com/user-attachments/assets/033b68dc-1761-4bc4-9f03-5909641d21b9" />
### 4. I2C (Inter-Integrated Circuit)
* 7-bit Addressing, Start/Stop/Ack 시그널링 직접 제어 및 버스 상태 모니터링 로직 구현.
  <img width="785" height="303" alt="006I2C_tx_testing" src="https://github.com/user-attachments/assets/6e69d529-d0de-481b-b6cb-85811c9fbbd4" />



