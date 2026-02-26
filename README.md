# stm32f103rb-baremetal
🚀 STM32F103RB Bare-Metal Driver Development
본 프로젝트는 STM32F103RB(Cortex-M3) MCU의 주변장치를 HAL 라이브러리 없이 레지스터 직접 제어(Bare-Metal) 방식으로 드라이버를 설계하고 구현한 실습 기록입니다.

📌 프로젝트 개요
목적: MCU 내부 레지스터 구조 이해 및 하드웨어 제어 역량 강화

타겟 보드: STM32F103RB (NUCLEO-F103RB)

개발 환경: STM32CubeIDE, GNU Arm Embedded Toolchain

주요 특징:

CMSIS 라이브러리를 활용한 레지스터 맵핑

Polling 및 Interrupt 기반의 데이터 송수신 구현

각 주변장치별 독립적인 드라이버 구조 설계

📂 폴더 구조
Src/: 각 주변장치별 테스트 애플리케이션 (Main logic)

drivers/Inc/: 주변장치 드라이버 헤더 파일 (레지스터 정의 및 API 선언)

drivers/Src/: 주변장치 드라이버 소스 파일 (드라이버 로직 구현)

Startup/: MCU 부팅 시퀀스 및 벡터 테이블 설정

💻 구현 및 실습 내용
1. GPIO (General Purpose I/O)
실습: 001ToggleLED.c, 002ButtonLEDPolling.c, 003ButtonLEDInterrupt.c

내용:

Push-pull/Open-drain 모드 설정 및 속도 제어

외부 인터럽트(EXTI)를 활용한 버튼 입력 처리

2. SPI (Serial Peripheral Interface)
실습: 004Spi_Tx_testing.c, 005Spi_Tx_arduino.c, 006spi_cmd_handle.c

내용:

Full-duplex 통신 모드 구현

Arduino와 연동한 Master-Slave 데이터 송수신 및 커맨드 핸들링

3. I2C (Inter-Integrated Circuit)
실습: 007I2C_Tx_arduino.c, 008I2C_Rx_arduino.c, 009I2C_RX_IT_arduino.c

내용:

I2C 프로토콜을 이용한 데이터 마스터 송수신 구현

Interrupt 기반 비동기 데이터 수신 로직 적용

4. USART (Universal Synchronous/Asynchronous Receiver Transmitter)
실습: 010USART_Tx_arduino.c, 011USART_Tx_IT_arduino.c

내용:

보드레이트(Baudrate) 계산 및 설정

시리얼 통신을 통한 디버깅 데이터 전송

5. RTC (Real Time Clock) & Others
실습: 012rtc_lcd.c, button.c, pulluptest.c

내용:

RTC를 활용한 시간 데이터 관리 및 LCD 출력 연동

내부 Pull-up/Pull-down 저항 설정 테스트

🛠️ 기술적 도전 및 해결
인터럽트 동기화: volatile 키워드를 사용하여 인터럽트 서비스 루틴(ISR) 내 변수값의 최적화 문제를 해결하였습니다.

통신 프로토콜 디버깅: 로직 분석기를 활용하여 SPI/I2C 통신 시 데이터 프레임 및 타이밍을 분석하며 드라이버의 신뢰성을 확보했습니다.
