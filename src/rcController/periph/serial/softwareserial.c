#include "/home/jwpark/miniProject/rcController/periph/serial/softwareserial.h"

// 각종 딜레이 값 저장
volatile uint16_t _tx_delay = 0;
volatile uint16_t _rx_delay_centering = 0;
volatile uint16_t _rx_delay_intrabit = 0;
volatile uint16_t _rx_delay_stopbit = 0;

uint16_t Calculate_Bit_Delay(long speed) {
    return (F_CPU / speed) / 4;
}

uint16_t Subtract_Cap(uint16_t value, uint16_t sub) {
    if (value > sub) {
        return value - sub;
    } else {
        return 1;  // 최소 값은 1
    }
}

void delay_us(uint16_t microseconds) {
    while (microseconds--) {
        // AVR 마이크로컨트롤러의 클럭 속도에 따라 적절한 루프 카운트를 사용합니다.
        // 예를 들어 1us 지연을 위해 다음과 같은 코드를 사용합니다.
        // F_CPU가 16MHz인 경우 이 값을 조정해야 합니다.
        // 이 값은 실험을 통해 조정해야 할 수 있습니다.
        _delay_loop_2(F_CPU / 1000000);  // 1us 지연
    }
}

// 초기화 함수
void Software_Serial_Begin(long speed) {
    uint16_t bit_delay = Calculate_Bit_Delay(speed);

    // 송신용 딜레이 계산
    _tx_delay = Subtract_Cap(bit_delay, 15 / 4);

    // 수신용 딜레이 계산 (센터링, 인터비트, 스탑비트)
    _rx_delay_centering = Subtract_Cap(bit_delay / 2, (75 + 17 - 23) / 4);
    _rx_delay_intrabit = Subtract_Cap(bit_delay, 23 / 4);
    _rx_delay_stopbit = Subtract_Cap(bit_delay * 3 / 4, (37 + 11) / 4);

    // RX
    //DDRB &= ~(1 << RX_PIN);  // 수신 핀 입력 설정
    //PORTB |= (1 << RX_PIN);  // 풀업 저항 활성화
    gpio_init_pin(&DDRD, INPUT, 2);
    gpio_write_pin(&PORTD, 2, GPIO_PIN_RESET);

    // TX
    //DDRB |= (1 << TX_PIN);   // 송신 핀 출력 설정
    //PORTB |= (1 << TX_PIN);  // 송신 핀 기본 HIGH 유지 (idle)
    gpio_init_pin(&DDRD, OUTPUT, 3);
    gpio_write_pin(&PORTD, 3, GPIO_PIN_RESET);

    // 핀 체인지 인터럽트 활성화
    //PCICR |= (1 << PCIE0);   // PCINT0 활성화 (핀 변화 인터럽트)
    // INT2 활성화
    EIMSK |= (1<<INT2);
    // RX_PIN에 대해 핀 변화(High Edge) 인터럽트 활성화
    EICRB |= ((1<<ISC21) | (1<<ISC20));
}

// 송신 함수
void Software_Serial_Write_Char(char data) {
    // Start bit: LOW
    PORTD &= ~(1 << UART1_TX_PIN);
    delay_us(_tx_delay);

    // 데이터 비트 전송 (LSB부터)
    for (uint8_t i = 0; i < 8; i++) {
        if (data & (1 << i)) {
            PORTD |= (1 << UART1_TX_PIN);  // HIGH 전송
        } else {
            PORTD &= ~(1 << UART1_TX_PIN); // LOW 전송
        }
        delay_us(_tx_delay);
    }

    // Stop bit: HIGH
    PORTD |= (1 << UART1_TX_PIN);
    delay_us(_tx_delay);
}

// 수신 함수 (PCINT 인터럽트에서 처리)
void ISR_Software_Serial_Read_Process() {
    if (!(PIND & (1 << UART1_RX_PIN))) {  // 수신 시작 (start bit 감지)
        delay_us(_rx_delay_centering);  // 첫 비트 읽기 위한 딜레이

        char received_data = 0;

        // 8비트 데이터 읽기
        //for (uint8_t i = 0; i < 8; i++) {
        //    delay_us(_rx_delay_intrabit);  // 각 비트 간의 딜레이
        //    if (PIND & (1 << UART1_RX_PIN)) {
        //        received_data |= (1 << i);  // HIGH 비트 읽기
        //    }
        //}

        //if (UART1_RX_Avail()) {
        //    if(UART1_getRxFlag()) {
        //        UART1_clearRxFlag();
        //        replyString = UART1_readRxBuff();
        //    }


        // 스탑비트 처리
        delay_us(_rx_delay_stopbit);

        // 수신한 데이터 처리 (예: UART 전송 등)
        Software_Serial_Write_Char(received_data);
    }
}