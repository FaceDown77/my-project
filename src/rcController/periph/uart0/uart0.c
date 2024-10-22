#include "/home/jwpark/miniProject/rcController/periph/uart0/uart0.h"
# define BUF_SIZE    100

uint8_t uart0Rxbuff[BUF_SIZE];
uint8_t uart0RxFlag;
char str[BUF_SIZE];

void UART0_init() {
	// UART0, TxD, RxD, x2 mode, Baud Rate 9600
	// Default Set
	// 8bit Data bit, 1bit stop bit, Parity bit None
	
	// UART0, TxD, RxD
	UCSR0B |= ((1<<RXEN0) | (1<<TXEN0));
	// x2 mode, speed x2 up
	UCSR0A |= ((1<<U2X0) | (1<<UPE0));
	// 1byte received interrupt enable
	UCSR0B |= (1<<RXCIE0);
	// Baud Rate 9600
	UBRR0H = 0X00;
	UBRR0L = 207;
	//UBRR0L = 103;
	// Baud rate 115200
	//UBRR0L = 17;
	//UBRR0L = 8;

    // 데이터 비트 설정: 8비트 전송
    //UCSR0C = ((1<<UCSZ00) | (1<<UCSZ01));
}

void UART0_ISR_Process() {
	// 배열 리스트 사용
	// 문자열의 인덱스를 1씩 증가시킴으로써, 차례대로 문자를 받아오기 위함
	static uint8_t uart0RxTail = 0;
	
	uint8_t rx0Data = UDR0;

	// UART는 이스케이프 문자를 수신 거부 함
	// 이스케이프 문자를 모두 수신하고 마지막 문자를 0으로 설정함으로써 문자열 종료 후 수신 완료 세트
	if(rx0Data == '\n') {	//LF가 입력되면
		uart0Rxbuff[uart0RxTail] = rx0Data;
		uart0RxTail++;
		uart0Rxbuff[uart0RxTail] = 0;
		uart0RxTail = 0;
		// 수신 완료 상태 알림
		uart0RxFlag = 1;
	} else {
		uart0Rxbuff[uart0RxTail] = rx0Data;
		uart0RxTail++;
	}
}

void UART0_ISR_Buffer_Clear_Process() {
    static uint8_t uart0RxTail = 0;
    uint8_t rx0Data;

    // RXC1 비트가 1인지 확인
    if (UCSR0A & (1 << RXC0)) {
        // 오버런 에러 체크
        if (UCSR0A & (1 << DOR0)) {
            // 오버런 에러 발생 시 처리 (예: 버퍼 초기화)
            uart0RxTail = 0; // 버퍼 초기화
            // 필요 시 에러 로그 출력
            // UART1_sendString("Overrun Error!\r\n");
        } else {
            rx0Data = UDR0; // 데이터를 UDR1에서 읽어옴
            uart0Rxbuff[uart0RxTail] = rx0Data; // 수신 데이터를 버퍼에 저장
            uart0RxTail++;

            // 버퍼가 가득 차면 다시 초기화
            if (uart0RxTail >= BUF_SIZE) {
                uart0RxTail = 0; // 버퍼 인덱스 초기화
            }

            // 수신 완료 상태 확인
            if (rx0Data == '\n') {
                uart0Rxbuff[uart0RxTail] = 0; // 문자열 종료
                uart0RxFlag = 1; // 수신 완료 플래그 설정
                uart0RxTail = 0; // 인덱스 초기화
            }
        }
    }
}


// ref Atmega128 data sheet 177p
void UART0_Transmit( unsigned char data ) {
	/* Wait for empty transmit buffer */
	// UDRE is Buffer
	// check UDRE is empty
	// UCSR0A & 001000000
	// if ( UCSR0A & (1<<UDRE0)) is 0, UDRE is empty
	// if ( UCSR0A & (1<<UDRE0)) is not 0, UDRE is full
	while ( !( UCSR0A & (1<<UDRE0)) )
	;
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

void UART0_sendString(char *str) {
	for(int i = 0; str[i]; i++)
		UART0_Transmit(str[i]);

	//UART0_Transmit(0x0A);
}

// ref Atmega128 data sheet 180p
unsigned char UART0_Receive( void ) {
	/* Wait for data to be received */
	uint8_t cnt = 0;
	while ( !(UCSR0A & (1<<RXC0)) )
	;
	/* Get and return received data from buffer */
	cnt++;
	return UDR0;
}

uint8_t UART0_Is_Receive( void ) {
	/* Wait for data to be received */
	uint8_t cnt = 0;
	while ( !(UCSR0A & (1<<RXC0)) )
	;
	/* Get and return received data from buffer */
	cnt++;
	return cnt;
}

int UART0_TX_Complete() {
	// Tx 데이터가 없으면 0
	if(!(UCSR0A & (1<<TXC0)))
		return 0;
	else
		return 1;
}

int UART0_RX_Avail() {
	// RXC0 비트가 1로 설정되어 수신 완료
	// 프로그램에서 UDR0 레지스터에서 데이터 읽고, 자동으로 0으로 리셋 됨
	// RXC0 비트가 1일 때만 UDR0에서 데이터를 읽고 0으로 리셋
	if(!(UCSR0A & (1<<RXC0)))
		return 0;
	else
		return 1;
}

void UART0_Clear(void) {
    unsigned char dummy;
    while (UCSR0A & (1 << RXC0)) dummy = UDR0;
}

void UART0_clearRxFlag() {
	uart0RxFlag = 0;
}

void UART0_setRxFlag() {
	uart0RxFlag = 1;
}

uint8_t UART0_getRxFlag() {
	return uart0RxFlag;
}

uint8_t* UART0_readRxBuff() {
	return uart0Rxbuff;
}