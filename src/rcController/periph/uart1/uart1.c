#include "/home/jwpark/miniProject/rcController/periph/uart1/uart1.h"
# define BUF_SIZE    100

uint8_t uart1Rxbuff[BUF_SIZE];
uint8_t uart1RxFlag;
char str[BUF_SIZE];

void UART1_init() {
	// UART1, TxD, RxD, x2 mode, Baud Rate 9600
	// Default Set
	// 8bit Data bit, 1bit stop bit, Parity bit None
	
	// UART1, TxD, RxD
	UCSR1B |= ((1<<RXEN1) | (1<<TXEN1));
	// x2 mode, speed x2 up
	UCSR1A |= ((1<<U2X1) | (1<<UPE1));
	// 1byte received interrupt enable
	UCSR1B |= (1<<RXCIE1);
	// Baud Rate 9600
	UBRR1H = 0X00;
	UBRR1L = 207;
	//UBRR1L = 103;
	// Baud rate 115200
	//UBRR1L = 8;

    // 데이터 비트 설정: 8비트 전송
    UCSR1C = ((1<<UCSZ10) | (1<<UCSZ11));
}

void UART1_ISR_Process() {
	// 배열 리스트 사용
	// 문자열의 인덱스를 1씩 증가시킴으로써, 차례대로 문자를 받아오기 위함
	static uint8_t uart1RxTail = 0;
	
	uint8_t rx1Data = UDR1;
	
	// UART는 이스케이프 문자를 수신 거부 함
	// 이스케이프 문자를 모두 수신하고 마지막 문자를 0으로 설정함으로써 문자열 종료 후 수신 완료 세트
	if(rx1Data == '\n'|| rx1Data == '\r') {
		uart1Rxbuff[uart1RxTail] = rx1Data;
		uart1RxTail++;
		uart1Rxbuff[uart1RxTail] = 0;
		uart1RxTail = 0;
		// 수신 완료 상태 알림
		uart1RxFlag = 1;
	} else {
		uart1Rxbuff[uart1RxTail] = rx1Data;
		uart1RxTail++;
	}
}

void UART1_ISR_Buffer_Clear_Process() {
    static uint8_t uart1RxTail = 0;
    uint8_t rx1Data;

    // RXC1 비트가 1인지 확인
    if (UCSR1A & (1 << RXC1)) {
        // 오버런 에러 체크
        if (UCSR1A & (1 << DOR1)) {
            // 오버런 에러 발생 시 처리 (예: 버퍼 초기화)
            uart1RxTail = 0; // 버퍼 초기화
            // 필요 시 에러 로그 출력
            // UART1_sendString("Overrun Error!\r\n");
        } else {
            rx1Data = UDR1; // 데이터를 UDR1에서 읽어옴
            uart1Rxbuff[uart1RxTail] = rx1Data; // 수신 데이터를 버퍼에 저장
            uart1RxTail++;

            // 버퍼가 가득 차면 다시 초기화
            if (uart1RxTail >= BUF_SIZE) {
                uart1RxTail = 0; // 버퍼 인덱스 초기화
            }

            // 수신 완료 상태 확인 (예: LF 또는 CR 수신 시)
            if (rx1Data == '\n' || rx1Data == '\r') {
                uart1Rxbuff[uart1RxTail] = 0; // 문자열 종료
                uart1RxFlag = 1; // 수신 완료 플래그 설정
                uart1RxTail = 0; // 인덱스 초기화
            }
        }
    }
}


// ref Atmega128 data sheet 177p
void UART1_Transmit( unsigned char data ) {
	/* Wait for empty transmit buffer */
	// UDRE is Buffer
	// check UDRE is empty
	// UCSR0A & 001000000
	// if ( UCSR0A & (1<<UDRE0)) is 0, UDRE is empty
	// if ( UCSR0A & (1<<UDRE0)) is not 0, UDRE is full
	while ( !( UCSR1A & (1<<UDRE1)) )
	;
	/* Put data into buffer, sends the data */
	UDR1 = data;
}

void UART1_sendString(char *str) {
	for(int i = 0; str[i]; i++)
		UART1_Transmit(str[i]);

	//UART1_Transmit(0x0A);
}

// ref Atmega128 data sheet 180p
unsigned char UART1_Receive( void ) {
	/* Wait for data to be received */
	uint8_t cnt = 0;
	while ( !(UCSR1A & (1<<RXC1)) )
	;
	/* Get and return received data from buffer */
	cnt++;
	return UDR1;
}

uint8_t UART1_Is_Receive( void ) {
	/* Wait for data to be received */
	uint8_t cnt = 0;
	while ( !(UCSR1A & (1<<RXC1)) )
	;
	/* Get and return received data from buffer */
	cnt++;
	return cnt;
}

int UART1_TX_Complete() {
	// Tx 데이터가 없으면 0
	if(!(UCSR1A & (1<<TXC1)))
		return 0;
	else
		return 1;
}

int UART1_RX_Avail() {
	// RXC1 비트가 1로 설정되어 수신 완료
	// 프로그램에서 UDR1 레지스터에서 데이터 읽고, 자동으로 0으로 리셋 됨
	// RXC1 비트가 1일 때만 UDR1에서 데이터를 읽고 0으로 리셋
	if(!(UCSR1A & (1<<RXC1)))
		return 0;
	else
		return 1;
}

// UART 버퍼 비우기 함수
void UART1_Clear(void) {
    unsigned char dummy;
    while (UCSR1A & (1 << RXC1)) dummy = UDR1;
}

void UART1_clearRxFlag() {
	uart1RxFlag = 0;
}

void UART1_setRxFlag() {
	uart1RxFlag = 1;
}

uint8_t UART1_getRxFlag() {
	return uart1RxFlag;
}

uint8_t* UART1_readRxBuff() {
	return uart1Rxbuff;
}