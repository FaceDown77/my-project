#include "/home/jwpark/miniProject/rcController/app/appMain.h"

// UART0 송신 함수
int UART0_Putc(char c, FILE *stream) {
    UART0_Transmit(c);
    return 0;
}

// UART0 수신 함수
int UART0_Getc() {
    return (int)UART0_Receive();
}

// UART0 데이터 I/O 파일 스트림
FILE OUTPUT_UART0 = FDEV_SETUP_STREAM(UART0_Putc, NULL, _FDEV_SETUP_WRITE);
FILE INPUT_UART0 = FDEV_SETUP_STREAM(NULL, UART0_Getc, _FDEV_SETUP_READ);

ISR(USART0_RX_vect) {
    UART0_ISR_Process();
    UART0_ISR_Buffer_Clear_Process();
}

ISR(USART1_RX_vect) {
    UART1_ISR_Process();
    UART1_ISR_Buffer_Clear_Process();
}

void appMain_init() {
    // UART0을 표준 입출력으로 사용
    stdout = &OUTPUT_UART0;
    stdin = &INPUT_UART0;

    // 디버깅용 LED
    PORTG = 0X00;

    sei();

    UART0_init();
    UART1_init();

    // Supply power for device if necessary
    gpio_init_port(&DDRA, OUTPUT);
    // LED on for debugging
    gpio_init_port(&DDRD, OUTPUT);

    // UART1 RX
    gpio_init_pin(&DDRD, INPUT, 2);
    // UART1  TX
    gpio_init_pin(&DDRD, OUTPUT, 3);

    // ADC input channel 0
    gpio_init_pin(&DDRF, INPUT, 0);
    // ADC input channel 1
    gpio_init_pin(&DDRF, INPUT, 1);
    // Joystick switch pin
    gpio_init_pin(&DDRF, INPUT, 2);
    // Joystick switch pin set to pull up
    gpio_write_pin(&PORTF, 2, GPIO_PIN_RESET);
    // Init state of pin
    Listener_Init();
}

void appMain_execute() {
    // Pin of power on
    PORTA |= ((1<<PA0) | (1<<PA1));

    //Control_Car_Via_Bluetooth();
    Execute_Car_Via_Wifi();
}