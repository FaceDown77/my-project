#include "/home/jwpark/miniProject/socket/serial/uart/uart.h"

// UART 초기화 함수
// ATmega128과 연결된 UART 장치 파일을 열고, UART로부터 데이터를 수신
int Uart_Init(const char *device) {
    int uart_fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_fd == -1) {
        perror("Failed to open UART device");
        return -1;
    }

    struct termios options;
    tcgetattr(uart_fd, &options);

    // UART 설정: Baud rate, 데이터 비트, 패리티 비트, 스톱 비트 등

    // 9600 baud rate 설정
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    options.c_cflag |= (CLOCAL | CREAD);
    // 패리티 비트 사용 안함
    options.c_cflag &= ~PARENB;
    // 1 스톱 비트
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
     // 8 데이터 비트
    options.c_cflag |= CS8;

    tcsetattr(uart_fd, TCSANOW, &options);
    return uart_fd;
}