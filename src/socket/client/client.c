#include "/home/jwpark/miniProject/socket/client/client.h"

int uart_fd, socket_fd;
struct sockaddr_in server_addr;
char buffer[256];
int bytes_read;

// 서버로 데이터 전송
void Send_Data_To_Server(const char *data, int socket_fd) {
    if (send(socket_fd, data, strlen(data), 0) < 0) {
        perror("Failed to send data to server");
    } else {
        printf("Sent: %s\n", data);
    }
}

void Start_TCP_Communication_With_Server() {
    // UART 초기화
    uart_fd = Uart_Init(UART_DEVICE);
    if (uart_fd == -1)
        return;

    // 소켓 생성
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd == -1) {
        perror("Failed to create socket");
        close(uart_fd);
        return;
    }

    // 서버 주소 설정
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);

    // 서버에 연결
    if (connect(socket_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1) {
        perror("Failed to connect to server");
        close(uart_fd);
        close(socket_fd);
        return;
    }

    printf("Connected to server %s on port %d\n", SERVER_IP, SERVER_PORT);
}

void Transmit_UART_Data_To_Server_Via_TCP() {
    // UART로부터 데이터 읽고 서버로 전송
    while (1) {
        // UART에서 데이터 읽기
        memset(buffer, 0, sizeof(buffer));
        bytes_read = read(uart_fd, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';  // Null-terminate the string

            // 서버로 전송
            Send_Data_To_Server(buffer, socket_fd);

            // 서버로부터 응답 받기
            memset(buffer, 0, sizeof(buffer));
            if (recv(socket_fd, buffer, sizeof(buffer) - 1, 0) > 0) {
                printf("Received from server: %s\n", buffer);
            }
        }

        usleep(110000);  // 0.2초 대기
    }

    // 연결 종료
    close(uart_fd);
    close(socket_fd);
}