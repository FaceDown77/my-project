#include "/home/jwpark/miniProject/socket/serial/uart/uart.h"
#include "/home/jwpark/miniProject/socket/client/client.h"

int main() {
    Start_TCP_Communication_With_Server();
    Transmit_UART_Data_To_Server_Via_TCP();

    return 0;
}
