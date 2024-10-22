
#ifndef CLIENT_H
#define CLIENT_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <termios.h>
#include <fcntl.h>

#include "/home/jwpark/miniProject/socket/serial/uart/uart.h"

# define SERVER_IP      "192.168.0.12"
# define SERVER_PORT    5000

# define UART_DEVICE    "/dev/ttyUSB0"

void Send_Data_To_Server(const char *data, int socket_fd);
void Start_TCP_Communication_With_Server();
void Transmit_UART_Data_To_Server_Via_TCP();

#endif