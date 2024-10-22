#ifndef ATCOMMANDS_H
#define ATCOMMANDS_H

# define F_CPU	16000000UL
# define BUF_SIZE   128
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>

#include "/home/jwpark/miniProject/rcController/periph/uart0/uart0.h"
#include "/home/jwpark/miniProject/rcController/periph/uart1/uart1.h"

#define TRUE 1
#define FALSE 0

void ISR_AT_Command_Read_Response_Process(const char *expectedReply, uint16_t timeout);
void AT_Send_Blind_Command(char *command);
int AT_Send_Command(char *command, uint16_t timeout);
int AT_Send_Command_And_Receive_Reply(char *command, const char *reply, uint16_t timeout);

#endif
