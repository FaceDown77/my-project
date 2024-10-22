#ifndef UART1_H_
#define UART1_H_

# define F_CPU	16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

//# define BUF_SIZE   128

void UART1_init();

void UART1_ISR_Process();
void UART1_ISR_Buffer_Clear_Process();

void UART1_Transmit( unsigned char data );
unsigned char UART1_Receive( void );
uint8_t UART1_Is_Receive( void );
void UART1_sendString(char *str);

int UART1_TX_Complete();
int UART1_RX_Avail();

void UART1_Clear(void);

void UART1_clearRxFlag();
void UART1_setRxFlag();
uint8_t UART1_getRxFlag();
uint8_t* UART1_readRxBuff();

#endif