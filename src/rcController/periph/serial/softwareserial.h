#ifndef SOFTWARE_SERIAL_H_
#define SOFTWARE_SERIAL_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define F_CPU 16000000UL  // 16 MHz 클럭
#define BAUD_RATE 9600

#include "/home/jwpark/miniProject/rcController/periph/gpio/gpio.h"

// UART0 RX
#define UART1_RX_PIN 2
// UART0 TX
#define UART1_TX_PIN 3

uint16_t Calculate_Bit_Delay(long speed);
uint16_t Subtract_Cap(uint16_t value, uint16_t sub);
void Software_Serial_Begin(long speed);
void Software_Serial_Write_Char(char data);
void ISR_Software_Serial_Read_Process();

#endif