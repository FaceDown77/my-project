#ifndef APMAIN_H_
#define APMAIN_H_

# define F_CPU	16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <stdio.h>
#include <string.h>

#include "/home/jwpark/miniProject/rcController/app/listener/listener.h"
#include "/home/jwpark/miniProject/rcController/periph/uart1/uart1.h"
#include "/home/jwpark/miniProject/rcController/app/service/control_patrol_car.h"
#include "/home/jwpark/miniProject/rcController/periph/gpio/gpio.h"
#include "/home/jwpark/miniProject/rcController/driver/wifi/esp8266.h"
#include "/home/jwpark/miniProject/rcController/periph/adc/adc.h"
#include "/home/jwpark/miniProject/rcController/periph/serial/softwareserial.h"
#include "/home/jwpark/miniProject/rcController/app/command/at/atcommands.h"

int UART0_Putc(char c, FILE *stream);
int UART0_Getc();
int UART1_Putc(char c, FILE *stream);
int UART1_Getc();

void On_LED_For_Debuggig(unsigned short value);

void appMain_init();
void appMain_execute();



#endif