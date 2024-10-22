#ifndef LISTENER_H_
#define LISTENER_H_

# define F_CPU	16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>

#include "/home/jwpark/miniProject/rcController/periph/uart0/uart0.h"
#include "/home/jwpark/miniProject/rcController/periph/uart1/uart1.h"
#include "/home/jwpark/miniProject/rcController/app/command/at/atcommands.h"

void Wifi_Connect_Test();

#endif
