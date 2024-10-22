#ifndef CONTROL_PATROL_CAR_
#define CONTROL_PATROL_CAR_

# define F_CPU	16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>

#include "/home/jwpark/miniProject/rcController/app/listener/listener.h"
#include "/home/jwpark/miniProject/rcController/periph/uart0/uart0.h"
#include "/home/jwpark/miniProject/rcController/periph/uart1/uart1.h"
#include "/home/jwpark/miniProject/rcController/driver/joystick/joystick.h"
#include "/home/jwpark/miniProject/rcController/app/command/at/atcommands.h"
#include "/home/jwpark/miniProject/rcController/test/test.h"

void Send_Data_Joystick_Via_Wifi();
void Wifi_Connect_Test();
void ESP8266_Start_Socket_Communication();
void Execute_Car_Via_Wifi();

#endif 