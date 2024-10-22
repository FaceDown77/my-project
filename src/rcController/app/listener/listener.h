#ifndef LISTENER_H_
#define LISTENER_H_

#include "/home/jwpark/miniProject/rcController/driver/button/button.h"
#include "/home/jwpark/miniProject/rcController/periph/uart0/uart0.h"
#include "/home/jwpark/miniProject/rcController/periph/uart1/uart1.h"

void Listener_Init();
void Check_Event_Is_Auto_Or_Manual_Mode();
void Check_Event_Listener();

#endif 