
# define F_CPU	16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#include "/home/jwpark/miniProject/rcController/app/appMain.h"
#include "/home/jwpark/miniProject/rcController/periph/uart0/uart0.h"

int main(void)
{
  //DDRG = 0xff;
  
  appMain_init();

  while(1) {
    appMain_execute();
    //uart0_transmit(uart0_receive());

  }
  
  return 0;
}