#ifndef JOYSTICK_H_
#define JOYSTICK_H__

# define F_CPU	16000000UL
#include <avr/io.h>
#include <util/delay.h>

void ADC0_Init();
void ADC1_Init();
unsigned short ADC_Read();

#endif