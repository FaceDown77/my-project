#include "/home/jwpark/miniProject/rcController/periph/timer/timer.h"

// 1ms overflow Interrupt, FND display
void TIMER0_Init() {
	// Timer/Counter
	// Register – TCNT0
	// Atmega128 datasheet 105p
	// 128 prescaler, 1ms interrupt
	// timer/counter 0(8bit)
	// register
	// 1/125000 * 125
	// convert time of period from 2048 micro seconds to 1000 micro seconds(1ms)
	TCNT0 = 130;
	// Timer/Counter
	// Register – TCNT0
	// Atmega128 datasheet 103, 105p
	// clock select bits set
	//1024 prescaler
	//TCCR0 |= ((1<<CS02) | (1<<CS01) | (1<<CS00));
	// 128 prescaler
	TCCR0 |= ((1<<CS02) | (0<<CS01) | (1<<CS00));
	// Timer/Counter
	// Interrupt Mask
	// Register – TIMSK
	// Atmega128 datasheet 108p
	// Overflow Interrupt Enable
	// mask
	TIMSK |= (1<<TOIE0);
}

void TIMER2_Init() {
	// 1/64 prescalser
	TCCR2 |= ((0<<CS22) | (1<<CS21) | (1<<CS20));
	// mask
	TIMSK = (1<<OCIE2);
	// CTC mode
	TCCR2 |= ((1<<WGM21) | (0<<WGM20));
	// OCR
	OCR2 = 250-1;
}