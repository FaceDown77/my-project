#ifndef JOYSTICK_H_
#define JOYSTICK_H__

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "/home/jwpark/miniProject/rcController/periph/gpio/gpio.h"

#define INIT_BUTTON_STATE   1

enum {PUSHED, RELEASED};
enum {ACT_PUSHED, ACT_RELEASED, ACT_NONE};

typedef struct _button
{
	volatile uint8_t* DDR;
	volatile uint8_t* PIN;
	uint8_t pinNum;
	uint8_t prevState;
	// port
	// pin num
	// static prevState
}button_t;

void Button_Init(button_t *btn, volatile uint8_t* ddr, volatile uint8_t* pin, uint8_t pinNum);
uint8_t Get_Button_State(button_t* btn);

#endif