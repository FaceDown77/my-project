#ifndef GPIO_H_
#define GPIO_H_

# define F_CPU	16000000UL
#include <avr/io.h>
#include <util/delay.h>

enum {OUTPUT, INPUT};
enum {GPIO_PIN_RESET, GPIO_PIN_SET};
enum {PULL_UP, PULL_DOWN};


void gpio_init_port(volatile uint8_t* DDR, uint8_t dirMode);
void gpio_init_pin(volatile uint8_t* DDR, uint8_t dirMode, uint8_t pinNum);
void gpio_write_port(volatile uint8_t* PORT, uint8_t data);
void gpio_write_pin(volatile uint8_t* PORT, uint8_t pinNum, uint8_t state);
uint8_t gpio_read_port(volatile uint8_t* PIN);
uint8_t gpio_read_pin(volatile uint8_t* PIN, uint8_t pinNum);

#endif