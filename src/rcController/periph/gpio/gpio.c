#include "/home/jwpark/miniProject/rcController/periph/gpio/gpio.h"

// 특정 포트에 대한 모든 핀을 동일한 방향 모드로 설정
void gpio_init_port(volatile uint8_t* DDR, uint8_t dirMode) {
    if(dirMode == OUTPUT)
        *DDR = 0xff;
    else
        *DDR = 0x00;
}

// 특정 포트에 대한 특정 핀을 지정한 방향 모드로 설정
void gpio_init_pin(volatile uint8_t* DDR, uint8_t dirMode, uint8_t pinNum) {
    /* example to refrence mode */
    // set mode of input
	// DDRA &= ~(1<<0);
    // set mode of output
	// DDRD |= (1<<0);

    if(dirMode == OUTPUT)
        *DDR |= (1<<pinNum);
    else
        *DDR &= ~(1<<pinNum);
}

// 특정 포트에 대한 모든 핀에 HIGH(5V) 또는 LOW(0V) 출력
void gpio_write_port(volatile uint8_t* PORT, uint8_t data) {
    *PORT = data;
}

// 특정 포트에 대한 특정 핀에 HIGH(5V) 또는 LOW(0V) 출력
void gpio_write_pin(volatile uint8_t* PORT, uint8_t pinNum, uint8_t state) {
    // PORTD |= (1<<0);
    // 풀업 설정
    if(state == GPIO_PIN_RESET)
        *PORT |= (1<<pinNum);
    else
        *PORT &= ~(1<<pinNum);
}

// 특정 포트에 대한 모든 핀의 HIGH(5V) 또는 LOW(0V)인지의 상태
uint8_t gpio_read_port(volatile uint8_t* PIN) {
    return *PIN;
}

// 특정 포트에 대한 특정 핀의 HIGH(5V) 또는 LOW(0V)인지의 상태
// High일 때 True, Low일 때 False
uint8_t gpio_read_pin(volatile uint8_t* PIN, uint8_t pinNum) {
    return ((*PIN & (1<<pinNum)) != 0);
}