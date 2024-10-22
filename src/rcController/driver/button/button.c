#include "/home/jwpark/miniProject/rcController/driver/button/button.h"

void Button_Init(button_t *btn, volatile uint8_t* ddr, volatile uint8_t* pin, uint8_t pinNum) {
    btn->DDR = ddr;
    btn->PIN = pin;
    btn->pinNum = pinNum;
    btn->prevState = INIT_BUTTON_STATE;
    //*btn->DDR &= ~(1<<btn->pinNum);
    gpio_init_pin(btn->DDR, INPUT, btn->pinNum);
}

uint8_t Get_Button_State(button_t* btn) {
    //uint8_t curState = *btn->PIN & (1<<btn->pinNum);
    uint8_t curState = gpio_read_pin(btn->PIN, btn->pinNum);
    // Low
    if((curState == PUSHED) && (btn->prevState == RELEASED)) {
        _delay_ms(10);
        btn->prevState = PUSHED;
        return ACT_PUSHED;
    // High    
    } else if ((curState != PUSHED) && (btn->prevState == PUSHED)) {
        _delay_ms(10);
        btn->prevState = RELEASED;
        return ACT_RELEASED;
    }
}