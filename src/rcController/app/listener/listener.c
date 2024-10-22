
#include "/home/jwpark/miniProject/rcController/app/listener/listener.h"

button_t btnAutoManualModeSelect;

void Listener_Init() {
    // 조이스틱 스위치 핀 초기화
    Button_Init(&btnAutoManualModeSelect, &DDRF, &PINF, 2);
}

void Check_Event_Is_Auto_Or_Manual_Mode() {
    // 스위치를 누르면 0, 떼면 1 전달
    if(Get_Button_State(&btnAutoManualModeSelect) == ACT_RELEASED) {
        UART0_Transmit('S');
        UART0_Transmit('W');
        UART0_Transmit(':');
        UART0_Transmit(' ');
        UART0_Transmit('1');
        UART0_Transmit(' ');
    } else {
        UART0_Transmit('S');
        UART0_Transmit('W');
        UART0_Transmit(':');
        UART0_Transmit(' ');
        UART0_Transmit('0');
        UART0_Transmit(' ');    
    }

}

void Check_Event_Listener() {
    Check_Event_Is_Auto_Or_Manual_Mode();
}