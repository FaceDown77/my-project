#include "/home/jwpark/miniProject/rcController/app/service/control_patrol_car.h"
#include <stdio.h>
#include <string.h>

//uint8_t joystickDataLen;

void Send_Data_Joystick_Via_Wifi() {
    //joystickDataLen = 0;

    //Operate_X_Axis_Joystick();
    //Operate_Y_Axis_Joystick();

    //_delay_ms(50);

    //joystickDataLen = Caculate_Joystick_X_Axis_Data_Len(joystickDataLen);
    //joystickDataLen = Caculate_Joystick_Y_Axis_Data_Len(joystickDataLen);

    //_delay_ms(50);

    //ESP8266_Start_Socket_Communication();

    //_delay_ms(50);

	// 조이스틱 X 좌표 값 전송
    // 모니터링 또는 소켓 통신용
    Operate_X_Axis_Joystick();
	UART0_Transmit('X');
	UART0_Transmit(':');
	UART0_Transmit(' ');
    Transmit_Joystick_X_Axis_Data_To_UART0();
    UART0_Transmit(' ');
    //ESP8266 데이터 전송 용
	//UART1_Transmit('X');
	//UART1_Transmit(':');
	//UART1_Transmit(' ');
    //Transmit_Joystick_X_Axis_Data_To_UART1();
    //UART1_Transmit(' ');
    //sprintf(buffer, "X: %hu", joystickXdata);
    //UART1_sendString(buffer);


	// 조이스틱 Y 좌표 값 전송
    // 모니터링 또는 소켓 통신용
    Operate_Y_Axis_Joystick();
	UART0_Transmit('Y');
	UART0_Transmit(':');
	UART0_Transmit(' ');
	Transmit_Joystick_Y_Axis_Data_To_UART0();
    UART0_Transmit(' ');
    //ESP8266 데이터 전송 용
    //UART1_Transmit('Y');
	//UART1_Transmit(':');
	//UART1_Transmit(' ');
	//Transmit_Joystick_Y_Axis_Data_To_UART1();
    //UART1_Transmit(' ');

    // 조이스틱 스위치 값 전송 (1 or 0)
    Check_Event_Listener();

    // 개행
	UART0_Transmit('\n');
	UART0_Transmit('\r');
	//UART1_Transmit('\n');
    //UART1_Transmit('\r');
}

void Execute_Car_Via_Wifi() {
    Wifi_Connect_Test();
    Send_Data_Joystick_Via_Wifi();
}