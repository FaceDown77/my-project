#include "/home/jwpark/miniProject/rcController/test/test.h"

void Wifi_Connect_Test() {
	// UART
	if(UART0_getRxFlag()) {
		UART0_clearRxFlag();
		uint8_t* rxString = UART0_readRxBuff();

        if (!strcmp(rxString, "AT\r\n")) {
            PORTG = (1<<0);
            if (AT_Send_Command_And_Receive_Reply(rxString, "OK\r\n", 2000))
                UART0_sendString("Success!\r\n");
            else 
                UART0_sendString("ESP8266 no response\r\n");
        } else if (!strcmp(rxString, "AT+RST\r\n")) {
            PORTG = (1<<0);
            if (AT_Send_Command_And_Receive_Reply(rxString, "OK\r\n", 2000))
                UART0_sendString("Success!\r\n");
            else 
                UART0_sendString("ESP8266 no response\r\n");
        } else if (!strcmp(rxString, "AT+CWMODE=1\r\n")) {
            PORTG = (1<<0);
            if (AT_Send_Command_And_Receive_Reply(rxString, "OK\r\n", 2000))
                UART0_sendString("Success!\r\n");
            else 
                UART0_sendString("ESP8266 no response\r\n");
        } else if (!strcmp(rxString, "AT+CWMODE=2\r\n")) {
            PORTG = (1<<0);
            if (AT_Send_Command_And_Receive_Reply(rxString, "OK\r\n", 2000))
                UART0_sendString("Success!\r\n");
            else 
                UART0_sendString("ESP8266 no response\r\n");
        } else if (!strcmp(rxString, "AT+CWMODE=3\r\n")) {
            PORTG = (1<<0);
            if (AT_Send_Command_And_Receive_Reply(rxString, "OK\r\n", 2000))
                UART0_sendString("Success!\r\n");
            else 
                UART0_sendString("ESP8266 no response\r\n");
        } else if (!strcmp(rxString, "AT+CWJAP=\"facedown\",\"dlsgur7053\"\r\n")) {
            PORTG = (1<<0);
            if (AT_Send_Command_And_Receive_Reply(rxString, "OK\r\n", 2000))
                UART0_sendString("Success!\r\n");
            else 
                UART0_sendString("ESP8266 no response\r\n");
        } else if (!strcmp(rxString, "AT+CIPSTART=\"TCP\",\"192.168.0.13\",5000\r\n")) {
            PORTG = (1<<0);
            if (AT_Send_Command_And_Receive_Reply(rxString, "OK\r\n", 2000))
                UART0_sendString("Success!\r\n");
            else 
                UART0_sendString("ESP8266 no response\r\n");
        } else if (!strcmp(rxString, "AT+CIPSTART=\"TCP\",\"192.168.0.12\",5000\r\n")) {
            PORTG = (1<<0);
            if (AT_Send_Command_And_Receive_Reply(rxString, "OK\r\n", 2000))
                UART0_sendString("Success!\r\n");
            else 
                UART0_sendString("ESP8266 no response\r\n");
        } else if (!strcmp(rxString, "AT+CIPSTART=\"TCP\",\"192.168.0.11\",5000\r\n")) {
            PORTG = (1<<0);
            if (AT_Send_Command_And_Receive_Reply(rxString, "OK\r\n", 2000))
                UART0_sendString("Success!\r\n");
            else 
                UART0_sendString("ESP8266 no response\r\n");
        } else if (!strcmp(rxString, "AT+CIPSEND\r\n")) {
            PORTG = (1<<0);
            if (AT_Send_Command_And_Receive_Reply(rxString, "OK\r\n", 2000))
                UART0_sendString("Success!\r\n");
            else 
                UART0_sendString("ESP8266 no response\r\n");
        } else if (!strcmp(rxString, "AT+CIPCLOSE\r\n")) {
            PORTG = (1<<0);
            if (AT_Send_Command_And_Receive_Reply(rxString, "OK\r\n", 2000))
                UART0_sendString("Success!\r\n");
            else 
                UART0_sendString("ESP8266 no response\r\n");
        }

    }
}