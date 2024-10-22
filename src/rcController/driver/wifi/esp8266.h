#ifndef ESP8266_H__
#define ESP8266_H__

#define F_CPU 16000000UL		
#include <avr/io.h>				
#include <util/delay.h>			
#include <stdbool.h>			
#include <string.h>				
#include <stdio.h>				
#include <stdlib.h>				
#include <avr/interrupt.h>	
#include "/home/jwpark/miniProject/rcController/periph/uart1/uart1.h"
#include "/home/jwpark/miniProject/rcController/app/command/at/atcommands.h"

#define SREG    _SFR_IO8(0x3F)

#define DEFAULT_BUFFER_SIZE		128
#define DEFAULT_TIMEOUT			10000

/* Connection Mode */
#define SINGLE				0
#define MULTIPLE			1

/* Application Mode */
#define NORMAL				0
#define TRANSPERANT			1

/* Application Mode */
#define STATION				1
#define ACCESSPOINT			2
#define BOTH_STATION_AND_ACCESPOINT	3

/* Select Demo */
#define RECEIVE_DEMO		/* Define RECEIVE demo */
//#define SEND_DEMO			/* Define SEND demo */

/* Define Required fields shown below */
#define DOMAIN				"api.thingspeak.com"
#define PORT_FOR_GET_API				"80"
#define API_WRITE_KEY		"C7JFHZY54GLCJY38"
#define CHANNEL_ID			"119922"

#define SSID				"choi"
#define PASSWORD			"s1234567890"

# define JOYSTICK_DATA_DEFAULT_LEN  7

enum ESP8266_RESPONSE_STATUS{
	ESP8266_RESPONSE_WAITING,
	ESP8266_RESPONSE_FINISHED,
	ESP8266_RESPONSE_TIMEOUT,
	ESP8266_RESPONSE_BUFFER_FULL,
	ESP8266_RESPONSE_STARTING,
	ESP8266_RESPONSE_ERROR
};

enum ESP8266_CONNECT_STATUS {
	ESP8266_CONNECTED_TO_AP,
	ESP8266_CREATED_TRANSMISSION,
	ESP8266_TRANSMISSION_DISCONNECTED,
	ESP8266_NOT_CONNECTED_TO_AP,
	ESP8266_CONNECT_UNKNOWN_ERROR
};

enum ESP8266_JOINAP_STATUS {
	ESP8266_WIFI_CONNECTED,
	ESP8266_CONNECTION_TIMEOUT,
	ESP8266_WRONG_PASSWORD,
	ESP8266_NOT_FOUND_TARGET_AP,
	ESP8266_CONNECTION_FAILED,
	ESP8266_JOIN_UNKNOWN_ERROR
};

char RESPONSE_BUFFER[DEFAULT_BUFFER_SIZE];

void Read_Response(char* _Expected_Response);

void ESP8266_Clear();

void Start_Read_Response(char* _ExpectedResponse);

void GetResponseBody(char* Response, uint16_t ResponseLength);

bool WaitForExpectedResponse(char* ExpectedResponse);

bool SendATandExpectResponse(char* ATCommand, char* ExpectedResponse);

bool ESP8266_ApplicationMode(uint8_t Mode);

bool ESP8266_ConnectionMode(uint8_t Mode);

bool ESP8266_Begin();

bool ESP8266_Close();

bool ESP8266_WIFIMode(uint8_t _mode);

uint8_t ESP8266_JoinAccessPoint(char* _SSID, char* _PASSWORD);

uint8_t ESP8266_connected();

uint8_t ESP8266_Start(uint8_t _ConnectionNumber, char* Domain, char* Port);

uint8_t ESP8266_Send(char* Data);

int16_t ESP8266_DataAvailable();

uint8_t ESP8266_DataRead();

uint16_t Read_Data(char* _buffer);

void ESP8266_ISR_Process();

void ESP8266_Print_AT_Command_Response(char* ATCommand);

bool SendATCommandAndPrintResponse(char* ATCommand, char* ExpectedResponse);

#endif