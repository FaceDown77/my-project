#ifndef JOYSTICK_H_
#define JOYSTICK_H__

# define F_CPU	16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#include "/home/jwpark/miniProject/rcController/periph/adc/adc.h"
#include "/home/jwpark/miniProject/rcController/periph/uart0/uart0.h"
#include "/home/jwpark/miniProject/rcController/periph/uart1/uart1.h"

# define BUF_SIZE 128

typedef struct _joystick_adc_data {
	unsigned short xPos;
	unsigned short yPos;
}JoystickData;

void On_LED_For_Debuggig(unsigned short data);
int Joystick_Value_Map(unsigned short curValue, unsigned short asIsMinValue, unsigned short asIsMaxValue, int toBeMinValue, int toBeMaxValue);

void Operate_X_Axis_Joystick();
void Operate_Y_Axis_Joystick();

unsigned short Get_ADC_X_Axis_Data_From_Joystick();
unsigned short Get_ADC_Y_Axis_Data_From_Joystick();
char* Get_ADC_Data_As_String_From_Joystick();

void Final_Transmit_Processed_ADC_Data_To_UART0(int cnvtData);
void Process_ADC_Data_For_Transmit_To_UART0(int cnvtData);

void Transmit_Joystick_X_Axis_Data_To_UART0();
void Transmit_Joystick_Y_Axis_Data_To_UART0();

void Final_Transmit_Processed_ADC_Data_To_UART1(int cnvtData);
void Process_ADC_Data_For_Transmit_To_UART1(int cnvtData);

uint8_t Caculate_Joystick_X_Axis_Data_Len(uint8_t joystickDataLen);
uint8_t Caculate_Joystick_Y_Axis_Data_Len(uint8_t joystickDataLen);

void Transmit_Joystick_X_Axis_Data_To_UART1();
void Transmit_Joystick_Y_Axis_Data_To_UART1();

#endif