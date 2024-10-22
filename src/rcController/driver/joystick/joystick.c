#include "/home/jwpark/miniProject/rcController/driver/joystick/joystick.h"

JoystickData joystickData;
char buffer[BUF_SIZE];
int mappedData;

void On_LED_For_Debuggig(unsigned short data) {
    unsigned short ADCVALUE = data;
    //PORTD |= (1<<PD0);
    //PORTD = 0x00;

    if(ADCVALUE > 1000)
        PORTG = (1<<0);
    else if(ADCVALUE < 100)
        PORTG = (1<<0);
    else
       PORTG = 0x00;
}

// helper function
int Joystick_Value_Map(unsigned short curValue, unsigned short asIsMinValue, unsigned short asIsMaxValue, int toBeMinValue, int toBeMaxValue) {
    float mappedValue = (float)(curValue - asIsMinValue) * (toBeMaxValue - toBeMinValue) / (asIsMaxValue - asIsMinValue) + toBeMinValue;
    return (int)mappedValue;
}

void Operate_X_Axis_Joystick() {
    ADC0_Init();
    joystickData.xPos = ADC_Read();

    On_LED_For_Debuggig(joystickData.xPos);
}

void Operate_Y_Axis_Joystick() {
    ADC1_Init();
    joystickData.yPos = ADC_Read();

    On_LED_For_Debuggig(joystickData.yPos);
}

unsigned short Get_ADC_X_Axis_Data_From_Joystick() {
    return joystickData.xPos;
}

unsigned short Get_ADC_Y_Axis_Data_From_Joystick() {
    return joystickData.yPos;
}

char* Get_ADC_Data_As_String_From_Joystick() {
    sprintf(buffer, "X: %hu, Y: %hu\r\n", Get_ADC_X_Axis_Data_From_Joystick(), Get_ADC_Y_Axis_Data_From_Joystick());

    return buffer;
}

// For transmit to UART0


void Final_Transmit_Processed_ADC_Data_To_UART0(int cnvtData) {
    switch(cnvtData) {
        case 0:
            UART0_Transmit('0');
            break;
        case 1:
            UART0_Transmit('1');
            break;
        case 2:
            UART0_Transmit('2');
            break;
        case 3:
            UART0_Transmit('3');
            break;
        case 4:
            UART0_Transmit('4');
            break;
        case 5:
            UART0_Transmit('5');
            break;
        case 6:
            UART0_Transmit('6');
            break;
        case 7:
            UART0_Transmit('7');
            break;
        case 8:
            UART0_Transmit('8');
            break; 
        case 9:
            UART0_Transmit('9');
            break; 
        case -1:
            UART0_Transmit('-');
            UART0_Transmit('1');
            break;
        case -2:
            UART0_Transmit('-');
            UART0_Transmit('2');
            break;
        case -3:
            UART0_Transmit('-');
            UART0_Transmit('3');
            break;
        case -4:
            UART0_Transmit('-');
            UART0_Transmit('4');
            break;
        case -5:
            UART0_Transmit('-');
            UART0_Transmit('5');
            break;
        case -6:
            UART0_Transmit('-');
            UART0_Transmit('6');
            break;
        case -7:
            UART0_Transmit('-');
            UART0_Transmit('7');
            break;
        case -8:
            UART0_Transmit('-');
            UART0_Transmit('8');
            break; 
        case -9:
            UART0_Transmit('-');
            UART0_Transmit('9');
            break; 
    }
}

// helper function
void Process_ADC_Data_For_Transmit_To_UART0(int cnvtData) {
    int thousandDigit, hundredDigit, tenDigit, oneDigit;

    // 음수일 경우 부호를 먼저 출력하고, 절대값으로 변환
    if(cnvtData < 0) {
        UART0_Transmit('-');
        cnvtData = -cnvtData;  // 절대값으로 변환
    }

    if(cnvtData > 1000) {
        thousandDigit = cnvtData/1000;
        cnvtData %= 1000;

        hundredDigit = cnvtData/100;
        cnvtData %= 100;

        tenDigit = cnvtData/10;

        oneDigit = cnvtData%10;
        
        Final_Transmit_Processed_ADC_Data_To_UART0(thousandDigit);
        Final_Transmit_Processed_ADC_Data_To_UART0(hundredDigit);
        Final_Transmit_Processed_ADC_Data_To_UART0(tenDigit);
        Final_Transmit_Processed_ADC_Data_To_UART0(oneDigit);
    } else if(cnvtData > 100) {
        hundredDigit = cnvtData/100;
        cnvtData %= 100;

        tenDigit = cnvtData/10;

        oneDigit = cnvtData%10;

        Final_Transmit_Processed_ADC_Data_To_UART0(hundredDigit);
        Final_Transmit_Processed_ADC_Data_To_UART0(tenDigit);
        Final_Transmit_Processed_ADC_Data_To_UART0(oneDigit);        
    } else if(cnvtData > 10) {
        tenDigit = cnvtData/10;

        oneDigit = cnvtData%10;

        Final_Transmit_Processed_ADC_Data_To_UART0(tenDigit);
        Final_Transmit_Processed_ADC_Data_To_UART0(oneDigit);         
    } else 
        Final_Transmit_Processed_ADC_Data_To_UART0(cnvtData);
}

void Transmit_Joystick_X_Axis_Data_To_UART0() {
    //unsigned short adcData = Get_ADC_X_Axis_Data_From_Joystick();
    int mappedData = Joystick_Value_Map(Get_ADC_X_Axis_Data_From_Joystick(), 0, 1024, -100, 100);
    Process_ADC_Data_For_Transmit_To_UART0(mappedData);
}

void Transmit_Joystick_Y_Axis_Data_To_UART0() {
    //unsigned short adcData = Get_ADC_Y_Axis_Data_From_Joystick();
    int mappedData = Joystick_Value_Map(Get_ADC_Y_Axis_Data_From_Joystick(), 0, 1024, -100, 100);
    Process_ADC_Data_For_Transmit_To_UART0(mappedData);
}


// For transmit to UART1


void Final_Transmit_Processed_ADC_Data_To_UART1(int cnvtData) {
    switch(cnvtData) {
        case 0:
            UART1_Transmit('0');
            break;
        case 1:
            UART1_Transmit('1');
            break;
        case 2:
            UART1_Transmit('2');
            break;
        case 3:
            UART1_Transmit('3');
            break;
        case 4:
            UART1_Transmit('4');
            break;
        case 5:
            UART1_Transmit('5');
            break;
        case 6:
            UART1_Transmit('6');
            break;
        case 7:
            UART1_Transmit('7');
            break;
        case 8:
            UART1_Transmit('8');
            break; 
        case 9:
            UART1_Transmit('9');
            break; 
        case -1:
            UART1_Transmit('-');
            UART1_Transmit('1');
            break;
        case -2:
            UART1_Transmit('-');
            UART1_Transmit('2');
            break;
        case -3:
            UART1_Transmit('-');
            UART1_Transmit('3');
            break;
        case -4:
            UART1_Transmit('-');
            UART1_Transmit('4');
            break;
        case -5:
            UART1_Transmit('-');
            UART1_Transmit('5');
            break;
        case -6:
            UART1_Transmit('-');
            UART1_Transmit('6');
            break;
        case -7:
            UART1_Transmit('-');
            UART1_Transmit('7');
            break;
        case -8:
            UART1_Transmit('-');
            UART1_Transmit('8');
            break; 
        case -9:
            UART1_Transmit('-');
            UART1_Transmit('9');
            break; 
    }
}

// helper function
void Process_ADC_Data_For_Transmit_To_UART1(int cnvtData) {
    int thousandDigit, hundredDigit, tenDigit, oneDigit;

    // 음수일 경우 부호를 먼저 출력하고, 절대값으로 변환
    if(cnvtData < 0) {
        UART1_Transmit('-');
        cnvtData = -cnvtData;  // 절대값으로 변환
    }

    if(cnvtData > 1000) {
        thousandDigit = cnvtData/1000;
        cnvtData %= 1000;

        hundredDigit = cnvtData/100;
        cnvtData %= 100;

        tenDigit = cnvtData/10;

        oneDigit = cnvtData%10;
        
        Final_Transmit_Processed_ADC_Data_To_UART1(thousandDigit);
        Final_Transmit_Processed_ADC_Data_To_UART1(hundredDigit);
        Final_Transmit_Processed_ADC_Data_To_UART1(tenDigit);
        Final_Transmit_Processed_ADC_Data_To_UART1(oneDigit);
    } else if(cnvtData > 100) {
        hundredDigit = cnvtData/100;
        cnvtData %= 100;

        tenDigit = cnvtData/10;

        oneDigit = cnvtData%10;

        Final_Transmit_Processed_ADC_Data_To_UART1(hundredDigit);
        Final_Transmit_Processed_ADC_Data_To_UART1(tenDigit);
        Final_Transmit_Processed_ADC_Data_To_UART1(oneDigit);        
    } else if(cnvtData > 10) {
        tenDigit = cnvtData/10;

        oneDigit = cnvtData%10;

        Final_Transmit_Processed_ADC_Data_To_UART1(tenDigit);
        Final_Transmit_Processed_ADC_Data_To_UART1(oneDigit);         
    }  else 
        Final_Transmit_Processed_ADC_Data_To_UART1(cnvtData);
}

uint8_t Caculate_Joystick_X_Axis_Data_Len(uint8_t joystickDataLen) {
    mappedData = Joystick_Value_Map(Get_ADC_X_Axis_Data_From_Joystick(), 0, 1024, -100, 100);

    // 전송할 문자열로 변환
    char buffer[BUF_SIZE]; // 충분한 크기의 버퍼
    memset(buffer, 0, sizeof(buffer));
    sprintf(buffer, "%d", mappedData);
    joystickDataLen += strlen(buffer); // 문자열 길이를 추가

    return joystickDataLen;
}

uint8_t Caculate_Joystick_Y_Axis_Data_Len(uint8_t joystickDataLen) {
    mappedData = Joystick_Value_Map(Get_ADC_X_Axis_Data_From_Joystick(), 0, 1024, -100, 100);
    
    // 전송할 문자열로 변환
    char buffer[BUF_SIZE]; // 충분한 크기의 버퍼
    memset(buffer, 0, sizeof(buffer));
    sprintf(buffer, "%d", mappedData);
    joystickDataLen += strlen(buffer); // 문자열 길이를 추가

    return joystickDataLen;
}

void Transmit_Joystick_X_Axis_Data_To_UART1() {
    //unsigned short adcData = Get_ADC_X_Axis_Data_From_Joystick();
    int mappedData = Joystick_Value_Map(Get_ADC_X_Axis_Data_From_Joystick(), 0, 1024, -100, 100);
    Process_ADC_Data_For_Transmit_To_UART1(mappedData);
}

void Transmit_Joystick_Y_Axis_Data_To_UART1() {
    //unsigned short adcData = Get_ADC_Y_Axis_Data_From_Joystick();
    int mappedData = Joystick_Value_Map(Get_ADC_Y_Axis_Data_From_Joystick(), 0, 1024, -100, 100);
    Process_ADC_Data_For_Transmit_To_UART1(mappedData);
}