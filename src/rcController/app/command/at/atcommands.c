#include "/home/jwpark/miniProject/rcController/app/command/at/atcommands.h"

int newline;
uint32_t timer;
// 수신 데이터 버퍼
char replybuffer[128];
char* replyString;

void ISR_AT_Command_Read_Response_Process(const char *expectedReply, uint16_t timeout) {
    uint8_t idx = 0;
    int reply_match = FALSE;
    timer = 0;

    while (!reply_match && timer < timeout) {
        if (UART1_RX_Avail()) {
            if(UART1_getRxFlag()) {
                UART1_clearRxFlag();
                replyString = UART1_readRxBuff();

                replybuffer[idx] = UART1_Receive();
                idx++;
                replybuffer[idx] = '\0';
            }

            if (!strcmp(replybuffer, expectedReply)) {
                reply_match = TRUE;
                break;
            }
        }
        _delay_ms(1);  // 1ms 지연
        timer++;
    }

}

// AT 명령어 전송 (응답 확인 없이)
void AT_Send_Blind_Command(char *command) {
    //UART1_sendString("\t---> ");
    UART1_sendString(command);
    //UART1_sendString("\r\n");

    //return TRUE;
}

// AT 명령어 전송 후 응답 확인 (타임아웃 있음)
int AT_Send_Command(char *command, uint16_t timeout) {
    // UART1 입력 버퍼 비우기
    UART1_Clear();

    UART1_sendString("\t---> ");
    UART1_sendString(command);
    if (newline) {
        UART1_sendString("\r\n");
    } else {
        UART1_sendString("\r");
    }

    int got_reply = FALSE;
    timer = 0;

    while (!got_reply && timer < timeout) {
        if (UART1_RX_Avail()) {
            got_reply = TRUE;
        }
        _delay_ms(1);
        timer++;
    }

    return got_reply;
}

// AT 명령어 전송 후 특정 응답 대기
int AT_Send_Command_And_Receive_Reply(char *command, const char *expectedReply, uint16_t timeout) {
    // UART 입력 버퍼 비우기
    //UART1_Clear();

    UART0_sendString(command);
    UART1_sendString(command);

    uint8_t idx = 0;
    int reply_match = FALSE;
    timer = 0;

    while (!reply_match && timer < timeout) {

        if(UART1_RX_Avail()) {
            if(UART1_getRxFlag()) {
                UART1_clearRxFlag();
                if (idx < BUF_SIZE - 1) { // 버퍼 크기 체크
                    replybuffer[idx] = UART1_Receive();
                    idx++;
                    replybuffer[idx] = '\0'; // 널 종료 추가

                    // 개행 후 예상 응답과 비교
                    if (strstr(replybuffer, expectedReply) != NULL) {
                        reply_match = TRUE;
                    }
                }  else printf("Buffer is overflow!");
            }
        }

        
        _delay_ms(1);
        timer++;
    }

    UART0_sendString("\t<--- ");
    //UART0_sendString(replyString);
    //UART0_sendString(replybuffer);
    UART0_sendString("test");
    UART0_sendString("\r\n");

    return reply_match;
}