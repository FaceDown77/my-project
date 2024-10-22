#include "/home/jwpark/miniProject/rcController/periph/adc/adc.h"

void ADC0_Init() {
    // ADMUX 설정 (참조 전압 및 입력 채널 선택)
    // ref datasheet 243p
    // 0100 0000 : (REFS1:0) = 01, AVCC with external capacitor at AREF pin
    // (MUX4:0) = 00000, Analog Channel 0 (ADC0)

    //ADMUX = 0b01000000
    
    // 참조 전압을 AVCC로 설정
    ADMUX |= (1<<REFS0);
    // 루프문을 수행하면서 비트셋을 지속적으로 초기화 해주어야 하므로, 사용하지 않는 채널 비트는 모두 0으로 리셋해 주어야 함
    ADMUX &= ~((1<<MUX4) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0));
    // REFS1 비트 클리어
    //ADMUX &= ~(1 << REFS1);
    // ADC0 선택
    //ADMUX &= ~(1 << MUX4) & ~(1 << MUX3) & ~(1 << MUX2) & ~(1 << MUX1) & ~(1 << MUX0);
    
    // ADCSRA 설정 (ADC 활성화 및 프리스케일러 설정)
    // ref datasheet 244p
    // 1001 1000 : (ADEN) = 1, ADC Enable
    // (ADSC) = 0, Conversion Start 비활성화
    // (ADATE) = 0, Auto Trigger Disable
    // (ADIF) = 0, Interrupt Flag 클리어
    // (ADIE) = 1, Interrupt Enable
    // 프리스케일러 = 8 (000) => (ADPS2:0) = 001

    //ADCSRA = 0b10000111

    // ADC 활성화
    ADCSRA |= (1<<ADEN);
    // ADC 인터럽트 활성화
    //ADCSRA |= (1<<ADIE);
    //ADCSRA |= (1<<ADIF);
    // Auto Trigger 비활성화
    //ADCSRA &= ~(1 << ADATE);
    // Start Conversion 비활성화 (필요할 때 설정)
    //ADCSRA &= ~(1 << ADSC);
    // 프리스케일러 설정
    // 프리스케일러 = 128
    // ref 248p
    ADCSRA |= ((1<<ADPS2) |(1<<ADPS1) |(1<<ADPS0));
}

void ADC1_Init() {
    ADMUX |= ((1<<REFS0) | (1<<MUX0));
    ADMUX &= ~((1<<MUX4) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1));

    ADCSRA |= (1<<ADEN);
    //ADCSRA |= (1<<ADIE);
    //ADCSRA |= (1<<ADIF);
    ADCSRA |= ((1<<ADPS2) | (1<<ADPS1) |(1<<ADPS0));
}

unsigned short ADC_Read() {
    unsigned char adc_low, adc_high;
    unsigned short value;

    // ADCSRA 레지스터 설정
    // ADSC 비트 (ADCSRA[6]) = 1: Start Conversion (ADC 변환 시작)
    // ADC 변환 시작
    //ADCSRA |= 0X40;
    ADCSRA |= (1<<ADSC);

    // ADC 완료할 때까지 변환 작업 실행
    // ADIF 비트 (ADCSRA[4])가 1이 될 때까지 대기 (변환 완료 플래그)
    // 변환 완료될 때까지 대기
    //while((ADCSRA & 0x10) != 0x10);
    //while ((ADCSRA & (1<<ADSC)) != (1<<ADSC));
    //while((ADCSRA & (1<<ADIF)) != (1<<ADIF));
    while(ADCSRA & (1<<ADSC));

    // 변환이 완료되었으므로 ADIF 플래그를 수동으로 클리어
    // ADIF는 쓰기로 클리어할 수 있음 (1로 설정하여 클리어)
    // ADIF 플래그 클리어
    //ADCSRA |= (1<<ADIF);

    // ADCL과 ADCH에서 변환 결과 읽기
    // 하위 바이트 읽기 (ADCL)
    adc_low = ADCL;
    // 상위 바이트 읽기 (ADCH)
    adc_high = ADCH;

    // 상위 바이트와 하위 바이트를 합쳐서 10비트 값으로 변환
    // 상위 바이트를 왼쪽으로 8비트 이동 후 하위 바이트와 결합
    value = ((adc_high<<8) | adc_low);

    return value;
}