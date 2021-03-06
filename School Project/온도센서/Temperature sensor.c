//////////////////////////////////////////////////////////////////////
//HW 3.1 외부 온도, 내부 온도 측정 HW
//제출자 2015130035 장동현
//TUE 02
// 블루투스 연결 BT SCAN 전까지의 자동
//BT NAME = TUE 02
//SW1 누를 시 자동 설정(AT부터 끝까지 BTSCAN전까지)
//SW2 누를 시 BTSCAN
//sw3 누를 시 기본 수업 시간 블루투스 모드 0부터 sw 6까지 동일하게 동작
//////////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"
#include "default.h"
#include "Util.h"
#include "Que.h"

#define SW0_PUSH        0xFE00  //PH8
#define SW1_PUSH        0xFD00  //PH9
#define SW2_PUSH        0xFB00  //PH10
#define SW3_PUSH        0xF700  //PH11
#define SW4_PUSH        0xEF00  //PH12
#define SW5_PUSH        0xDF00  //PH13
#define SW6_PUSH        0xBF00  //PH14
#define SW7_PUSH        0x7F00  //PH15
#define RGB_PINK GET_RGB(255,169,214)
void DispayTitle(void);
void _GPIO_Init(void);
void _ADC_Init(void);
uint16_t KEY_Scan(void);
void TIMER8_Init(void);

void USART1_Init(void);
void UART4_Init(void);
void USART1_BRR_Configuration(uint32_t USART_BaudRate);
void UART4_BRR_Configuration(uint32_t USART_BaudRate);

void SerialSendChar_PC(uint8_t c);
void SerialSendString_PC(char* s);
void SerialSendChar_BT(uint8_t c);
void SerialSendString_BT(char* s);

int IUART_GetSize(UINT8 id);
void IUART_GetData(UINT8 id, UINT8* ch1);
void IUART_SendData(UINT8 id, UINT8 ch1);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);
void Dis(void);
void Connect(void);
UINT8 strBuffer[2][100];
UINT8 strBufferIdx[2];
UINT8 size;
UINT8 data;
UINT8 ch;
UINT8 no_PC, no_BT, CRdata_PC, CRdata_BT;
uint16_t ADC_Value, Voltage;
int val;                      //외부온도 변수
int vol;                      //외부온도 변수            
int on;                      //외부온도 변수
int sel = 0;                //bluetooth 동작 모드 0자동 1기존
double check;                      //외부온도 변수
int Temp;                       //내부온도 변수
int barTemp = 0;                //내부온도 변수
uint8_t str[20];                //통신용 변수
int start = 1;                  //내부, 외부 판단 변수
uint16_t ADC_Value2, Voltage2;  //외부온도 변수
uint16_t Temp2;                 //외부온도 전압 변수
void ADC_IRQHandler(void)
{
    if (ADC3->SR)
    {
        ADC3->SR &= ~(1 << 1);      // EOC flag clear
        if (start == 1)             // BT 내부 온도 센서
        {
            ADC_Value = ADC3->DR;      // Reading ADC result
            Voltage = ADC_Value * (3.3 * 100) / 4095;   // 3.3 : 4095 =  Volatge : ADC_Value 
            // 1 : 4095
            Temp = ((Voltage * Voltage * 3.5 / 10000) + 1) * 10;            // 온도 값 계산
            LCD_SetBrushColor(RGB_YELLOW);
            LCD_SetTextColor(RGB_RED);      //글자색 : 빨강
            LCD_DisplayChar(1, 18, Voltage / 100 + 0x30);
            LCD_DisplayChar(1, 20, Voltage % 100 / 10 + 0x30);
            LCD_DisplayChar(1, 21, Voltage % 10 + 0x30);
            LCD_SetTextColor(RGB_RED);      //글자색 : 빨강
            LCD_DisplayChar(1, 11, Temp / 100 + 0x30);
            LCD_DisplayChar(1, 12, Temp % 100 / 10 + 0x30);
            LCD_DisplayChar(1, 14, Temp % 10 + 0x30);
            barTemp = Temp / 10;
            if (barTemp == 39)           // 바가 꽉찬경우
            {
                LCD_SetBrushColor(RGB_GREEN);
                LCD_DrawFillRect(11, 29, 138, 9); // 138 - 11 = 127
            }
            else
            {
                val = (4095 - on) / 4095 * 137; // 볼티지 오른쪽 나머지 왼쪽
                vol = barTemp * 137 / 39;       // 전압 값 39등분
                LCD_SetBrushColor(RGB_YELLOW);
                LCD_DrawFillRect((11 + vol), 29, 137, 9); // 138 - 11 = 127
                LCD_SetBrushColor(RGB_GREEN);
                LCD_DrawFillRect(11, 29, vol, 9); // 138 - 11 = 127
                LCD_SetPenColor(RGB_GREEN);      //펜색 : 초록
                LCD_DrawRectangle(10, 28, 139, 9);
            }

        }
    }
    if (ADC1->SR)
    {
        ADC1->SR &= ~(1 << 1);      // EOC flag clear
        if (start == 2)             // BT 내부 온도 센서
        {
            ADC_Value2 = ADC1->DR;                              //측정값 받아오기
            Voltage2 = ADC_Value2 * 330 / 4095;                 //전압값 계산
            Temp2 = ((Voltage2 - 76) * 4 + 25) * 10;            // 계산식 적용
            LCD_SetTextColor(RGB_RED);      //글자색 : 빨강
            LCD_DisplayChar(1, 11, Temp2 / 100 + 0x30);          //내부온도 10의자리
            LCD_DisplayChar(1, 12, Temp2 % 100 / 10 + 0x30);     //내부온도 1의자리
            LCD_DisplayChar(1, 14, Temp2 % 10 + 0x30);         //내부온도 0.1의자리
            LCD_SetBrushColor(RGB_YELLOW);                     //브러쉬색 : 노랑
            LCD_DrawFillRect(11, 27, 137, 9);                  //사각형 채우기
            LCD_SetBrushColor(RGB_PINK);                       //브러쉬색 : 핑크 
            LCD_DrawRectangle(10, 27, 138, 10);                //사각형 그려주기
            LCD_DrawFillRect(11, 27, Temp2 / 10 * 2, 9);       //사각형 채우기
        }
    }
}
int main(void)
{
    LCD_Init();   // LCD 구동 함수
    DelayMS(100);   // LCD구동 딜레이
    _GPIO_Init();
    TIMER8_Init();
    _ADC_Init();
    USART1_Init();
    UART4_Init();
    Que_Clear(&txQue[1]);   // PC(USART1) QUEUE clear
    Que_Clear(&rxQue[1]);
    Que_Clear(&txQue[2]);   // BT(UART4) QUEUE clear
    Que_Clear(&rxQue[2]);
    GPIOG->ODR &= 0x00;   // LED0~7 Off 
    DispayTitle();   //LCD 초기화면구동 함수
    //Connect();       //블루투스 연결 함수
    LCD_SetBackColor(RGB_WHITE);
    LCD_DisplayText(6, 0, "BT MODE AUTO");              //블루투스 설명
    LCD_DisplayText(7, 0, "BT NAME TUE 02");
    LCD_DisplayText(8, 0, "SW1 : Auto Set");
    LCD_DisplayText(9, 0, "SW2 : BT SCAN");
    LCD_DisplayText(10, 0, "SW3 : USE Default BT");            //스위치 3 누를 시 수업 시간 블루투스 예제로 동작 가능
    LCD_SetBackColor(RGB_YELLOW);
    strBufferIdx[0] = 0;
    strBufferIdx[1] = 0;

    start = 1;  //외부 온도 모드
    sel = 1;    //자동 bt 모드
    int btcnt = 0;
    while (1)
    {

        if (start == 2)             //내부 온도 모드 일시
        {
            ADC1->CR2 |= (1 << 30);
            DelayMS(300);           //내부 온도 센서 값 받아오는 속도 너무 빨라 딜레이 적용
        }
        if (sel == 0)
        {
            switch (KEY_Scan())
            {
            case SW1_PUSH:       //SW0
                GPIOG->ODR ^= 0x02;   // LED0 On
                SerialSendString_BT("AT+BTCANCEL"); // BT module 연결 해제
                SerialSendChar_BT(0x0D); // CR(Carriage Return : 0x0D)
                SerialSendString_PC("AT+BTCANCEL"); // BT module 연결 해제
                ADC1->CR2 |= (1 << 30);
                break;
            case SW2_PUSH:       //SW0
                GPIOG->ODR ^= 0x04;// LED1 On
                SerialSendString_BT("AT+BTSCAN");
                SerialSendChar_BT(0x0D); // USART1을 통해 PC로 “HELLO! ” 문자열을 전송
                break;
            case SW3_PUSH:      //블루 투스 자동 모드
                GPIOG->ODR ^= 0x08;// LED1 On
                Connect();
                break;
            case SW4_PUSH:      //블루 투스 자동 모드
                GPIOG->ODR ^= 0x10;// LED1 On
                sel = 1;
                break;
            }
        }
        else if (sel == 1)
        {
            switch (KEY_Scan())
            {
            case SW0_PUSH: 		//SW0
                GPIOG->ODR |= 0x01;	// LED0 On
                SerialSendString_BT("AT+BTCANCEL"); // BT module 연결 해제
                SerialSendChar_BT(0x0D); // CR(Carriage Return : 0x0D)
                SerialSendString_PC("AT+BTCANCEL"); // BT module 연결 해제
                break;
            case SW1_PUSH: 		//SW1
                GPIOG->ODR |= 0x02;	// LED1 On
                SerialSendString_BT("AT"); // BT module 연결 확인
                SerialSendChar_BT(0x0D); // CR(Carriage Return : 0x0D)
                SerialSendString_PC("AT"); // BT module 연결 해제
                break;
            case SW2_PUSH: 		//SW2
                GPIOG->ODR |= 0x04;	// LED2 On
                SerialSendString_BT("AT+BTSCAN"); // BT module 을 외부 BT 기기(Master)에서 찾게 하기 위한 명령
                SerialSendChar_BT(0x0D); // CR(Carriage Return : 0x0D)
                SerialSendString_PC("AT+BTSCAN"); // BT module 연결 해제
                break;
            case SW3_PUSH: 		//SW3
                GPIOG->ODR |= 0x08;	// LED3 On
                SerialSendString_BT("AT+BTNAME=TUE02"); // BT module 이름 설정(변경)
                SerialSendChar_BT(0x0D); // CR(Carriage Return : 0x0D)
                SerialSendString_PC("AT+BTNAME=TUE02"); // BT module 연결 해제
                break;
            case SW4_PUSH: 		//SW4
                GPIOG->ODR |= 0x10;	// LED4 On
                SerialSendString_BT("ATZ"); // BT 명령후에 명령 인식을 위한 소프트웨어 리셋
                SerialSendChar_BT(0x0D);  // CR(Carriage Return : 0x0D)
                SerialSendString_PC("ATZ"); // BT module 연결 해제
                break;
            case SW5_PUSH: 		//SW5
                GPIOG->ODR |= 0x20;	// LED5 On
                SerialSendString_BT("AT+BTINFO?0"); // BT module 이름 확인
                SerialSendChar_BT(0x0D); // CR(Carriage Return : 0x0D)
                SerialSendString_PC("AT+BTINFO?0"); // BT module 연결 해제
                break;
            case SW6_PUSH: 		//SW6
                GPIOG->ODR |= 0x40;	// LED6 On
                SerialSendChar_BT('B'); // 일반 데이터 전송 
                SerialSendChar_PC('B'); // PC 통신 확인을 위한 문자 전송
                break;
            case SW7_PUSH: 		//SW7
                GPIOG->ODR |= 0x80;	// LED7 On
                SerialSendChar_PC('P'); // PC 통신 확인을 위한 문자 전송
                break;
            }
        }
    }
}
void _ADC_Init(void)
{      // ADC1: PA3(pin 47)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // (1<<0) ENABLE GPIOA CLK (stm32f4xx.h 참조)
    GPIOA->MODER |= (3 << 2 * 1);      // CONFIG GPIOA PIN3(PA3) TO ANALOG IN MODE

    RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;   // (1<<8) ENABLE ADC3 CLK (stm32f4xx.h 참조)

    ADC->CCR &= ~(0X1F << 0);      // MULTI[4:0]: ADC_Mode_Independent
    ADC->CCR |= (1 << 16);       // 0x00010000 ADCPRE:ADC_Prescaler_Div4 (ADC MAX Clock 36MHz, 84Mhz(APB2)/4 = 21MHz)

    ADC3->CR1 &= ~(3 << 24);      // RES[1:0]=0b00 : 12bit Resolution
    ADC3->CR1 &= ~(1 << 8);      // SCAN=0 : ADC_ScanCovMode Disable
    ADC3->CR1 |= (1 << 5);      // EOCIE=1: Interrupt enable for EOC

    ADC3->CR2 &= ~(1 << 1);      // CONT=0: ADC_Continuous ConvMode Disable
    ADC3->CR2 |= (2 << 28);      // EXTEN[1:0]: ADC_ExternalTrigConvEdge_Enable(Falling Edge)
    ADC3->CR2 |= (0x0D << 24);   // EXTSEL[3:0]: ADC_ExternalTrig (TIM8_CC1)
    ADC3->CR2 &= ~(1 << 11);      // ALIGN=0: ADC_DataAlign_Right
    ADC3->CR2 &= ~(1 << 10);      // EOCS=0: The EOC bit is set at the end of each sequence of regular conversions

    ADC3->SQR1 &= ~(0xF << 20);   // L[3:0]=0b0000: ADC Regular channel sequece length 
             // 0b0000:1 conversion)
     //Channel selection, The Conversion Sequence of PIN1(ADC1_CH1) is first, Config sequence Range is possible from 0 to 17
    ADC3->SQR3 |= (1 << 0);      // SQ1[4:0]=0b0001 : CH1
    ADC3->SMPR2 |= (0x7 << (3 * 3));   // ADC3_CH1 Sample TIme_480Cycles (3*Channel_1)
 //Channel selection, The Conversion Sequence of PIN1(ADC1_CH1) is first, Config sequence Range is possible from 0 to 17

    NVIC->ISER[0] |= (1 << 18);   // Enable ADC global Interrupt





    //RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // (1<<0) ENABLE GPIOA CLK (stm32f4xx.h 참조)
   //GPIOA->MODER |= (3<<2*1);      // CONFIG GPIOA PIN3(PA3) TO ANALOG IN MODE

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;	// (1<<8) ENABLE ADC3 CLK (stm32f4xx.h 참조)

    ADC->CCR &= ~(0X1F << 0);		// MULTI[4:0]: ADC_Mode_Independent
    ADC->CCR |= (1 << 16); 		// 0x00010000 ADCPRE:ADC_Prescaler_Div4 (ADC MAX Clock 36MHz, 84Mhz(APB2)/4 = 21MHz)
    ADC->CCR |= (1 << 23);

    ADC1->CR1 &= ~(3 << 24);		// RES[1:0]= 0b00 : 12bit Resolution
    ADC1->CR1 &= ~(1 << 8);		// SCAN=0 : ADC_ScanCovMode Disable
    ADC1->CR1 |= (1 << 5);		// EOCIE=1: Interrupt enable for EOC

    ADC1->CR2 &= ~(1 << 1);		// CONT=0: ADC_Continuous ConvMode Disable
    ADC1->CR2 |= (2 << 28);		// EXTEN[1:0]: ADC_ExternalTrigConvEdge_Enable(Falling Edge)
    ADC1->CR2 &= ~(1 << 11);		// ALIGN=0: ADC_DataAlign_Right
    ADC1->CR2 &= ~(1 << 10);		// EOCS=0: The EOC bit is set at the end of each sequence of regular conversions

    ADC1->SQR1 &= ~(0xF << 20);	// L[3:0]=0b0000: ADC Regular channel sequece length 
                    // 0b0000:1 conversion)
    //Channel selection, The Conversion Sequence of PIN1(ADC3_CH1) is first, Config sequence Range is possible from 0 to 17
    ADC1->SQR3 |= (16 << 0);		// SQ1[4:0]=0b10000 : CH6
    ADC1->SMPR2 |= (0x7 << (3 * 1));	// ADC3_CH1 Sample TIme_480Cycles (3*Channel_1)
//Channel selection, The Conversion Sequence of PIN1(ADC3_CH1) is first, Config sequence Range is possible from 0 to 17

    ADC1->CR2 |= (1 << 0);		// ADON: ADC ON
    ADC3->CR2 |= (1 << 0);		// ADON: ADC ON

}
void TIMER8_Init(void)
{
    // TIM8_CH1 PI5 
    // 인터럽트 주기 : 200ms
    // Clock Enable : GPIOI & TIMER8
    RCC->AHB1ENR |= (1 << 8);   // GPIOI Enable
    RCC->APB2ENR |= (1 << 1);   // TIMER8 Enable 

  // PA2을 출력설정하고 Alternate function(TIM8_CH1)으로 사용 선언 
    GPIOI->MODER |= (2 << 2 * 5);   // PI5 Output Alternate function mode               
    GPIOI->OSPEEDR |= (3 << 2 * 5);   // PI5 Output speed (100MHz High speed)
    GPIOI->OTYPER &= ~(1 << 5);   // PI5 Output type push-pull (reset state)
    GPIOI->AFR[0] |= (3 << 20);    // 0x00000200   (AFR[0].(11~8)=0b0010): Connect TIM8 pins(PI5) to AF3(TIM8..11)
             // PI5 ==> TIM8_CH1

    // Assign 'Interrupt Period' and 'Output Pulse Period'
    TIM8->PSC = 1680 - 1;   // Prescaler 168MHz/1680 = 0.1MHz (10us)
    TIM8->ARR = 20000 - 1;   // Auto reload  : 10us * 20K = 200ms(period)

    // CR1 : Up counting
    TIM8->CR1 &= ~(1 << 4);   // DIR=0(Up counter)(reset state)
    TIM8->CR1 &= ~(1 << 1);   // UDIS=0(Update event Enabled): By one of following events
           //   - Counter Overflow/Underflow, 
           //    - Setting the UG bit Set,
           //   - Update Generation through the slave mode controller 
    TIM8->CR1 &= ~(1 << 2);   // URS=0(Update event source Selection): one of following events
           //   - Counter Overflow/Underflow, 
           //    - Setting the UG bit Set,
           //   - Update Generation through the slave mode controller 
    TIM8->CR1 &= ~(1 << 3);   // OPM=0(The counter is NOT stopped at update event) (reset state)
    TIM8->CR1 &= ~(1 << 7);   // ARPE=0(ARR is NOT buffered) (reset state)
    TIM8->CR1 &= ~(3 << 8);    // CKD(Clock division)=00(reset state)
    TIM8->CR1 &= ~(3 << 5);    // CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
           // Center-aligned mode: The counter counts Up and DOWN alternatively

    // Event & Interrup Enable : UI  
    TIM8->EGR |= (1 << 0);    // UG: Update generation    

    ////////////////////////////////
    // Disable Tim8 Update interrupt

    // Define the corresponding pin by 'Output'  
    TIM8->CCER |= (1 << 0);   // CC3E=1: CC3 channel Output Enable
           // OC1(TIM8_CH1) Active: 해당핀을 통해 신호출력
    TIM8->CCER &= ~(1 << 1);   // CC3P=0: CC1channel Output Polarity (OCPolarity_High : OC1으로 반전없이 출력)  


    // 'Mode' Selection : Output mode, toggle  
    TIM8->CCMR1 &= ~(3 << 0); // CC3S(CC1 channel) = '0b00' : Output 
    TIM8->CCMR1 &= ~(1 << 3); // OC3P=0: Output Compare 3 preload disable
    TIM8->CCMR1 |= (3 << 4);   // OC3M=0b011: Output Compare 3 Mode : toggle
           // OC1REF toggles when CNT = CCR3

    TIM8->CCR1 = 10000;   // TIM8 CCR3 TIM8_Pulse
    TIM8->BDTR |= (1 << 15);
    ////////////////////////////////
    // Disable Tim8 CC1 interrupt

    TIM8->CR1 |= (1 << 0);   // CEN: Enable the Tim8 Counter                 
}
void USART1_IRQHandler(void)
{
    UINT8 ch1;

    if (USART1->SR & USART_SR_RXNE) // USART_SR_RXNE= 1? RX Buffer Full?
       // #define  USART_SR_RXNE ((uint16_t)0x0020)    //  Read Data Register Not Empty     
    {
        ch1 = (uint16_t)(USART1->DR & (uint16_t)0x01FF);   // 수신된 문자 저장
        if (CRdata_PC == 1)
        {
            LCD_DisplayText(3, 0, "                              ");
            no_PC = 0;
            CRdata_PC = 0;
        }
        if (ch1 == 0x0D) CRdata_PC++;

        if (ch1 != 0x0D)
            LCD_DisplayChar(3, no_PC++, ch1);
        // PC에서 보낸 데이터를 rxQue[1]에 저장
        Que_PutByte(&rxQue[1], ch1);

    }
    // DR 을 읽으면 SR.RXNE bit(flag bit)는 clear 된다. 즉 강제 clear 할 필요없음 

    if (USART1->SR & USART_SR_TXE) // USART_SR_TXE= 1? TX Buffer EMPTY?
       // #define  USART_SR_TXE ((uint16_t)0x0080)    //  WRITE Data Register Empty     
    {
        if (Que_GetSize(&txQue[1]) != 0)
        {
            // MCU --> PC 전송
            Que_GetByte(&txQue[1], &ch1);
            USART1->DR = ch1;
        }
        else
            USART1->CR1 &= ~(1 << 7);  // TXE interrupt Disable
    }
}
int mycnt = 0;      //1 카운트 변수
int mycnt2 = 0;     //2 카운트 변수
uint8_t str2[20];   //문자 저장 배열
int k = 0;
void UART4_IRQHandler(void)
{
    UINT8 ch1;
    if ((UART4->SR & USART_SR_RXNE)) // USART_SR_RXNE= 1? RX Buffer Full?
       // #define  USART_SR_RXNE ((uint16_t)0x0020)    //  Read Data Register Not Empty     
    {
        //      UART4->SR &= ~USART_SR_RXNE;
        ch1 = UART4->DR;   // 수신된 문자 저장

        if ((CRdata_BT == 0) && (ch1 == 0x0D))
        {
            no_BT = 0;
        }
        if (ch1 == 0x0A) CRdata_BT++;
        if (CRdata_BT == 2) CRdata_BT = 0;

        if ((ch1 != 0x0D) && (ch1 != 0x0A))
        {
            str2[k] = ch1;                              //배열에 문자 저장
            k++;                                        //배열 인덱스 up
            for (int i = 0; i < k; i++)                 //1, 2 카운트 for문
            {
                if (str2[i] == '1')
                {
                    mycnt++;                            //1개수 up
                    if (mycnt == 1)                     //1개수 1개 일 시
                    {
                        for (int j = 0; j < 20; j++)    //배열 초기화
                        {
                            str2[i] = NULL;
                        }
                        DispayTitle();                  //내부온도 화면 활성화
                        mycnt = 0;                      //변수 초기화
                        mycnt2 = 0;
                        start = 1;
                        k = 0;
                    }
                }
                else if (str2[i] == '2')
                {
                    mycnt2++;                           //2개수 up

                    if (mycnt2 == 1)                    //2개수 1개 일 시
                    {
                        for (int j = 0; j < 20; j++)    //배열 초기화
                        {
                            str2[i] = NULL;

                        }
                    }
                    Dis();                          //내부온도 화면 활성화
                    mycnt = 0;                      //변수 초기화
                    mycnt2 = 0;
                    start = 2;
                    k = 0;
                }
            }
        }

        // BT에서 보낸 데이터를 rxQue[2]에 저장
        Que_PutByte(&rxQue[2], ch1);  // 

    }
    // DR 을 읽으면 SR.RXNE bit(flag bit)는 clear 된다. 즉 강제 clear 할 필요없음 
    if (UART4->SR & USART_SR_TXE) // USART_SR_TXE= 1? TX Buffer EMPTY?
       // #define  USART_SR_TXE ((uint16_t)0x0080)    //  WRITE Data Register Empty     
    {
        if (Que_GetSize(&txQue[2]) != 0)
        {
            // MCU --> BT
            Que_GetByte(&txQue[2], &ch1);
            UART4->DR = (ch1 & 0x00FF);
        }
        else
            UART4->CR1 &= ~(1 << 7);  // TXE interrupt Disable
    }
}

int IUART_GetSize(UINT8 id)//id=1 : PC(USART1) QUEUE
                    // id=2 : BT(UART4) QUEUE
{
    return Que_GetSize(&rxQue[id]);
}
void IUART_GetData(UINT8 id, UINT8* ch1)
{
    Que_GetByte(&rxQue[id], ch1);
}
void IUART_SendData(UINT8 id, UINT8 ch1)
{
    Que_PutByte(&txQue[id], ch1);
}

void _GPIO_Init(void)
{
    // LED (GPIO G) 설정
    RCC->AHB1ENR |= 0x00000040;   // RCC_AHB1ENR : GPIOG(bit#6) Enable                     
    GPIOG->MODER |= 0x00005555;   // GPIOG 0~7 : Output mode (0b01)                  
    GPIOG->OTYPER &= ~0x00FF;   // GPIOG 0~7 : Push-pull  (GP8~15:reset state)   
    GPIOG->OSPEEDR |= 0x00005555;   // GPIOG 0~7 : Output speed 25MHZ Medium speed 

    // SW (GPIO H) 설정 
    RCC->AHB1ENR |= 0x00000080;   // RCC_AHB1ENR : GPIOH(bit#7) Enable                     
    GPIOH->MODER &= ~0xFFFF0000;   // GPIOH 8~15 : Input mode (reset state)            
    GPIOH->PUPDR &= ~0xFFFF0000;   // GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state

    // Buzzer (GPIO F) 설정 
    RCC->AHB1ENR |= 0x00000020; // RCC_AHB1ENR : GPIOF(bit#5) Enable                     
    GPIOF->MODER |= 0x00040000;   // GPIOF 9 : Output mode (0b01)                  
    GPIOF->OTYPER &= ~0x0200;   // GPIOF 9 : Push-pull     
    GPIOF->OSPEEDR |= 0x00040000;   // GPIOF 9 : Output speed 25MHZ Medium speed 
}

void USART1_Init(void)
{
    // USART1 : TX(PA9)
    RCC->AHB1ENR |= (1 << 0);   // RCC_AHB1ENR GPIOA Enable
    GPIOA->MODER |= (2 << 2 * 9);   // GPIOA PIN9 Output Alternate function mode               
    GPIOA->OSPEEDR |= (3 << 2 * 9);   // GPIOA PIN9 Output speed (100MHz Very High speed)
    GPIOA->PUPDR |= (1 << 2 * 9);   // GPIOA  PIN9 : Pull-up   
    GPIOA->AFR[1] |= (7 << 4);   // Connect GPIOA pin9 to AF7(USART1)

    // USART1 : RX(PA10)
    GPIOA->MODER |= (2 << 2 * 10);   // GPIOA PIN10 Output Alternate function mode
    GPIOA->OSPEEDR |= (3 << 2 * 10);   // GPIOA PIN10 Output speed (100MHz Very High speed
    GPIOA->PUPDR |= (1 << 2 * 10);   // GPIOA  PIN10 : Pull-up   
    GPIOA->AFR[1] |= (7 << 8);   // Connect GPIOA pin10 to AF7(USART1)

    RCC->APB2ENR |= (1 << 4);   // RCC_APB2ENR USART1 Enable

    USART1_BRR_Configuration(9600); // USART1 Baud rate Configuration

    USART1->CR1 &= ~(1 << 12);   // USART_WordLength 8 Data bit
    USART1->CR1 &= ~(1 << 10);   // NO USART_Parity

    USART1->CR1 |= (1 << 2);   // 0x0004, USART_Mode_RX Enable
    USART1->CR1 |= (1 << 3);   // 0x0008, USART_Mode_Tx Enable
    USART1->CR2 &= ~(3 << 12);   // 0b00, USART_StopBits_1
    USART1->CR3 = 0x0000;   // No HardwareFlowControl, No DMA

    USART1->CR1 |= (1 << 5);   // 0x0020, RXNE interrupt Enable
    USART1->CR1 &= ~(1 << 7);   // 0x0080, TXE interrupt Disable

    NVIC->ISER[1] |= (1 << (37 - 32));// Enable Interrupt USART1 (NVIC 37번)
    USART1->CR1 |= (1 << 13);   //  0x2000, USART1 Enable
}

// BlueTooth(UART4) Init
void UART4_Init(void)
{
    // UART4 : TX(PC10)
    RCC->AHB1ENR |= (1 << 2);   // RCC_AHB1ENR GPIOC Enable
    GPIOC->MODER |= (2 << 2 * 10);   // GPIOC PIN10 Output Alternate function mode               
    GPIOC->OSPEEDR |= (3 << 2 * 10);   // GPIOC PIN10 Output speed (100MHz Very High speed)
    GPIOC->PUPDR |= (1 << 2 * 10);   // GPIOC  PIN10 : Pull-up   
    GPIOC->AFR[1] |= (8 << 4 * (10 - 8));   // Connect GPIOC pin10 to AF8(UART4)

    // UART4 : RX(PC11)
    GPIOC->MODER |= (2 << 2 * 11);   // GPIOC PIN11 Output Alternate function mode
    GPIOC->OSPEEDR |= (3 << 2 * 11);   // GPIOC PIN11 Output speed (100MHz Very High speed
    GPIOC->PUPDR |= (1 << 2 * 11);   // GPIOC PIN11 Pull-up   
    GPIOC->AFR[1] |= (8 << 4 * (11 - 8));   // Connect GPIOC pin11 to AF8(UART4)

    // UART4 : RST(PC13) : GPIO
    GPIOC->MODER |= (1 << 2 * 13);   // GPIOC PIN13 Output mode               
    GPIOC->OSPEEDR |= (3 << 2 * 13);   // GPIOC PIN13  Output speed (100MHz Very High speed)
    GPIOC->ODR |= (1 << 13);       // BT Reset

    RCC->APB1ENR |= (1 << 19);   // RCC_APB1ENR UART4 Enable

    UART4_BRR_Configuration(9600); // USART Baud rate Configuration

    UART4->CR1 &= ~(1 << 12);   // USART_WordLength 8 Data bit
    UART4->CR1 &= ~(1 << 10);   // NO USART_Parity

    UART4->CR1 |= (1 << 2);   // 0x0004, USART_Mode_RX Enable
    UART4->CR1 |= (1 << 3);   // 0x0008, USART_Mode_Tx Enable
    UART4->CR2 &= ~(3 << 12);   // 0b00, USART_StopBits_1
    UART4->CR3 = 0x0000;   // No HardwareFlowControl, No DMA

    UART4->CR1 |= (1 << 5);   // 0x0020, RXNE interrupt Enable
    UART4->CR1 &= ~(1 << 7);   // 0x0080, TXE interrupt Disable

    NVIC->ISER[1] |= (1 << (52 - 32));// Enable Interrupt UART4 (NVIC 52번)
    UART4->CR1 |= (1 << 13);   //  0x2000, UART4 Enable
}

void SerialSendChar_PC(uint8_t Ch) // 1문자 보내기 함수
{
    // USART_SR_TXE(1<<7)=0?, TX Buffer NOT Empty? 
    // TX buffer Empty되지 않으면 계속 대기(송신 가능한 상태까지 대기)
    while ((USART1->SR & USART_SR_TXE) == RESET);
    USART1->DR = (Ch & 0x01FF);   // 전송 (최대 9bit 이므로 0x01FF과 masking)
}
void SerialSendString_PC(char* str) // 여러문자 보내기 함수
{
    while (*str != '\0') // 종결문자가 나오기 전까지 구동, 종결문자가 나온후에도 구동시 메모리 오류 발생가능성 있음.
    {
        SerialSendChar_PC(*str);// 포인터가 가르키는 곳의 데이터를 송신
        str++;          // 포인터 수치 증가
    }
}
void SerialSendChar_BT(uint8_t Ch) // 1문자 보내기 함수
{
    // USART_SR_TXE(1<<7)=0?, TX Buffer NOT Empty? 
    // TX buffer Empty되지 않으면 계속 대기(송신 가능한 상태까지 대기)
    while ((UART4->SR & USART_SR_TXE) == RESET);
    UART4->DR = (Ch & 0x01FF);   // 전송 (최대 9bit 이므로 0x01FF과 masking)
}
void SerialSendString_BT(char* str) // 여러문자 보내기 함수
{
    while (*str != '\0') // 종결문자가 나오기 전까지 구동, 종결문자가 나온후에도 구동시 메모리 오류 발생가능성 있음.
    {
        SerialSendChar_BT(*str);// 포인터가 가르키는 곳의 데이터를 송신
        str++;          // 포인터 수치 증가
    }
}

// USART1 Baud rate 설정
void USART1_BRR_Configuration(uint32_t USART_BaudRate)
{
    uint32_t tmpreg = 0x00;
    uint32_t APB2clock = 84000000;   //PCLK2_Frequency
    uint32_t integerdivider = 0x00;
    uint32_t fractionaldivider = 0x00;

    // Find the integer part 
    if ((USART1->CR1 & USART_CR1_OVER8) != 0) // USART_CR1_OVER8=(1<<15)
       //  #define  USART_CR1_OVER8 ((uint16_t)0x8000) // USART Oversampling by 8 enable   
    {       // USART1->CR1.OVER8 = 1 (8 oversampling)
       // Computing 'Integer part' when the oversampling mode is 8 Samples 
        integerdivider = ((25 * APB2clock) / (2 * USART_BaudRate));  // 공식에 100을 곱한 곳임(소수점 두번째자리까지 유지하기 위함)  
    }
    else  // USART1->CR1.OVER8 = 0 (16 oversampling)
    {   // Computing 'Integer part' when the oversampling mode is 16 Samples 
        integerdivider = ((25 * APB2clock) / (4 * USART_BaudRate));  // 공식에 100을 곱한 곳임(소수점 두번째자리까지 유지하기 위함)    
    }
    tmpreg = (integerdivider / 100) << 4;

    // Find the fractional part 
    fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

    // Implement the fractional part in the register 
    if ((USART1->CR1 & USART_CR1_OVER8) != 0)
    {   // 8 oversampling
        tmpreg |= (((fractionaldivider * 8) + 50) / 100) & (0x07);
    }
    else   // 16 oversampling
    {
        tmpreg |= (((fractionaldivider * 16) + 50) / 100) & (0x0F);
    }

    // Write to USART BRR register
    USART1->BRR = (uint16_t)tmpreg;
}

// UART4 Baud rate 설정
void UART4_BRR_Configuration(uint32_t USART_BaudRate)
{
    uint32_t tmpreg = 0x00;
    uint32_t APB1clock = 42000000;   //PCLK1_Frequency
    uint32_t integerdivider = 0x00;
    uint32_t fractionaldivider = 0x00;

    // Find the integer part 
    if ((UART4->CR1 & USART_CR1_OVER8) != 0) // USART_CR1_OVER8=(1<<15)
       //  #define  USART_CR1_OVER8 ((uint16_t)0x8000) // USART Oversampling by 8 enable   
    {       // UART4->CR1.OVER8 = 1 (8 oversampling)
    // Computing 'Integer part' when the oversampling mode is 8 Samples 
        integerdivider = ((25 * APB1clock) / (2 * USART_BaudRate));  // 공식에 100을 곱한 곳임(소수점 두번째자리까지 유지하기 위함)  
    }
    else    // UART4->CR1.OVER8 = 0 (16 oversampling)
    {   // Computing 'Integer part' when the oversampling mode is 16 Samples 
        integerdivider = ((25 * APB1clock) / (4 * USART_BaudRate));  // 공식에 100을 곱한 곳임(소수점 두번째자리까지 유지하기 위함)    
    }
    tmpreg = (integerdivider / 100) << 4;

    // Find the fractional part 
    fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

    // Implement the fractional part in the register 
    if ((UART4->CR1 & USART_CR1_OVER8) != 0)
    {   // 8 oversampling
        tmpreg |= (((fractionaldivider * 8) + 50) / 100) & (0x07);
    }
    else   // 16 oversampling
    {
        tmpreg |= (((fractionaldivider * 16) + 50) / 100) & (0x0F);
    }

    // Write to UART BRR register
    UART4->BRR = (uint16_t)tmpreg;
}

void DispayTitle(void)
{
    LCD_Clear(RGB_WHITE);           //흰색으로 화면클리어
    LCD_SetFont(&Gulim7);
    LCD_SetBackColor(RGB_YELLOW);   //배경색
    LCD_SetTextColor(RGB_BLACK);   //글자색
    LCD_SetPenColor(RGB_GREEN);      //펜색 : 초록
    LCD_DrawRectangle(0, 0, 159, 43); //사각형 그려주기
    LCD_SetBrushColor(RGB_YELLOW);

    LCD_DrawFillRect(1, 1, 158, 42);    //사각형 그려주기
    LCD_SetTextColor(RGB_BLUE);         //글자색 : 파랑
    LCD_DisplayText(0, 2, "<Internal Temp.>");
    LCD_SetTextColor(RGB_BLACK);        //글자색 검정
    LCD_DisplayText(1, 2, "EXT TEMP:39.1C (3.30V)");
    LCD_SetTextColor(RGB_RED);      //글자색 : 빨강
    LCD_DisplayText(1, 11, "39.1C (3.30V)");
    LCD_SetTextColor(RGB_BLACK);    //글자색 : 검정
    LCD_DisplayChar(1, 15, 'C');
    LCD_DisplayChar(1, 17, '(');
    LCD_DisplayChar(1, 23, ')');
    LCD_DisplayChar(1, 22, 'V');
    //막대 사각형 그려주기 색 : 초록
    LCD_SetBrushColor(RGB_GREEN);   //브러쉬색 : 초록
    LCD_SetPenColor(RGB_GREEN);      //펜색 : 초록
    LCD_DrawRectangle(10, 28, 139, 10);
    LCD_DrawFillRect(11, 29, 138, 9);
}
void Dis(void)
{
    LCD_Clear(RGB_WHITE);
    LCD_SetFont(&Gulim7);
    LCD_SetBackColor(RGB_YELLOW);   //배경색
    LCD_SetTextColor(RGB_BLACK);   //글자색
    LCD_SetPenColor(RGB_GREEN);      //펜색 : 초록
    LCD_DrawRectangle(0, 0, 159, 43); //사각형 그려주기
    LCD_SetBrushColor(RGB_YELLOW);

    LCD_DrawFillRect(1, 1, 158, 42);
    LCD_SetTextColor(RGB_BLUE);
    LCD_DisplayText(0, 2, "<Internal Temp.>");
    LCD_SetTextColor(RGB_BLACK);
    LCD_DisplayText(1, 2, "INT TEMP:39.1C");
    LCD_SetTextColor(RGB_RED);      //글자색 : 빨강
    LCD_DisplayText(1, 11, "39.1C");

    //막대 사각형 그려주기 색 : 초록
    LCD_SetBrushColor(RGB_PINK);
    LCD_SetPenColor(RGB_PINK);      //펜색 : 초록
    LCD_DrawRectangle(10, 28, 139, 10);
    LCD_DrawFillRect(11, 29, 138, 9);
}
int aa = 500;
void Connect()              //블루투스 자동 연결 함수
{
    // AT
    SerialSendString_BT("AT"); // BT module 연결 확인
    SerialSendChar_BT(0x0D); // CR(Carriage Return : 0x0D)
    SerialSendString_PC("AT"); // BT module 연결 해제
    DelayMS(aa);
    // 이름 BTNAME
    SerialSendString_BT("AT+BTNAME=TUE02"); // BT module 이름 설정(변경)
    SerialSendChar_BT(0x0D); // CR(Carriage Return : 0x0D)
    SerialSendString_PC("AT+BTNAME=TUE02"); // BT module 연결 해제
    DelayMS(aa);
    // 리셋 ATZ
    SerialSendString_BT("ATZ"); // BT 명령후에 명령 인식을 위한 소프트웨어 리셋
    SerialSendChar_BT(0x0D);  // CR(Carriage Return : 0x0D)
    SerialSendString_PC("ATZ"); // BT module 연결 해제
    DelayMS(aa);
    // 이름 확인
    SerialSendString_BT("AT+BTINFO?0"); // BT module 이름 확인
    SerialSendChar_BT(0x0D); // CR(Carriage Return : 0x0D)
    SerialSendString_PC("AT+BTINFO?0"); // BT module 연결 해제
    DelayMS(aa);
    SerialSendString_BT("AT+BTSCAN");
    SerialSendChar_BT(0x0D); // USART1을 통해 PC로 “HELLO! ” 문자열을 전송
    SerialSendString_PC("AT+BTSCAN"); // BT module 연결 해제
    // 검색가능 BTSCAN
//    SerialSendString_BT("AT+BTSCAN"); // BT module 을 외부 BT 기기(Master)에서 찾게 하기 위한 명령
//    SerialSendChar_BT(0x0D); // CR(Carriage Return : 0x0D)
////    SerialSendString_PC("AT+BTSCAN"); // BT module 연결 해제
////    DelayMS(1000);

}
void DelayMS(unsigned short wMS)
{
    register unsigned short i;
    for (i = 0; i < wMS; i++)
        DelayUS(1000);  // 1000us => 1ms
}
void DelayUS(unsigned short wUS)
{
    volatile int Dly = (int)wUS * 17;
    for (; Dly; Dly--);
}
uint8_t key_flag = 0;
uint16_t KEY_Scan(void)   // input key SW0 - SW7 
{
    uint16_t key;
    key = GPIOH->IDR & 0xFF00;   // any key pressed ?
    if (key == 0xFF00)      // if no key, check key off
    {
        if (key_flag == 0)
            return key;
        else
        {
            DelayMS(10);
            key_flag = 0;
            return key;
        }
    }
    else            // if key input, check continuous key
    {
        if (key_flag != 0)   // if continuous key, treat as no key input
            return 0xFF00;
        else         // if new key,delay for debounce
        {
            key_flag = 1;
            DelayMS(10);
            return key;
        }
    }
}
void BEEP(void)         // Beep for 20 ms 
{
    GPIOF->ODR |= (1 << 9);   // PF9 'H' Buzzer on
    DelayMS(20);      // Delay 20 ms
    GPIOF->ODR &= ~(1 << 9);   // PF9 'L' Buzzer off
}
