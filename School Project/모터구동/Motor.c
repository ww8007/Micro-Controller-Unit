//////////////////////////////////////////////////////////
// Output Compare Mode
// Timer4 CH1
// Compare Match Interrupt(CC1I) 발생 & OC1을 통한 Pulse 출력
/////////////////////////////////////////////////////////////
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

#define GOAL_DEG    360
#define STEP_DEG    15  // 7.5
#define BUFF_SIZE 256




void _GPIO_Init(void);
void _EXTI_Init(void);
uint16_t KEY_Scan(void);
void TIMER2_Init(void);   // General-purpose Timer 4 (Output Compare mode)
void USART1_Init(void);
void DisplayInitScreen(void);
void USART1_BRR_Configuration(uint32_t USART_BaudRate);
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);
void SerialSendChar_PC(uint8_t c);
void SerialSendString_PC(char* s);
int IUART_GetSize(UINT8 id);
void IUART_GetData(UINT8 id, UINT8* ch1);
void IUART_SendData(UINT8 id, UINT8 ch1);
void _ADC_Init(void);

UINT16 INT_COUNT;
UINT8 strBuffer[2][100];
UINT8 strBufferIdx[2];
UINT8 size;
UINT8 data;
UINT8 ch;
UINT8 no_PC, no_BT, CRdata_PC, CRdata_BT;
UINT8 mode = 0;
int stepdg = 0;
int goaldg = 0;
int totpulse = 0;
int crrpulse = 0;
UINT8 rpm = 0;
uint16_t ADC_Value, Voltage;  //외부온도 변수
int main(void)
{
    _GPIO_Init();
    _EXTI_Init();
    USART1_Init();

    LCD_Init();
    Que_Clear(&txQue[1]);   // PC(USART1) QUEUE clear
    Que_Clear(&rxQue[1]);
    Que_Clear(&txQue[2]);   // BT(UART4) QUEUE clear
    Que_Clear(&rxQue[2]);
    DelayMS(10);
    //BEEP();
    _ADC_Init();
    DisplayInitScreen();   // LCD 초기화면
    GPIOG->ODR &= 0xFF00;// 초기값: LED0~7 Off

    TIMER2_Init();   // TIM4 Init (Output Compare mode: UI & CCI 발생, )                  
    strBufferIdx[0] = 0;
    strBufferIdx[1] = 0;
    while (1)
    {
        // TIMER Example SW4~6을 사용하여 LED0의 밝기 조절
        switch (KEY_Scan())
        {
        case SW0_PUSH:       //SW0
            GPIOG->ODR &= 0x0F;
            GPIOG->ODR ^= 0x01;   // LED0 On
            TIM2->CCER |= (1 << 12); // CC3E Enable 
            crrpulse = 0;
            TIM2->CCR4 = (60 * stepdg / 2) / (360 * rpm);   // TIM4 CCR1 TIM4_Pulse0
            TIM2->CR1 |= (1 << 0); // TIM1 Enable전송
            //사각형 초기화면으로 초기화
            LCD_SetPenColor(RGB_GREEN);
            LCD_SetBrushColor(RGB_GREEN);
            LCD_DrawRectangle(10, 102, 140, 12);
            LCD_DrawFillRect(10, 103, 7, 11);

            break;
        case SW1_PUSH:       //SW0
            if (mode == 1);
            {
                GPIOG->ODR &= 0x0F;
                GPIOG->ODR ^= 0x01;   // LED0 On
                TIM2->CCER &= ~(1 << 12); // CC3E Disable
                TIM2->CR1 &= ~(1 << 0); // TIM3 OFF
                crrpulse = 0;
            }

            break;
        case SW5_PUSH:    //SW4
            goaldg += 100;
            LCD_SetTextColor(RGB_BLUE);
            if (goaldg >= 1000)
            {
                goaldg -= 1000;
            }

            LCD_DisplayChar(4, 13, (goaldg / 100) + 0x30);
            LCD_DisplayChar(4, 14, ((goaldg / 10) % 10) + 0x30);
            LCD_DisplayChar(4, 15, ((goaldg % 100) % 10) + 0x30);
            break;

        case SW6_PUSH:    //SW5
            goaldg += 10;
            if (goaldg >= 1000)
            {
                goaldg -= 1000;
            }
            LCD_SetTextColor(RGB_BLUE);
            LCD_DisplayChar(4, 13, (goaldg / 100) + 0x30);
            LCD_DisplayChar(4, 14, ((goaldg / 10) % 10) + 0x30);
            LCD_DisplayChar(4, 15, ((goaldg % 100) % 10) + 0x30);
            break;

        case SW7_PUSH:    //SW6
            goaldg += 1;
            if (goaldg >= 1000)
            {
                goaldg -= 1000;
            }
            LCD_SetTextColor(RGB_BLUE);
            LCD_DisplayChar(4, 13, (goaldg / 100) + 0x30);
            LCD_DisplayChar(4, 14, ((goaldg / 10) % 10) + 0x30);
            LCD_DisplayChar(4, 15, ((goaldg % 100) % 10) + 0x30);
            break;
        }
    }
    size = IUART_GetSize(2);  // BT용 QUEUE buffer
    if (size != 0)
    {
        for (int i = 0; i < size; i++)
        {
            IUART_GetData(2, &data); // Get data from BT_RX_Queue
            IUART_SendData(1, data); // Send data to PC_TX_Queue
            if (data == '\r') // 0x0D
            {
                strBufferIdx[1] = 0;
                for (int j = 0; j < 100; j++)
                    strBuffer[1][j] = 0;
            }
            else if (data != '\n') // 0x0A
            {
                strBuffer[1][strBufferIdx[1]++] = data;
            }
        }
        USART1->CR1 |= (1 << 7);   // TXE interrupt Enable
    }  // if(size != 0)    

}
// INT 주기: 10ms, CCR1 : 10%, 40%, 80%
// INT 주기: 2s, CCR1 : 10%, 40%, 80%
int getc[255];
int cidx = 0;
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
        // PC에서 보낸 데이터를 rxQue[1]에 저장
        Que_PutByte(&rxQue[1], ch1);
        if (ch1 != 0x0D && ch1 != 0x0A)
        {
            getc[cidx] = ch1;
            cidx++;
            if (getc[0] == 'M')
            {
                if (getc[1] == '1')
                {
                    LCD_DisplayText(2, 6, "              ");
                    LCD_DisplayText(2, 6, "Velocity");
                    mode = 1;
                    memset(getc, NULL, sizeof(getc));
                    cidx = 0;
                }
                else if (getc[1] == '2')
                {
                    LCD_DisplayText(2, 6, "              ");
                    LCD_DisplayText(2, 6, "Position");
                    mode = 2;
                    memset(getc, NULL, sizeof(getc));
                    cidx = 0;
                }
            }
            else if (getc[0] == 'S')
            {
                if (cidx == 3)
                {
                    stepdg += (getc[1] - 0x30) * 10;
                    stepdg += (getc[2] - 0x30);
                    LCD_DisplayChar(3, 13, stepdg / 10 + 0x30);
                    LCD_DisplayChar(3, 14, stepdg % 10 + 0x30);
                    memset(getc, NULL, sizeof(getc));
                    cidx = 0;
                }
            }


        }
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
void TIMER2_Init(void)
{
    // PB11: TIM2_CH4
    // PB11을 출력설정하고 Alternate function(TIM2_CH4)으로 사용 선언
    RCC->AHB1ENR |= (1 << 1);   // 0x08, RCC_AHB1ENR GPIOB Enable 

    GPIOD->MODER |= (2 << 2 * 11);   // 0x02000000(MODER.(25,24)=0b10), GPIOD PIN12 Output Alternate function mode                
    GPIOD->OSPEEDR |= (3 << 2 * 11);   // 0x03000000(OSPEEDER.(25,24)=0b11), GPIOD PIN12 Output speed (100MHz High speed)
    GPIOD->PUPDR |= (1 << 2 * 11);    // 0x01000000, GPIOD PIN12 Pull-up
                  // PD12 ==> TIM4_CH1
    GPIOD->AFR[1] |= (1 << 12);  // (AFR[1].(19~16)=0b0010): Connect TIM2 pins(PB11) to AF2(TIM3..5)

 // Timerbase 설정
    RCC->APB1ENR |= 0x01;// RCC_APB1ENR TIMER2 Enable

    // Setting CR1 : 0x0000 
    TIM2->CR1 &= ~(1 << 4);   // DIR=0(Up counter)(reset state)
    TIM2->CR1 &= ~(1 << 1);   // UDIS=0(Update event Enabled): By one of following events
                             //  Counter Overflow/Underflow, 
                             //  Setting the UG bit Set,
                             //  Update Generation through the slave mode controller 
                             // UDIS=1 : Only Update event Enabled by  Counter Overflow/Underflow,
    TIM2->CR1 &= ~(1 << 2);   // URS=0(Update event source Selection): one of following events
                             //   Counter Overflow/Underflow, 
                             // Setting the UG bit Set,
                             //   Update Generation through the slave mode controller 
                             // URS=1 : Only Update Interrupt generated  By  Counter Overflow/Underflow,
    TIM2->CR1 &= ~(3 << 8);    // CKD(Clock division)=00(reset state)
    TIM2->CR1 &= ~(3 << 5);    // CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
             // Center-aligned mode: The counter counts Up and DOWN alternatively
    TIM2->EGR |= (1 << 4);   // CC4 gerneration
    // Setting the Period
    TIM2->PSC = 8400 - 1;   // Prescaler=84, 84MHz/8400 = 1kHz (0.1ms)
    TIM2->ARR = 1000 - 1;   // Auto reload  : 0.1s
    TIM2->CCER &= ~(1 << 12);   // CC4E: OC4 Active 
    TIM2->CCER |= (1 << 13);  // CC4P: OCPolarity_High
    TIM2->CCR4 = 500;   // TIM2_Pulse
    // Update(Clear) the Counter
    TIM2->EGR |= (1 << 4);    // UG: Update generation    
    TIM2->DIER |= (1 << 4);   // CC4IE: Enable the Tim2 CC4 interrupt
 // Output Compare 설정
    // CCMR2(Capture/Compare Mode Register 2) : Setting the MODE of Ch3 or Ch4
    TIM2->CCMR2 &= ~(3 << 8); // CC4S(CC1 channel) = '0b00' : Output 
    TIM2->CCMR2 &= ~(1 << 11); // OC4PE=0: Output Compare 3 preload enable
    TIM2->CCMR2 |= (3 << 12);   // OC4M=0b011 (Output Compare 3 Mode : toggle)
             // OC1REF toggles when CNT = CCR1
    TIM2->DIER |= (1 << 4);   // UIE: Enable Tim4 Update interrupt
    TIM2->DIER |= (1 << 7);   // CC4IE: Enable the Tim4 CC1 interrupt

    NVIC->ISER[0] |= (1 << 28);   // Enable Timer4 global Interrupt on NVIC


}
void ADC_IRQHandler(void)
{
    GPIOG->ODR ^= 0x01;
    ADC1->SR &= ~(1 << 1);      // EOC flag clear

    ADC_Value = ADC1->DR;      // Reading ADC result
    Voltage = ADC_Value * (3.3 * 10) / 4095;   // 3.3 : 4095 =  Volatge : ADC_Value 
    // 1 : 4095
    if (Voltage <= 8)
    {
        rpm = 1;
    }
    else if (Voltage >= 9 && Voltage <= 16)
    {
        rpm = 2;
    }
    else if (Voltage >= 17 && Voltage <= 24)
    {
        rpm = 3;
    }
    else if (Voltage >= 25 && Voltage <= 33)
    {
        rpm = 4;
    }


}
void TIM2_IRQHandler(void)      //RESET: 0
{
    if ((TIM2->SR & 0x10) != RESET)   //CC4 INT
    {
        TIM2->SR &= ~0x10;   // CC4 Interrupt Clear
        crrpulse++;
        LCD_DisplayChar(7, 15, (crrpulse / 100) + 0x30);
        LCD_DisplayChar(7, 16, (crrpulse / 10 % 10) + 0x30);
        LCD_DisplayChar(7, 17, (crrpulse % 10) + 0x30);
        if (crrpulse < 10)
        {
            SerialSendChar_PC(crrpulse + 0x30);
        }
        else if (crrpulse >= 10 && crrpulse < 100)
        {
            SerialSendChar_PC((crrpulse / 10) + 0x30);
            SerialSendChar_PC((crrpulse % 10) + 0x30);
        }
        else
        {
            SerialSendChar_PC((crrpulse / 100) + 0x30);
            SerialSendChar_PC((crrpulse / 10 % 10) + 0x30);
            SerialSendChar_PC((crrpulse % 10) + 0x30);
        }
        totpulse = (goaldg / stepdg);
        LCD_DisplayChar(6, 19, (totpulse / 100) + 0x30);
        LCD_DisplayChar(6, 20, (totpulse / 10 % 10) + 0x30);
        LCD_DisplayChar(6, 21, (totpulse % 10) + 0x30);
        if (mode == 2)
        {
            GPIOG->ODR ^= 0x04;
            LCD_DrawFillRect(10, 103, (7 + (66 * (crrpulse / totpulse))), 11);
            if (crrpulse >= 2 * totpulse)
            {
                TIM2->CCER &= ~(1 << 12);// CC3E Disable 
                TIM2->CR1 &= ~(1 << 0); // TIM2 Disable
                crrpulse = 0;
                GPIOG->ODR &= ~(0x01);   // LED0 

                LCD_DrawFillRect(11, 103, 139, 11);     //max mode
            }
        }
    }

    if ((TIM4->SR & 0x04) != RESET)   // Capture/Compare 1 interrupt flag
    {
        TIM4->SR &= ~(1 << 4);   // CC 1 Interrupt Claer
        GPIOG->ODR &= ~0x01;   // LED0 Off
    }
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
void _ADC_Init(void)
{   // ADC2: PA1(pin 41)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // (1<<0) ENABLE GPIOA CLK (stm32f4xx.h 참조)
    GPIOA->MODER |= (3 << 2 * 1);      // CONFIG GPIOA PIN0(PA0) TO ANALOG IN MODE

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;   // (1<<9) ENABLE ADC1 CLK (stm32f4xx.h 참조)

    ADC->CCR &= ~(0X1F << 0);      // MULTI[4:0]: ADC_Mode_Independent
    ADC->CCR |= (1 << 16);       // 0x00010000 ADCPRE:ADC_Prescaler_Div4 (ADC MAX Clock 36MHz, 84Mhz(APB2)/4 = 21MHz)

    ADC1->CR1 &= ~(3 << 24);      // RES[1:0]=0b00 : 12bit Resolution
    ADC1->CR1 &= ~(1 << 8);      // SCAN=0 : ADC_ScanCovMode Disable
    ADC1->CR1 |= (1 << 5);      // EOCIE=1: Interrupt enable for EOC

    ADC1->CR2 &= ~(1 << 1);      // CONT=0: ADC_Continuous ConvMode Disable
    ADC1->CR2 |= (3 << 28);      // EXTEN[1:0]=0b00: ADC_ExternalTrigConvEdge_Falling
    ADC1->CR2 |= (0x0F << 24);   // EXTI 11 Line Event
    ADC1->CR2 &= ~(1 << 11);      // ALIGN=0: ADC_DataAlign_Right
    ADC1->CR2 &= ~(1 << 10);      // EOCS=0: The EOC bit is set at the end of each sequence of regular conversions

    ADC1->SQR1 &= ~(0xF << 20);   // L[3:0]=0b0000: ADC Regular channel sequece length 
    // 0b0000:1 conversion)
    ADC1->SMPR2 |= (0x7 << (3 * 1));   // ADC1_CH1 Sample Time_480Cycles (3*Channel_1)
    //Channel selection, The Conversion Sequence of PIN1(ADC2_CH0) is first, Config sequence Range is possible from 0 to 17
    ADC1->SQR3 |= (1 << 0);   // SQ1[4:0]=0b0000 : CH1
    NVIC->ISER[0] |= (1 << 18);   // Enable ADC global Interrupt

}
void _EXTI_Init(void)
{
    RCC->AHB1ENR |= 0x0080;   // RCC_AHB1ENR GPIOH Enable
    RCC->APB2ENR |= 0x4000;   // Enable System Configuration Controller Clock

    GPIOH->MODER &= 0x0000FFFF;   // GPIOH PIN8~PIN15 Input mode (reset state)             

    SYSCFG->EXTICR[2] |= 0x7000;    // EXTI8,9에 대한 소스 입력은 GPIOH로 설정 (EXTICR3) (reset value: 0x0000)   

    EXTI->FTSR |= 0x008888;      // Falling Trigger Enable  (EXTI8:PH8)
    EXTI->RTSR |= 0x008888;      // Rising Trigger  Enable  (EXTI9:PH9) 
    EXTI->IMR |= 0x008888;     // EXTI8,9 인터럽트 mask (Interrupt Enable)

    NVIC->ISER[1] |= (1 << 8);      // Enable Interrupt EXTI8,9 Vector table Position 참조
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
void EXTI15_10_IRQHandler(void)      // EXTI 15~10 인터럽트 핸들러
{

    if (EXTI->PR & 0x0800)       // EXTI11 Interrupt Pending?
    {
        GPIOG->ODR &= 0x0F;
        GPIOG->ODR ^= 0x08;   // LED0 On
        EXTI->PR |= 0x0800;    // Pending bit Clear
        ADC1->CR2 |= (1 << 0);      // ADON: ADC ON
        LCD_SetTextColor(RGB_BLUE);   // 글자색 : Blue
        LCD_DisplayChar(5, 13, rpm + 0x30);
    }
}

void BEEP(void)         // Beep for 20 ms 
{
    GPIOF->ODR |= 0x0200;   // PF9 'H' Buzzer on
    DelayMS(20);      // Delay 20 ms
    GPIOF->ODR &= ~0x0200;   // PF9 'L' Buzzer off
}

void DelayMS(unsigned short wMS)
{
    register unsigned short i;
    for (i = 0; i < wMS; i++)
        DelayUS(1000);   // 1000us => 1ms
}

void DelayUS(unsigned short wUS)
{
    volatile int Dly = (int)wUS * 17;
    for (; Dly; Dly--);
}

void DisplayInitScreen(void)
{
    LCD_Clear(RGB_WHITE);      // 화면 클리어
    LCD_SetFont(&Gulim7);      // 폰트 : 굴림 7
    LCD_SetBackColor(RGB_YELLOW);   // 글자배경색 : YELLOW
    LCD_SetTextColor(RGB_BLACK);   // 글자색 : Black
    LCD_SetPenColor(RGB_BLACK);      // 펜 색 : BLACK 
    LCD_SetBrushColor(RGB_YELLOW);

    LCD_DrawRectangle(2, 2, 130, 13);
    LCD_SetBrushColor(RGB_YELLOW);
    LCD_DrawFillRect(3, 3, 129, 12);
    LCD_DisplayText(0, 1, "Step Motor Pulse Gen.");  // Title
    LCD_SetBackColor(RGB_WHITE);   //글자배경색 : WHITE

    LCD_DisplayText(2, 0, "Mode: ");
    LCD_DisplayText(3, 0, "Step deg(S):   (deg/p)");
    LCD_DisplayText(4, 0, "Goal deg(G):    (deg)");
    LCD_DisplayText(5, 0, "Goal RPM(R):  (rpm)");
    LCD_DisplayText(6, 0, "Total pulse(P=G/S):    (p)");
    LCD_DisplayText(7, 0, "Current pulse:    (p)");
    LCD_SetTextColor(RGB_BLUE);   // 글자색 : Blue
    //사각형 그려주기
    LCD_SetPenColor(RGB_GREEN);
    LCD_SetBrushColor(RGB_GREEN);
    LCD_DrawRectangle(10, 102, 140, 12);
    LCD_DrawFillRect(10, 103, 7, 11);
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

/**************************************************************************
// 보충 설명자료
// 다음은 stm32f4xx.h에 있는 RCC관련 주요 선언문임
#define HSE_STARTUP_TIMEOUT    ((uint16_t)0x05000)   // Time out for HSE start up
typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

#define FLASH_BASE            ((uint32_t)0x08000000) // FLASH(up to 1 MB) base address in the alias region
#define CCMDATARAM_BASE       ((uint32_t)0x10000000) // CCM(core coupled memory) data RAM(64 KB) base address in the alias region
#define SRAM1_BASE            ((uint32_t)0x20000000) // SRAM1(112 KB) base address in the alias region

#if defined(STM32F40_41xxx)
#define SRAM2_BASE            ((uint32_t)0x2001C000) // SRAM2(16 KB) base address in the alias region
#define SRAM3_BASE            ((uint32_t)0x20020000) // SRAM3(64 KB) base address in the alias region
#endif

#define PERIPH_BASE           ((uint32_t)0x40000000) // Peripheral base address in the alias region
#define BKPSRAM_BASE          ((uint32_t)0x40024000) // Backup SRAM(4 KB) base address in the alias region

// Peripheral memory map
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000)

// AHB1 peripherals
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASE            (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASE            (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASE            (AHB1PERIPH_BASE + 0x2000)
#define GPIOJ_BASE            (AHB1PERIPH_BASE + 0x2400)
#define GPIOK_BASE            (AHB1PERIPH_BASE + 0x2800)
#define CRC_BASE              (AHB1PERIPH_BASE + 0x3000)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800)
#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x3C00)
#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x3800)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x3C00)

// APB1 peripherals address
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00)
#define TIM6_BASE             (APB1PERIPH_BASE + 0x1000)
#define TIM7_BASE             (APB1PERIPH_BASE + 0x1400)
#define TIM12_BASE            (APB1PERIPH_BASE + 0x1800)
#define TIM13_BASE            (APB1PERIPH_BASE + 0x1C00)
#define TIM14_BASE            (APB1PERIPH_BASE + 0x2000)

// APB2 peripherals address
#define TIM1_BASE             (APB2PERIPH_BASE + 0x0000)
#define TIM8_BASE             (APB2PERIPH_BASE + 0x0400)
#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x3800)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x3C00)
#define TIM9_BASE             (APB2PERIPH_BASE + 0x4000)
#define TIM10_BASE            (APB2PERIPH_BASE + 0x4400)
#define TIM11_BASE            (APB2PERIPH_BASE + 0x4800)

// RCC Structue
typedef struct
{
  __IO uint32_t CR;            // RCC clock control register, Address offset: 0x00
  __IO uint32_t PLLCFGR;       // RCC PLL configuration register, Address offset: 0x04
  __IO uint32_t CFGR;          // RCC clock configuration register, Address offset: 0x08
  __IO uint32_t CIR;           // RCC clock interrupt register, Address offset: 0x0C
  __IO uint32_t AHB1RSTR;      // RCC AHB1 peripheral reset register, Address offset: 0x10
  __IO uint32_t AHB2RSTR;      // RCC AHB2 peripheral reset register, Address offset: 0x14
  __IO uint32_t AHB3RSTR;      // RCC AHB3 peripheral reset register, Address offset: 0x18
  __IO uint32_t APB1RSTR;      // RCC APB1 peripheral reset register, Address offset: 0x20
  __IO uint32_t APB2RSTR;      // RCC APB2 peripheral reset register, Address offset: 0x24
  __IO uint32_t AHB1ENR;       // RCC AHB1 peripheral clock register, Address offset: 0x30
  __IO uint32_t AHB2ENR;       // RCC AHB2 peripheral clock register, Address offset: 0x34
  __IO uint32_t AHB3ENR;       // RCC AHB3 peripheral clock register, Address offset: 0x38
  __IO uint32_t APB1ENR;       // RCC APB1 peripheral clock enable register, Address offset: 0x40
  __IO uint32_t APB2ENR;       // RCC APB2 peripheral clock enable register, Address offset: 0x44
  __IO uint32_t AHB1LPENR;     // RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50
  __IO uint32_t AHB2LPENR;     // RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54
  __IO uint32_t AHB3LPENR;     // RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58
  __IO uint32_t APB1LPENR;     // RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60
  __IO uint32_t APB2LPENR;     // RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64
  __IO uint32_t BDCR;          // RCC Backup domain control register, Address offset: 0x70
  __IO uint32_t CSR;           // RCC clock control & status register, Address offset: 0x74
  __IO uint32_t SSCGR;         // RCC spread spectrum clock generation register, Address offset: 0x80
  __IO uint32_t PLLI2SCFGR;    // RCC PLLI2S configuration register, Address offset: 0x84
  __IO uint32_t PLLSAICFGR;    // RCC PLLSAI configuration register, Address offset: 0x88
  __IO uint32_t DCKCFGR;       // RCC Dedicated Clocks configuration register, Address offset: 0x8C
} RCC_TypeDef;

// FLASH Structue
typedef struct
{
  __IO uint32_t ACR;      // FLASH access control register,   Address offset: 0x00
  __IO uint32_t KEYR;     // FLASH key register,              Address offset: 0x04
  __IO uint32_t OPTKEYR;  // FLASH option key register,       Address offset: 0x08
  __IO uint32_t SR;       // FLASH status register,           Address offset: 0x0C
  __IO uint32_t CR;       // FLASH control register,          Address offset: 0x10
  __IO uint32_t OPTCR;    // FLASH option control register ,  Address offset: 0x14
  __IO uint32_t OPTCR1;   // FLASH option control register 1, Address offset: 0x18
} FLASH_TypeDef;

// GPIO Structue
typedef struct
{
  __IO uint32_t MODER;    // GPIO port mode register,               Address offset: 0x00
  __IO uint32_t OTYPER;   // GPIO port output type register,        Address offset: 0x04
  __IO uint32_t OSPEEDR;  // GPIO port output speed register,       Address offset: 0x08
  __IO uint32_t PUPDR;    // GPIO port pull-up/pull-down register,  Address offset: 0x0C
  __IO uint32_t IDR;      // GPIO port input data register,         Address offset: 0x10
  __IO uint32_t ODR;      // GPIO port output data register,        Address offset: 0x14
  __IO uint16_t BSRRL;    // GPIO port bit set/reset low register,  Address offset: 0x18
  __IO uint16_t BSRRH;    // GPIO port bit set/reset high register, Address offset: 0x1A
  __IO uint32_t LCKR;     // GPIO port configuration lock register, Address offset: 0x1C
  __IO uint32_t AFR[2];   // GPIO alternate function registers,     Address offset: 0x20-0x24
} GPIO_TypeDef;

// EXTI Structue
typedef struct
{
  __IO uint32_t IMR;    // EXTI Interrupt mask register, Address offset: 0x00
  __IO uint32_t EMR;    // EXTI Event mask register, Address offset: 0x04
  __IO uint32_t RTSR;   // EXTI Rising trigger selection register,  Address offset: 0x08
  __IO uint32_t FTSR;   // EXTI Falling trigger selection register, Address offset: 0x0C
  __IO uint32_t SWIER;  // EXTI Software interrupt event register,  Address offset: 0x10
  __IO uint32_t PR;     // EXTI Pending register, Address offset: 0x14
} EXTI_TypeDef;

// SYSCFG Structue
typedef struct
{
  __IO uint32_t MEMRMP;       // SYSCFG memory remap register, Address offset: 0x00
  __IO uint32_t PMC;          // SYSCFG peripheral mode configuration register, Address offset: 0x04
  __IO uint32_t EXTICR[4];    // SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14
  __IO uint32_t CMPCR;        // SYSCFG Compensation cell control register,Address offset: 0x20

} SYSCFG_TypeDef;

// Timer Structue
typedef struct
{
  __IO uint16_t CR1;         // TIM control register 1, Address offset: 0x00
  __IO uint16_t CR2;         // TIM control register 2, 0x04
  __IO uint16_t SMCR;        // TIM slave mode control register, 0x08
  __IO uint16_t DIER;        // TIM DMA/interrupt enable register, 0x0C
  __IO uint16_t SR;          // TIM status register, 0x10
  __IO uint16_t EGR;         // TIM event generation register, 0x14
  __IO uint16_t CCMR1;       // TIM capture/compare mode register 1, 0x18
  __IO uint16_t CCMR2;       // TIM capture/compare mode register 2, 0x1C
  __IO uint16_t CCER;        // TIM capture/compare enable register, 0x20
  __IO uint32_t CNT;         // TIM counter register, 0x24
  __IO uint16_t PSC;         // TIM prescaler, 0x28
  __IO uint32_t ARR;         // TIM auto-reload register, 0x2C
  __IO uint16_t RCR;         // TIM repetition counter register, 0x30
  __IO uint32_t CCR1;        // TIM capture/compare register 1, 0x34
  __IO uint32_t CCR2;        // TIM capture/compare register 2, 0x38
  __IO uint32_t CCR3;        // TIM capture/compare register 3, 0x3C
  __IO uint32_t CCR4;        // TIM capture/compare register 4, 0x40
  __IO uint16_t BDTR;        // TIM break and dead-time register, 0x44
  __IO uint16_t DCR;         // TIM DMA control register, 0x48
  __IO uint16_t DMAR;        // TIM DMA address for full transfer, 0x4C
  __IO uint16_t OR;          // TIM option register, 0x50
} TIM_TypeDef;

// 각 주변장치 모듈 선언
#define GPIOA    ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB   ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC   ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD   ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE     ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF   ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG   ((GPIO_TypeDef *) GPIOG_BASE)
#define GPIOH   ((GPIO_TypeDef *) GPIOH_BASE)
#define GPIOI   ((GPIO_TypeDef *) GPIOI_BASE)
#define GPIOJ   ((GPIO_TypeDef *) GPIOJ_BASE)
#define GPIOK   ((GPIO_TypeDef *) GPIOK_BASE)

#define CRC     ((CRC_TypeDef *) CRC_BASE)
#define RCC     ((RCC_TypeDef *) RCC_BASE)
#define FLASH   ((FLASH_TypeDef *) FLASH_R_BASE)

#define SYSCFG  ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define EXTI    ((EXTI_TypeDef *) EXTI_BASE)

#define TIM2    ((TIM_TypeDef *) TIM2_BASE)
#define TIM3    ((TIM_TypeDef *) TIM3_BASE)
#define TIM4    ((TIM_TypeDef *) TIM4_BASE)
#define TIM5    ((TIM_TypeDef *) TIM5_BASE)
#define TIM6    ((TIM_TypeDef *) TIM6_BASE)
#define TIM7    ((TIM_TypeDef *) TIM7_BASE)
#define TIM2   ((TIM_TypeDef *) TIM12_BASE)
#define TIM13   ((TIM_TypeDef *) TIM13_BASE)

#define TIM1    ((TIM_TypeDef *) TIM1_BASE)
#define TIM8    ((TIM_TypeDef *) TIM8_BASE)
#define TIM9    ((TIM_TypeDef *) TIM9_BASE)
#define TIM10   ((TIM_TypeDef *) TIM10_BASE)
#define TIM11   ((TIM_TypeDef *) TIM11_BASE)

#define FLASH_ACR_PRFTEN             ((uint32_t)0x00000100)
#define FLASH_ACR_ICEN               ((uint32_t)0x00000200)
#define FLASH_ACR_DCEN               ((uint32_t)0x00000400)
#define FLASH_ACR_ICRST              ((uint32_t)0x00000800)
#define FLASH_ACR_DCRST              ((uint32_t)0x00001000)
#define FLASH_ACR_BYTE0_ADDRESS      ((uint32_t)0x40023C00)
#define FLASH_ACR_BYTE2_ADDRESS      ((uint32_t)0x40023C03)

#define FLASH_ACR_LATENCY_5WS        ((uint32_t)0x00000005)

typedef struct {
  __IO uint32_t ISER[8];  // Offset: 0x000 Interrupt Set Enable Register
  __IO uint32_t ICER[8];  // Offset: 0x080 Interrupt Clear Enable Register
  __IO uint32_t ISPR[8];  // Offset: 0x100 Interrupt Set Pending Register
  __IO uint32_t ICPR[8];  // Offset: 0x180 Interrupt Clear Pending Register
  __IO uint32_t IABR[8];  // Offset: 0x200 Interrupt Active bit Register
  __IO uint8_t  IP[240];  // Offset: 0x300 Interrupt Priority Register (8Bit)
  __O  uint32_t STIR;  // Offset: 0xE00 Software Trigger Interrupt Register
}  NVIC_Type;

// Memory mapping of Cortex-M4 Hardware
#define SCS_BASE     (0xE000E000)    // System Control Space Base Address
#define NVIC_BASE   (SCS_BASE +  0x0100)  // NVIC Base Address
#define NVIC        ((NVIC_Type *)  NVIC_BASE) // NVIC configuration struct

*/