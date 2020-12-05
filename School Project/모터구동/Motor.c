//////////////////////////////////////////////////////////////////////
// USART RX Interrupt 
// USART1(PC): TX pin: PA9,  RX pin: PA10 
// UART4(BT):  TX pin: PC10, RX pin: PC11, RST pin(GPIO): PC13
// TX: Interrupt 방식, RX: Interrupt 방식 
// 문자를 TX를 통해 PC(Hyper terminal)로 전송하고, 
// PC에서 보내온 문자를 받아 LCD에 표시
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

void DispayTitle(void);
void _GPIO_Init(void);
uint16_t KEY_Scan(void);

void USART1_Init(void);
void UART4_Init(void);
void USART1_BRR_Configuration(uint32_t USART_BaudRate);
void UART4_BRR_Configuration(uint32_t USART_BaudRate);

void SerialSendChar_PC(uint8_t c);
void SerialSendString_PC(char *s);
void SerialSendChar_BT(uint8_t c);
void SerialSendString_BT(char *s);

int IUART_GetSize(UINT8 id);
void IUART_GetData(UINT8 id, UINT8 *ch1);
void IUART_SendData(UINT8 id, UINT8 ch1);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

UINT8 strBuffer[2][100];
UINT8 strBufferIdx[2];
UINT8 size;
UINT8 data;
UINT8 ch;
UINT8 no_PC, no_BT, CRdata_PC, CRdata_BT;

int main(void)
{
    LCD_Init();	// LCD 구동 함수
	DelayMS(100);	// LCD구동 딜레이
    
	_GPIO_Init();
	USART1_Init();
	UART4_Init();
    Que_Clear(&txQue[1]);	// PC(USART1) QUEUE clear
    Que_Clear(&rxQue[1]);  
    Que_Clear(&txQue[2]);	// BT(UART4) QUEUE clear
    Que_Clear(&rxQue[2]);  

	GPIOG->ODR &= 0x00;	// LED0~7 Off 
	DispayTitle();	//LCD 초기화면구동 함수
	BEEP();

	strBufferIdx[0] = 0;
	strBufferIdx[1] = 0;
	while(1)
	{
		switch(KEY_Scan())
		{
            case SW0_PUSH : 		//SW0
				GPIOG->ODR |= 0x01;	// LED0 On
				SerialSendString_BT("AT+BTCANCEL"); // BT module 연결 해제
				SerialSendChar_BT(0x0D); // CR(Carriage Return : 0x0D)
				SerialSendString_PC("AT+BTCANCEL"); // BT module 연결 해제
           break;
            case SW1_PUSH : 		//SW1
				GPIOG->ODR |= 0x02;	// LED1 On
				SerialSendString_BT("AT"); // BT module 연결 확인
				SerialSendChar_BT(0x0D); // CR(Carriage Return : 0x0D)
				SerialSendString_PC("AT"); // BT module 연결 해제
            break;
            case SW2_PUSH : 		//SW2
				GPIOG->ODR |= 0x04;	// LED2 On
				SerialSendString_BT("AT+BTSCAN"); // BT module 을 외부 BT 기기(Master)에서 찾게 하기 위한 명령
				SerialSendChar_BT(0x0D); // CR(Carriage Return : 0x0D)
				SerialSendString_PC("AT+BTSCAN"); // BT module 연결 해제
            break;
            case SW3_PUSH : 		//SW3
				GPIOG->ODR |= 0x08;	// LED3 On
				SerialSendString_BT("AT+BTNAME=KPU1"); // BT module 이름 설정(변경)
				SerialSendChar_BT(0x0D); // CR(Carriage Return : 0x0D)
				SerialSendString_PC("AT+BTNAME=KPU1"); // BT module 연결 해제
            break;
            case SW4_PUSH : 		//SW4
				GPIOG->ODR |= 0x10;	// LED4 On
				SerialSendString_BT("ATZ"); // BT 명령후에 명령 인식을 위한 소프트웨어 리셋
				SerialSendChar_BT(0x0D);  // CR(Carriage Return : 0x0D)
				SerialSendString_PC("ATZ"); // BT module 연결 해제
			break;
            case SW5_PUSH : 		//SW5
				GPIOG->ODR |= 0x20;	// LED5 On
				SerialSendString_BT("AT+BTINFO?0"); // BT module 이름 확인
				SerialSendChar_BT(0x0D); // CR(Carriage Return : 0x0D)
				SerialSendString_PC("AT+BTINFO?0"); // BT module 연결 해제
  			break;
            case SW6_PUSH : 		//SW6
				GPIOG->ODR |= 0x40;	// LED6 On
				SerialSendChar_BT('B'); // 일반 데이터 전송 
				SerialSendChar_PC('B'); // PC 통신 확인을 위한 문자 전송
 			break;
            case SW7_PUSH : 		//SW7
				GPIOG->ODR |= 0x80;	// LED7 On
				SerialSendChar_PC('P'); // PC 통신 확인을 위한 문자 전송
 			break;
       
        }   
        
        
        // PC --> BT
        size = IUART_GetSize(1); // PC용 QUEUE buffer (USART1)
        if(size != 0)
        {
            for(int i = 0; i < size; i++)
            {
                IUART_GetData(1,&data);  // Get data from PC_RX_Queue
                IUART_SendData(2,data);  // Send data to BT_TX_Queue
                if(data == '\r')  // CR(carriage return: 0x0D)
                {
                    LCD_DisplayText(2,0, "                            ");
                    LCD_DisplayText(2,0, strBuffer[0]);
                  
                    strBufferIdx[0] = 0;
                    for(int j=0; j<100; j++)
                        strBuffer[0][j] = 0;
                }
                else if(data != '\n') // LF(Line Feed: 0x0A)
                {
                    strBuffer[0][strBufferIdx[0]++] = data;
                }
			}
            UART4->CR1 |= (1<<7);	// TXE interrupt Enable

        } // if(size != 0)

		// BT --> PC
        size = IUART_GetSize(2);  // BT용 QUEUE buffer
        if(size != 0)
        {
            for(int i = 0; i < size; i++)
            {
                    IUART_GetData(2,&data); // Get data from BT_RX_Queue
                    IUART_SendData(1,data); // Send data to PC_TX_Queue
                    if(data == '\r') // 0x0D
                    {
                        LCD_DisplayText(5,0, "                             ");
                        LCD_DisplayText(5,0, strBuffer[1]);
                  
                        strBufferIdx[1] = 0;
                        for(int j=0; j<100; j++)
                            strBuffer[1][j] = 0;
                    }
                    else if(data != '\n') // 0x0A
                    {
                        strBuffer[1][strBufferIdx[1]++] = data;
                    }
            }
            USART1->CR1 |= (1<<7);	// TXE interrupt Enable
		}  // if(size != 0)    
 
	}
}

void USART1_IRQHandler(void)	
{       
    UINT8 ch1;

    if ( USART1->SR & USART_SR_RXNE ) // USART_SR_RXNE= 1? RX Buffer Full?
    	// #define  USART_SR_RXNE ((uint16_t)0x0020)    //  Read Data Register Not Empty     
	{
        ch1 = (uint16_t)(USART1->DR & (uint16_t)0x01FF);	// 수신된 문자 저장
        if (CRdata_PC == 1) 
        {
            LCD_DisplayText(3,0, "                              ");
            no_PC = 0;
            CRdata_PC = 0;
        }
        if (ch1 == 0x0D) CRdata_PC++;

        if (ch1 != 0x0D)
            LCD_DisplayChar(3,no_PC++, ch1);
        // PC에서 보낸 데이터를 rxQue[1]에 저장
        Que_PutByte(&rxQue[1] , ch1);

 	} 
        // DR 을 읽으면 SR.RXNE bit(flag bit)는 clear 된다. 즉 강제 clear 할 필요없음 

    if ( USART1->SR & USART_SR_TXE ) // USART_SR_TXE= 1? TX Buffer EMPTY?
    	// #define  USART_SR_TXE ((uint16_t)0x0080)    //  WRITE Data Register Empty     
    {   
        if(Que_GetSize(&txQue[1]) != 0) 
        {
            // MCU --> PC 전송
            Que_GetByte(&txQue[1],&ch1);
            USART1->DR = ch1;
        } 
        else 
            USART1->CR1 &= ~(1<<7);  // TXE interrupt Disable
    }   	
}

void UART4_IRQHandler(void)	
{  
    UINT8 ch1;
	if ( (UART4->SR & USART_SR_RXNE) ) // USART_SR_RXNE= 1? RX Buffer Full?
    	// #define  USART_SR_RXNE ((uint16_t)0x0020)    //  Read Data Register Not Empty     
	{
//		UART4->SR &= ~USART_SR_RXNE;
		ch1 = UART4->DR ;	// 수신된 문자 저장
        
        if ((CRdata_BT == 0) && (ch1 == 0x0D)) 
        {
            LCD_DisplayText(6,0, "                               ");
            no_BT = 0;
        }
        if (ch1 == 0x0A) CRdata_BT++;
        if (CRdata_BT == 2) CRdata_BT = 0;
 
        if ( (ch1 != 0x0D) && (ch1 != 0x0A) )
            LCD_DisplayChar(6, no_BT++, ch1);
       
        // BT에서 보낸 데이터를 rxQue[2]에 저장
        Que_PutByte(&rxQue[2], ch1);  // 
        
	} 
        // DR 을 읽으면 SR.RXNE bit(flag bit)는 clear 된다. 즉 강제 clear 할 필요없음 
    if ( UART4->SR & USART_SR_TXE ) // USART_SR_TXE= 1? TX Buffer EMPTY?
    	// #define  USART_SR_TXE ((uint16_t)0x0080)    //  WRITE Data Register Empty     
    {   
        if(Que_GetSize(&txQue[2]) != 0) 
        {
            // MCU --> BT
            Que_GetByte(&txQue[2],&ch1);
            UART4->DR = (ch1 & 0x00FF);
        } 
        else 
            UART4->CR1 &= ~(1<<7);  // TXE interrupt Disable
    }  
}

int IUART_GetSize(UINT8 id)//id=1 : PC(USART1) QUEUE
                          // id=2 : BT(UART4) QUEUE
{
	return Que_GetSize(&rxQue[id]);
}
void IUART_GetData(UINT8 id, UINT8 *ch1)
{
	Que_GetByte(&rxQue[id],ch1);
}
void IUART_SendData(UINT8 id, UINT8 ch1)
{   
	Que_PutByte(&txQue[id],ch1);
}

void _GPIO_Init(void)
{
	// LED (GPIO G) 설정
    RCC->AHB1ENR	|=  0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
	GPIOG->MODER 	|=  0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
	GPIOG->OTYPER	&= ~0x00FF;	// GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
 	GPIOG->OSPEEDR 	|=  0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 
    
	// SW (GPIO H) 설정 
	RCC->AHB1ENR    |=  0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
	GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
	GPIOH->PUPDR 	&= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state

	// Buzzer (GPIO F) 설정 
    RCC->AHB1ENR	|=  0x00000020; // RCC_AHB1ENR : GPIOF(bit#5) Enable							
	GPIOF->MODER 	|=  0x00040000;	// GPIOF 9 : Output mode (0b01)						
	GPIOF->OTYPER 	&= ~0x0200;	// GPIOF 9 : Push-pull  	
 	GPIOF->OSPEEDR 	|=  0x00040000;	// GPIOF 9 : Output speed 25MHZ Medium speed 
}

void USART1_Init(void)
{
	// USART1 : TX(PA9)
	RCC->AHB1ENR	|= (1<<0);	// RCC_AHB1ENR GPIOA Enable
	GPIOA->MODER	|= (2<<2*9);	// GPIOA PIN9 Output Alternate function mode					
	GPIOA->OSPEEDR	|= (3<<2*9);	// GPIOA PIN9 Output speed (100MHz Very High speed)
	GPIOA->PUPDR 	|= (1<<2*9);	// GPIOA  PIN9 : Pull-up   
	GPIOA->AFR[1]	|= (7<<4);	// Connect GPIOA pin9 to AF7(USART1)
    
	// USART1 : RX(PA10)
	GPIOA->MODER 	|= (2<<2*10);	// GPIOA PIN10 Output Alternate function mode
	GPIOA->OSPEEDR	|= (3<<2*10);	// GPIOA PIN10 Output speed (100MHz Very High speed
	GPIOA->PUPDR 	|= (1<<2*10);	// GPIOA  PIN10 : Pull-up   
	GPIOA->AFR[1]	|= (7<<8);	// Connect GPIOA pin10 to AF7(USART1)

	RCC->APB2ENR	|= (1<<4);	// RCC_APB2ENR USART1 Enable
    
	USART1_BRR_Configuration(9600); // USART1 Baud rate Configuration
    
	USART1->CR1	&= ~(1<<12);	// USART_WordLength 8 Data bit
	USART1->CR1	&= ~(1<<10);	// NO USART_Parity

	USART1->CR1	|= (1<<2);	// 0x0004, USART_Mode_RX Enable
	USART1->CR1	|= (1<<3);	// 0x0008, USART_Mode_Tx Enable
	USART1->CR2	&= ~(3<<12);	// 0b00, USART_StopBits_1
	USART1->CR3	= 0x0000;	// No HardwareFlowControl, No DMA
    
	USART1->CR1 	|= (1<<5);	// 0x0020, RXNE interrupt Enable
	USART1->CR1 	&= ~(1<<7);	// 0x0080, TXE interrupt Disable

	NVIC->ISER[1]	|= (1<<(37-32));// Enable Interrupt USART1 (NVIC 37번)
	USART1->CR1 	|= (1<<13);	//  0x2000, USART1 Enable
}

// BlueTooth(UART4) Init
void UART4_Init(void)
{
	// UART4 : TX(PC10)
	RCC->AHB1ENR	|= (1<<2);	// RCC_AHB1ENR GPIOC Enable
	GPIOC->MODER	|= (2<<2*10);	// GPIOC PIN10 Output Alternate function mode					
	GPIOC->OSPEEDR	|= (3<<2*10);	// GPIOC PIN10 Output speed (100MHz Very High speed)
	GPIOC->PUPDR 	|= (1<<2*10);	// GPIOC  PIN10 : Pull-up   
	GPIOC->AFR[1]	|= (8<<4*(10-8));	// Connect GPIOC pin10 to AF8(UART4)
    
	// UART4 : RX(PC11)
	GPIOC->MODER 	|= (2<<2*11);	// GPIOC PIN11 Output Alternate function mode
	GPIOC->OSPEEDR	|= (3<<2*11);	// GPIOC PIN11 Output speed (100MHz Very High speed
	GPIOC->PUPDR 	|= (1<<2*11);	// GPIOC PIN11 Pull-up   
	GPIOC->AFR[1]	|= (8<<4*(11-8));	// Connect GPIOC pin11 to AF8(UART4)

	// UART4 : RST(PC13) : GPIO
	GPIOC->MODER	|= (1<<2*13);	// GPIOC PIN13 Output mode					
	GPIOC->OSPEEDR	|= (3<<2*13);	// GPIOC PIN13  Output speed (100MHz Very High speed)
	GPIOC->ODR 	    |= (1<<13);	    // BT Reset

	RCC->APB1ENR	|= (1<<19);	// RCC_APB1ENR UART4 Enable
    
	UART4_BRR_Configuration(9600); // USART Baud rate Configuration
    
	UART4->CR1	&= ~(1<<12);	// USART_WordLength 8 Data bit
	UART4->CR1	&= ~(1<<10);	// NO USART_Parity

	UART4->CR1	|= (1<<2);	// 0x0004, USART_Mode_RX Enable
	UART4->CR1	|= (1<<3);	// 0x0008, USART_Mode_Tx Enable
	UART4->CR2	&= ~(3<<12);	// 0b00, USART_StopBits_1
	UART4->CR3	= 0x0000;	// No HardwareFlowControl, No DMA
    
	UART4->CR1 	|= (1<<5);	// 0x0020, RXNE interrupt Enable
	UART4->CR1 	&= ~(1<<7);	// 0x0080, TXE interrupt Disable

	NVIC->ISER[1]	|= (1<<(52-32));// Enable Interrupt UART4 (NVIC 52번)
	UART4->CR1 	|= (1<<13);	//  0x2000, UART4 Enable
}

void SerialSendChar_PC(uint8_t Ch) // 1문자 보내기 함수
{
	// USART_SR_TXE(1<<7)=0?, TX Buffer NOT Empty? 
	// TX buffer Empty되지 않으면 계속 대기(송신 가능한 상태까지 대기)
    while((USART1->SR & USART_SR_TXE) == RESET); 
	USART1->DR = (Ch & 0x01FF);	// 전송 (최대 9bit 이므로 0x01FF과 masking)
}
void SerialSendString_PC(char *str) // 여러문자 보내기 함수
{
	while (*str != '\0') // 종결문자가 나오기 전까지 구동, 종결문자가 나온후에도 구동시 메모리 오류 발생가능성 있음.
	{
		SerialSendChar_PC(*str);// 포인터가 가르키는 곳의 데이터를 송신
		str++; 			// 포인터 수치 증가
	}
}
void SerialSendChar_BT(uint8_t Ch) // 1문자 보내기 함수
{
	// USART_SR_TXE(1<<7)=0?, TX Buffer NOT Empty? 
	// TX buffer Empty되지 않으면 계속 대기(송신 가능한 상태까지 대기)
    while((UART4->SR & USART_SR_TXE) == RESET); 
	UART4->DR = (Ch & 0x01FF);	// 전송 (최대 9bit 이므로 0x01FF과 masking)
}
void SerialSendString_BT(char *str) // 여러문자 보내기 함수
{
	while (*str != '\0') // 종결문자가 나오기 전까지 구동, 종결문자가 나온후에도 구동시 메모리 오류 발생가능성 있음.
	{
		SerialSendChar_BT(*str);// 포인터가 가르키는 곳의 데이터를 송신
		str++; 			// 포인터 수치 증가
	}
}

// USART1 Baud rate 설정
void USART1_BRR_Configuration(uint32_t USART_BaudRate)
{ 
	uint32_t tmpreg = 0x00;
	uint32_t APB2clock = 84000000;	//PCLK2_Frequency
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
	{	// Computing 'Integer part' when the oversampling mode is 16 Samples 
		integerdivider = ((25 * APB2clock) / (4 * USART_BaudRate));  // 공식에 100을 곱한 곳임(소수점 두번째자리까지 유지하기 위함)    
	}
	tmpreg = (integerdivider / 100) << 4;
  
	// Find the fractional part 
	fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

	// Implement the fractional part in the register 
	if ((USART1->CR1 & USART_CR1_OVER8) != 0)	
	{	// 8 oversampling
		tmpreg |= (((fractionaldivider * 8) + 50) / 100) & (0x07);
	}
	else	// 16 oversampling
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
	uint32_t APB1clock = 42000000;	//PCLK1_Frequency
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
	{	// Computing 'Integer part' when the oversampling mode is 16 Samples 
		integerdivider = ((25 * APB1clock) / (4 * USART_BaudRate));  // 공식에 100을 곱한 곳임(소수점 두번째자리까지 유지하기 위함)    
	}
	tmpreg = (integerdivider / 100) << 4;
  
	// Find the fractional part 
	fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

	// Implement the fractional part in the register 
	if ((UART4->CR1 & USART_CR1_OVER8) != 0)	
	{	// 8 oversampling
		tmpreg |= (((fractionaldivider * 8) + 50) / 100) & (0x07);
	}
	else	// 16 oversampling
	{
		tmpreg |= (((fractionaldivider * 16) + 50) / 100) & (0x0F);
	}

	// Write to UART BRR register
	UART4->BRR = (uint16_t)tmpreg;
}

void DispayTitle(void)
{	LCD_Clear(RGB_WHITE);
	LCD_SetFont(&Gulim7);
	LCD_SetBackColor(RGB_GREEN);	//배경색
	LCD_SetTextColor(RGB_BLACK);	//글자색
	LCD_DisplayText(0,0,"BLUETOOTH");

    LCD_DisplayText(1,0,"[PC SEND]");
    LCD_DisplayText(4,0,"[BT SEND]");

	LCD_SetBackColor(RGB_WHITE);	//글자배경색
}


void DelayMS(unsigned short wMS)
{	register unsigned short i;
	for (i=0; i<wMS; i++)
		DelayUS(1000);  // 1000us => 1ms
}
void DelayUS(unsigned short wUS)
{	volatile int Dly = (int)wUS*17;
	for(; Dly; Dly--);
}
uint8_t key_flag = 0;
uint16_t KEY_Scan(void)	// input key SW0 - SW7 
{ 
	uint16_t key;
	key = GPIOH->IDR & 0xFF00;	// any key pressed ?
	if(key == 0xFF00)		// if no key, check key off
	{  	if(key_flag == 0)
        		return key;
      		else
		{	DelayMS(10);
        		key_flag = 0;
        		return key;
        	}
    	}
  	else				// if key input, check continuous key
	{	if(key_flag != 0)	// if continuous key, treat as no key input
        		return 0xFF00;
      		else			// if new key,delay for debounce
		{	key_flag = 1;
			DelayMS(10);
 			return key;
        	}
	}
}
void BEEP(void)			// Beep for 20 ms 
{ 	GPIOF->ODR |= (1<<9);	// PF9 'H' Buzzer on
	DelayMS(20);		// Delay 20 ms
	GPIOF->ODR &= ~(1<<9);	// PF9 'L' Buzzer off
}