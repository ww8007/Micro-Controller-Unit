//////////////////////////////////////////////////////////////////////
// USART RX Interrupt 
// USART1: TX pin: PA9, RX pin: PA10 
// TX: Polling ���, RX: Interrupt ��� 
// ���ڸ� TX�� ���� PC(Hyper terminal)�� �����ϰ�, 
// PC���� ������ ���ڸ� �޾� LCD�� ǥ��
//////////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"

#define SW0_PUSH        0xFE00  //PH8
#define SW1_PUSH        0xFD00  //PH9
#define SW2_PUSH        0xFB00  //PH10
#define SW3_PUSH        0xF700  //PH11
#define SW4_PUSH        0xEF00  //PH12
#define SW5_PUSH        0xDF00  //PH13
#define SW6_PUSH        0xBF00  //PH14
#define SW7_PUSH        0x7F00  //PH15

void DisplayTitle(void);
void _GPIO_Init(void);
uint16_t KEY_Scan(void);

void USART6_Init(void);
void USART_BRR_Configuration(uint32_t USART_BaudRate);

void SerialSendChar(uint8_t c);
void SerialSendString(char *s);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

int main(void)
{
	LCD_Init();	// LCD ���� �Լ�
	DelayMS(1000);	// LCD���� ������
    
	_GPIO_Init();
	USART6_Init();
	GPIOG->ODR &= 0x00;	// LED0~7 Off 
          
	DisplayTitle();	//LCD �ʱ�ȭ�鱸�� �Լ�
	BEEP();
    
	while(1)
	{
		switch(KEY_Scan())
		{
			case SW0_PUSH : 		//SW0
				GPIOG->ODR |= 0x01;	// LED0 On
				SerialSendChar('A'); // USART1�� ���� PC�� ��HELLO! �� ���ڿ��� ����
 			break;
                }      
 	}
}

void USART6_IRQHandler(void)	
{       
	if ( (USART6->SR & USART_SR_RXNE) ) // USART_SR_RXNE= 1? RX Buffer Full?
    // #define  USART_SR_RXNE ((uint16_t)0x0020)    //  Read Data Register Not Empty     
	{
		char ch;
		ch = (uint16_t)(USART6->DR & (uint16_t)0x01FF);	// ���ŵ� ���� ����
//		SerialSendChar(ch); 		// PC�� ���ŵ� ���ڸ� Echo
        LCD_DisplayChar(1,0,ch); 	// ���ŵ� ���ڸ� LCD�� display
	} 
        // DR �� ������ SR.RXNE bit(flag bit)�� clear �ȴ�. �� clear �� �ʿ���� 
}

void _GPIO_Init(void)
{
	// LED (GPIO G) ����
    RCC->AHB1ENR	|=  0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
	GPIOG->MODER 	|=  0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
	GPIOG->OTYPER	&= ~0x00FF;	// GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
 	GPIOG->OSPEEDR 	|=  0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 
    
	// SW (GPIO H) ���� 
	RCC->AHB1ENR    |=  0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
	GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
	GPIOH->PUPDR 	&= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state

	// Buzzer (GPIO F) ���� 
    RCC->AHB1ENR	|=  0x00000020; // RCC_AHB1ENR : GPIOF(bit#5) Enable							
	GPIOF->MODER 	|=  0x00040000;	// GPIOF 9 : Output mode (0b01)						
	GPIOF->OTYPER 	&= ~0x0200;	// GPIOF 9 : Push-pull  	
 	GPIOF->OSPEEDR 	|=  0x00040000;	// GPIOF 9 : Output speed 25MHZ Medium speed 
}

void USART6_Init(void)
{
	// USART6 : TX(PC6)
	RCC->AHB1ENR	|= (1<<2);	// RCC_AHB1ENR GPIOC Enable
	GPIOC->MODER	|= (2<<2*6);	// GPIOC PIN6 Output Alternate function mode					
	GPIOC->OSPEEDR	|= (3<<2*6);	// GPIOC PIN6 Output speed (100MHz Very High speed)
	GPIOC->AFR[0]	|= (8<<4*6);	// Connect GPIOC pin6 to AF8(USART1)
    
	// USART1 : RX(PC7)
	GPIOC->MODER 	|= (2<<2*7);	// GPIOC PIN10 Output Alternate function mode
	GPIOC->OSPEEDR	|= (3<<2*7);	// GPIOC PIN10 Output speed (100MHz Very High speed
	GPIOC->AFR[0]	|= (8<<4*7);	// Connect GPIOC pin7 to AF8(USART1)

	RCC->APB2ENR	|= (1<<5);	// RCC_APB2ENR USART6 Enable
    
	USART_BRR_Configuration(19200); // USART Baud rate Configuration
    
	USART6->CR1	&= ~(1<<12);	// USART_WordLength 8 Data bit
	USART6->CR1	&= ~(1<<10);	// NO USART_Parity

	USART6->CR1	|= (1<<2);	// 0x0004, USART_Mode_RX Enable
	USART6->CR1	|= (1<<3);	// 0x0008, USART_Mode_Tx Enable
	USART6->CR2	&= ~(3<<12);	// 0b00, USART_StopBits_1
	USART6->CR3	= 0x0000;	// No HardwareFlowControl, No DMA
    
	USART6->CR1 	|= (1<<5);	// 0x0020, RXNE interrupt Enable
	NVIC->ISER[2]	|= (1<<(71-64));// Enable Interrupt USART6 (NVIC 71��)
	USART6->CR1 	|= (1<<13);	//  0x2000, USART6 Enable
}

void SerialSendChar(uint8_t Ch) // 1���� ������ �Լ�
{
	// USART_SR_TXE(1<<7)=0?, TX Buffer NOT Empty? 
	// TX buffer Empty���� ������ ��� ���(�۽� ������ ���±��� ���)
        while((USART6->SR & USART_SR_TXE) == RESET); 
	USART6->DR = (Ch & 0x01FF);	// ���� (�ִ� 9bit �̹Ƿ� 0x01FF�� masking)
}

void SerialSendString(char *str) // �������� ������ �Լ�
{
	while (*str != '\0') // ���Ṯ�ڰ� ������ ������ ����, ���Ṯ�ڰ� �����Ŀ��� ������ �޸� ���� �߻����ɼ� ����.
	{
		SerialSendChar(*str);	// �����Ͱ� ����Ű�� ���� �����͸� �۽�
		str++; 			// ������ ��ġ ����
	}
}

// Baud rate ����
void USART_BRR_Configuration(uint32_t USART_BaudRate)
{ 
	uint32_t tmpreg = 0x00;
	uint32_t APB2clock = 84000000;	//PCLK2_Frequency
	uint32_t integerdivider = 0x00;
	uint32_t fractionaldivider = 0x00;

	// Find the integer part 
	if ((USART6->CR1 & USART_CR1_OVER8) != 0) // USART_CR1_OVER8=(1<<15)
        //  #define  USART_CR1_OVER8 ((uint16_t)0x8000) // USART Oversampling by 8 enable   
    {       // USART6->CR1.OVER8 = 1 (8 oversampling)
		// Computing 'Integer part' when the oversampling mode is 8 Samples 
		integerdivider = ((25 * APB2clock) / (2 * USART_BaudRate));  // ���Ŀ� 100�� ���� ����(�Ҽ��� �ι�°�ڸ����� �����ϱ� ����)  
	}
	else  // USART1->CR1.OVER8 = 0 (16 oversampling)
	{	// Computing 'Integer part' when the oversampling mode is 16 Samples 
		integerdivider = ((25 * APB2clock) / (4 * USART_BaudRate));  // ���Ŀ� 100�� ���� ����(�Ҽ��� �ι�°�ڸ����� �����ϱ� ����)    
	}
	tmpreg = (integerdivider / 100) << 4;
  
	// Find the fractional part 
	fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

	// Implement the fractional part in the register 
	if ((USART6->CR1 & USART_CR1_OVER8) != 0)	
	{	// 8 oversampling
		tmpreg |= (((fractionaldivider * 8) + 50) / 100) & (0x07);
	}
	else	// 16 oversampling
	{
		tmpreg |= (((fractionaldivider * 16) + 50) / 100) & (0x0F);
	}

	// Write to USART BRR register
	USART6->BRR = (uint16_t)tmpreg;
}

void DisplayTitle(void)
{	LCD_Clear(RGB_WHITE);
	LCD_SetFont(&Gulim8);
	LCD_SetBackColor(RGB_GREEN);	//����
	LCD_SetTextColor(RGB_BLACK);	//���ڻ�
	LCD_DisplayText(0,0,"USART1");

	LCD_SetBackColor(RGB_WHITE);	//���ڹ���
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