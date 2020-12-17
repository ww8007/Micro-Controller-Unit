/////////////////////////////////////////////////////////////
// Output Compare Mode
// Timer4 CH1
// Compare Match Interrupt(CC1I) �߻� & OC1�� ���� Pulse ���
/////////////////////////////////////////////////////////////
include "stm32f4xx.h"
include "FRAM.h"
include "GLCD.h"
include "Que.h"
#define SW0_PUSH        0xFE00  //PH8
#define SW1_PUSH        0xFD00  //PH9
#define SW2_PUSH        0xFB00  //PH10
#define SW3_PUSH        0xF700  //PH11
#define SW4_PUSH        0xEF00  //PH12
#define SW5_PUSH        0xDF00  //PH13
#define SW6_PUSH        0xBF00  //PH14
#define SW7_PUSH        0x7F00  //PH15

void _GPIO_Init(void);
void _EXTI_Init(void);
uint16_t KEY_Scan(void);
void TIMER4_OC_Init(void);	// General-purpose Timer 4 (Output Compare mode
void DisplayInitScreen(void);
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);
void USART1_Init(void);
void USART1_BRR_Configuration(uint32_t USART_BaudRate);
void SerialSendChar_PC(uint8_t c);
void SerialSendString_PC(char *s);

int curh = 0;	//���� �ð�
int curm = 0;	//���� ��
int fnum = 0;	//ù ��° ����
int snum = 0;	//�� ��° ����
int temp = 0;	//�µ�
int h = 0;		//����
int c = 1;		//��
int calres = 0; //�������
int mode = 1; 	//�ʱ� �˶����
int alh = 46;
int alm = 41;

int main(void)
{
	_GPIO_Init();
	_EXTI_Init();
	LCD_Init();	
	DelayMS(10);
	//BEEP();
   	
	GPIOG->ODR &= 0xFF00;// �ʱⰪ: LED0~7 Off
	Fram_Init();            //FRAM ����
    Fram_Status_Config();   // FRAM �ʱ�ȭ S/W �ʱ�ȭ
	TIMER4_OC_Init();	// TIM4 Init (Output Compare mode: UI & CCI �߻�, )                  
	

	alh = Fram_Read(1208);     //FRAM ���� mode �о��
    alm = Fram_Read(1209);   //FRAM ���� stegpdg �о��
	DisplayInitScreen();	// LCD �ʱ�ȭ��
	while(1)
	{
		// TIMER Example SW4~6�� ����Ͽ� LED0�� ��� ����
		switch(KEY_Scan())
		{
			case SW0_PUSH  : 	//SW0
                GPIOG->ODR &= 0xFF;	// LED4~7 OFF
                GPIOG->ODR |= 0x10;	// LED4 ON
				//BEEP();
				alh += 1;
				if (alh == 46)	//F �Ѿ ��
				{
					alh = 30;	//0���� �ʱ�ȭ
				}
				DisplayInitScreen();

 			break;

			case SW1_PUSH  : 	//SW1
                GPIOG->ODR &= 0xFF;	// LED4~7 OFF
                GPIOG->ODR |= 0x20;	// LED5 ON
				//BEEP();
				alm += 1;
				if (alm == 46)	//F �Ѿ ��
				{
					alm = 30;	//0���� �ʱ�ȭ
				}
				DisplayInitScreen();

			break;

            case SW2_PUSH  : 	//SW6
                GPIOG->ODR &= 0x0F;	// LED4~7 OFF
                GPIOG->ODR |= 0x40;	// LED6 ON
				Fram_Write(1208, alh);      // fram ����
				Fram_Write(1209, alm);      // fram ����
				//BEEP();
				//BEEP();

		
			break;	
        }
	}
}

void TIMER4_OC_Init(void)
{
// PD12: TIM4_CH1
// PD12�� ��¼����ϰ� Alternate function(TIM4_CH1)���� ��� ����
	RCC->AHB1ENR	|= (1<<3);	// 0x08, RCC_AHB1ENR GPIOD Enable : AHB1ENR.3

	GPIOD->MODER    |= (2<<24);	// 0x02000000(MODER.(25,24)=0b10), GPIOD PIN12 Output Alternate function mode 					
	GPIOD->OSPEEDR 	|= (3<<24);	// 0x03000000(OSPEEDER.(25,24)=0b11), GPIOD PIN12 Output speed (100MHz High speed)
	GPIOD->OTYPER	&= ~(1<<12);	// ~0x1000, GPIOD PIN12 Output type push-pull (reset state)
	GPIOD->PUPDR    |= (1<<24); 	// 0x01000000, GPIOD PIN12 Pull-up
  					// PD12 ==> TIM4_CH1
	GPIOD->AFR[1]	|= 0x00020000;  // (AFR[1].(19~16)=0b0010): Connect TIM4 pins(PD12) to AF2(TIM3..5)
 
// Timerbase ����
	RCC->APB1ENR |= (1<<2);	// 0x04, RCC_APB1ENR TIMER4 Enable

	// Setting CR1 : 0x0000 
	TIM4->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
	TIM4->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled): By one of following events
                            //  Counter Overflow/Underflow, 
                            //  Setting the UG bit Set,
                            //  Update Generation through the slave mode controller 
                            // UDIS=1 : Only Update event Enabled by  Counter Overflow/Underflow,
	TIM4->CR1 &= ~(1<<2);	// URS=0(Update event source Selection): one of following events
                            //	Counter Overflow/Underflow, 
                            // Setting the UG bit Set,
                            //	Update Generation through the slave mode controller 
                            // URS=1 : Only Update Interrupt generated  By  Counter Overflow/Underflow,
	TIM4->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM4->CR1 &= ~(1<<7);	// ARPE=0(ARR is NOT buffered) (reset state)
	TIM4->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM4->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
				// Center-aligned mode: The counter counts Up and DOWN alternatively

	// Setting the Period
	TIM4->PSC = 8400-1;	// Prescaler=84, 84MHz/8400 = 1KHz (0.1ms)
	TIM4->ARR = 10000-1;	// Auto reload  : 0.1ms * 10K = 1s(period) : ���ͷ�Ʈ�ֱ⳪ ��½�ȣ�� �ֱ� ����

	// Update(Clear) the Counter
	TIM4->EGR |= (1<<0);    // UG: Update generation    

// Output Compare ����
	// CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2
	TIM4->CCMR1 &= ~(3<<0); // CC1S(CC1 channel) = '0b00' : Output 
	TIM4->CCMR1 &= ~(1<<2); // OC1FE=0: Output Compare 1 Fast disable 
	TIM4->CCMR1 &= ~(1<<3); // OC1PE=0: Output Compare 1 preload disable(CCR1�� �������� ���ο� ���� loading ����) 
	TIM4->CCMR1 |= (3<<4);	// OC1M=0b011 (Output Compare 1 Mode : toggle)
				// OC1REF toggles when CNT = CCR1
				
	// CCER(Capture/Compare Enable Register) : Enable "Channel 1" 
	TIM4->CCER |= (1<<0);	// CC1E=1: CC1 channel Output Enable
				// OC1(TIM4_CH1) Active: �ش���(100��)�� ���� ��ȣ���
	TIM4->CCER &= ~(1<<1);	// CC1P=0: CC1 channel Output Polarity (OCPolarity_High : OC1���� �������� ���)  

	// CC1I(CC ���ͷ�Ʈ) ���ͷ�Ʈ �߻��ð� �Ǵ� ��ȣ��ȭ(���)�ñ� ����: ��ȣ�� ����(phase) ����
	// ���ͷ�Ʈ �߻��ð�(10000 �޽�)�� 10%(1000) �ð����� compare match �߻�
	TIM4->CCR1 = 1000;	// TIM4 CCR1 TIM4_Pulse

	TIM4->DIER |= (1<<0);	// UIE: Enable Tim4 Update interrupt
	TIM4->DIER |= (1<<1);	// CC1IE: Enable the Tim4 CC1 interrupt

	NVIC->ISER[0] 	|= (1<<30);	// Enable Timer4 global Interrupt on NVIC

	TIM4->CR1 |= (1<<0);	// CEN: Enable the Tim4 Counter  					
}

void TIM4_IRQHandler(void)      //RESET: 0
{
	if ((TIM4->SR & 0x01) != RESET)	// Update interrupt flag (10ms)
	{
		TIM4->SR &= ~(1<<0);	// Update Interrupt Claer
		GPIOG->ODR |= 0x01;	// LED0 On
	}
    
	if((TIM4->SR & 0x02) != RESET)	// Capture/Compare 1 interrupt flag
	{
		TIM4->SR &= ~(1<<1);	// CC 1 Interrupt Claer
		GPIOG->ODR &= ~0x01;	// LED0 Off
	}
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

void _EXTI_Init(void)
{
    RCC->AHB1ENR 	|= 0x0080;	// RCC_AHB1ENR GPIOH Enable
	RCC->APB2ENR 	|= 0x4000;	// Enable System Configuration Controller Clock
	
	GPIOH->MODER 	&= 0x0000FFFF;	// GPIOH PIN8~PIN15 Input mode (reset state)				 
	
	SYSCFG->EXTICR[3] |= 0x7000; 	// EXTI15�� ���� �ҽ� �Է��� GPIOH�� ���� (EXTI15) (reset value: 0x0000)	
	
	EXTI->FTSR |= 0x000100;		// Falling Trigger Enable  (EXTI15)
    EXTI->RTSR |= 0x000200;		// Rising Trigger  Enable  (EXTI9) 
    EXTI->IMR  |= 0x000300;  	// EXTI15 ���ͷ�Ʈ mask (Interrupt Enable)
		
	NVIC->ISER[1] |= (1<<8);   	// Enable Interrupt EXTI15 Vector table Position ����
}

void EXTI15_10_IRQHandler(void)      // EXTI 15~10 ���ͷ�Ʈ �ڵ鷯
{
    if (EXTI->PR & 0x8000)       // EXTI11 Interrupt Pending?
    {
		EXTI->PR |= 0x8000;    // Pending bit Clear
        GPIOG->ODR &= 0x0F;
        GPIOG->ODR |= 0x80;   // LED0 On
        mode +=1;
		if (mode ==4)
				mode =1;
		DisplayInitScreen();
		//BEEP();
    }
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

	NVIC->ISER[1]	|= (1<<(37-32));// Enable Interrupt USART1 (NVIC 37��)
	USART1->CR1 	|= (1<<13);	//  0x2000, USART1 Enable
}
// USART1 Baud rate ����
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
void BEEP(void)			// Beep for 20 ms 
{ 	GPIOF->ODR |= 0x0200;	// PF9 'H' Buzzer on
	DelayMS(20);		// Delay 20 ms
	GPIOF->ODR &= ~0x0200;	// PF9 'L' Buzzer off
}

void DelayMS(unsigned short wMS)
{
	register unsigned short i;
	for (i=0; i<wMS; i++)
		DelayUS(1000);   // 1000us => 1ms
}
void SerialSendChar_PC(uint8_t Ch) // 1���� ������ �Լ�
{
	// USART_SR_TXE(1<<7)=0?, TX Buffer NOT Empty? 
	// TX buffer Empty���� ������ ��� ���(�۽� ������ ���±��� ���)
    while((USART1->SR & USART_SR_TXE) == RESET); 
	USART1->DR = (Ch & 0x01FF);	// ���� (�ִ� 9bit �̹Ƿ� 0x01FF�� masking)
}
void SerialSendString_PC(char *str) // �������� ������ �Լ�
{
	while (*str != '\0') // ���Ṯ�ڰ� ������ ������ ����, ���Ṯ�ڰ� �����Ŀ��� ������ �޸� ���� �߻����ɼ� ����.
	{
		SerialSendChar_PC(*str);// �����Ͱ� ����Ű�� ���� �����͸� �۽�
		str++; 			// ������ ��ġ ����
	}
}
void DelayUS(unsigned short wUS)
{
	volatile int Dly = (int)wUS*17;
		for(; Dly; Dly--);
}

void DisplayInitScreen(void)
{
	LCD_Clear(RGB_WHITE);		// ȭ�� Ŭ����
	LCD_SetFont(&Gulim8);		// ��Ʈ : ���� 8
	LCD_SetBackColor(RGB_WHITE);	// ���ڹ��� : WHITE
	LCD_SetTextColor(RGB_BLACK);	// ���ڻ� : Black
	LCD_SetPenColor(RGB_GREEN);		// ��� GREEN
	LCD_DrawRectangle(1, 1, 158, 60);	// �簢�� �׷��ֱ�

	if (mode == 1)
	{
		LCD_DisplayText(0,0,"1.ALARM");  
		LCD_DisplayText(1, 0, "ALARM");
		LCD_SetTextColor(RGB_RED);	// ���ڻ� : RED
		LCD_DisplayChar(1, 6, alh);
		LCD_DisplayChar(1, 7, ':');
		LCD_DisplayChar(1, 8, alm);
		LCD_SetTextColor(RGB_BLUE);	// ���ڻ� : BLUE
		LCD_DisplayChar(0, 15, curh);
		LCD_DisplayChar(0, 16, ':');
		LCD_DisplayChar(0, 17, curm);
	}
	else if (mode ==2)
	{
		LCD_DisplayText(0,0,"2.Calculator");  
		LCD_SetTextColor(RGB_RED);	// ���ڻ� : RED
		LCD_DisplayChar(1, 2, fnum+0x30);
		LCD_DisplayChar(1, 6, snum+0x30);
		LCD_DisplayChar(1, 10, (calres/10)+0x30);
		LCD_DisplayChar(1, 11, (calres%10)+0x30);
		LCD_SetTextColor(RGB_BLACK);	// ���ڻ� : Black	
		LCD_DisplayChar(1, 4, ':');
		LCD_DisplayChar(1, 8, '=');
		
	}
	else if (mode ==3)
	{
		LCD_SetTextColor(RGB_BLACK);	// ���ڻ� : Black	
		LCD_DisplayText(0,0,"3.Thermostat");
		LCD_DisplayText(1,0,"T:");
		LCD_DisplayText(2,0,"H:");
		LCD_DisplayChar(2, 4, 'C');
		LCD_DisplayChar(2, 5, ':');

		LCD_SetTextColor(RGB_GREEN);	// ���ڻ� : Black	
		LCD_DisplayChar(1, 2, (tmep/10)+0x30);
		LCD_DisplayChar(1, 3, (tmep&10)+0x30);

		LCD_SetTextColor(RGB_RED);	// ���ڻ� : Black	
		LCD_DisplayChar(2, 2, h+0x30);

		LCD_SetTextColor(RGB_BLUE);	// ���ڻ� : Black	
		LCD_DisplayChar(2, 6, c+0x30);
	}
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
		{   key_flag = 1;
            DelayMS(10);
            return key;
        }
	}
}

