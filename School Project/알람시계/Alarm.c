/////////////////////////////////////////////////////////////
// Output Compare Mode
// Timer4 CH1
// Compare Match Interrupt(CC1I)  & OC1  Pulse 
/////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"
#include "FRAM.h"
#include "Que.h"
#include "String.h"
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
void DisplayInitScreen(void);
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);
void USART1_Init(void);
void USART1_BRR_Configuration(uint32_t USART_BaudRate);
void SerialSendChar_PC(uint8_t c);
void SerialSendString_PC(char* s);
void TIMER3_Init(void);
void _ADC_Init(void);
void TIMER3_Init(void);
void TIMER7_Init(void);
void TIMER4_PWM_Init(void);
int tmode;		//시간모드
int curh = 0;	//현재시간 
int curm = 0;	//현재 분 
int fnum = 0;	//통신 첫번 째 숫자  
int snum = 0;	//통신 두번 째 숫자  
int Temp = 0;	//온도 변수
int h = 0;		//히터 변수
int c = 1;		//쿨러 변수
int calres = 0; //계산 변수
int calres10 = 0; //계산 10의 자리
int calres1 = 0;	//계산 1의 자리
int mode = 1; 	//동작 모드 
int alh = 0;	//알람 시간	
int alm = 0;	//알람 분
int getnum = 0;	//통신 인덱스 변수
int ADC_Value, Voltage;   //adc 필요변수
UINT8 ch;
UINT8 no_PC, no_BT, CRdata_PC, CRdata_BT;
int main(void)
{
	_GPIO_Init();
	_EXTI_Init();
	LCD_Init();
	DelayMS(10);
	BEEP();

	GPIOG->ODR &= 0xFF00;// : LED0~7 Off
	Fram_Init();            //FRAM 
	Fram_Status_Config();   // FRAM  S/W 
	TIMER4_PWM_Init();		// tim4 구동
	TIMER3_Init();			// tim3 구동
	_ADC_Init();			// adc 시작
	alh = Fram_Read(1208);     //FRAM  mode 
	alm = Fram_Read(1209);   //FRAM  stegpdg 
	DisplayInitScreen();	// LCD 
	TIMER7_Init();			// tim7 구동
	USART1_Init();			// usart 시작
	while (1)
	{
		// TIMER Example SW4~6  LED0  
		switch (KEY_Scan())
		{
		case SW0_PUSH: 	//SW0

			BEEP();
			alh += 1;		//알람 시간 ++
			if (alh == 10)	//10이 되면 16진수 A 쓰기 위해 변환  
			{
				alh = 17;	// 0xA
			}
			else if (alh == 23) //if 0xF -> 0x0
			{
				alh = 0;	// 0x0
			}
			DisplayInitScreen();	//화면에 써주기
			break;

		case SW1_PUSH: 	//SW1

			BEEP();
			alm += 1;
			if (alm == 10)	//10이 되면 16진수 A 쓰기 위해 변환   
			{
				alm = 17;	// 0xA
			}
			else if (alm == 23) //if 0xF -> 0x0
			{
				alm = 0; // 0x0
			}
			DisplayInitScreen(); //화면에 써주기

			break;

		case SW2_PUSH: 	//SW6

			Fram_Write(1208, alh);      // fram 써주기
			Fram_Write(1209, alm);      // fram 써주기
			BEEP();			//부저 2번 저장 알림
			BEEP();
			break;
			DisplayInitScreen();	// LCD 
		}
	}
}
void USART1_IRQHandler(void)
{
	UINT8 ch1;

	if (USART1->SR & USART_SR_RXNE) // USART_SR_RXNE= 1 RX Buffer Full
		// #define  USART_SR_RXNE ((uint16_t)0x0020)    //  Read Data Register Not Empty     
	{
		ch1 = (uint16_t)(USART1->DR & (uint16_t)0x01FF);	//   
		if (CRdata_PC == 1)
		{
			no_PC = 0;
			CRdata_PC = 0;
		}
		if (ch1 == 0x0D) CRdata_PC++;

		if (ch1 != 0x0D)	//carrige return 아닐 시
		{
			getnum++;		//통신 인덱스 확인 변수 ++
			if (getnum == 1)	//첫번 째 자리 일 시
			{
				LCD_SetTextColor(RGB_RED);	//글자색 빨강
				fnum = ch1;		//받아온 숫자 -> fnum에 저장
				LCD_DisplayChar(1, 2, fnum);	//글자 써주기
				BEEP();	//부저한번
			}
			else if (getnum == 2)	//두번 째 자리 들어 왓을 때
			{
				LCD_SetTextColor(RGB_RED);	//글자색 빨강
				snum = ch1;	//받아온 숫자 -> snum에 저장
				LCD_DisplayChar(1, 6, snum);
				BEEP();	//부저한번
			}
			else if (getnum == 3)	//3번 째 자리 들어 왔을 때
			{
				if (ch1 == '=')	//=이면 계산 동작
				{
					LCD_SetTextColor(RGB_RED);	//글자색 빨강
					fnum = fnum - 0x30;	//char -> int 변환
					snum = snum - 0x30; //char -> int 변환
					calres = fnum + snum;	//+계산
					if (calres >= 16)	//16보다 클 시 > 0xF
					{
						calres10 = 1;	//16진수 두번 째 자리 1
						calres1 = calres % 16;	//16진수 첫번 째 자리 구해주기 
						if (calres1 >= 10)	//10보다 클 시 -> A로 변환
						{
							calres1 -= 10;	//10 빼주고 +0x43 더해주기
							LCD_DisplayChar(1, 11, (calres1)+0x43);
						}
						else
						{
							LCD_DisplayChar(1, 11, (calres1)+0x30); //아닐 경우 +0x30
						}
						LCD_DisplayChar(1, 10, (calres10)+0x30);	//두번 째 자리 써주기

					}
					else	//16보다 작으면
					{
						calres10 = 0;	//10의자리는 0
						calres1 = calres;	//첫번째 자리와 계산 동일
						if (calres1 >= 10)	//위와 동일한 알고릐즘으로 생성
						{
							calres1 -= 10;
							LCD_DisplayChar(1, 11, (calres1)+0x41);
						}
						else
						{
							LCD_DisplayChar(1, 11, (calres1)+0x30);
						}
						LCD_DisplayChar(1, 10, (calres10)+0x30);
						BEEP();	//부저한번
					}
				}
				getnum = 0;	//통신 인덱스 변수 초기화
			}
			
		}

		// PC   rxQue[1] 
		Que_PutByte(&rxQue[1], ch1);

	}
	// DR   SR.RXNE bit(flag bit) clear .   clear   

	if (USART1->SR & USART_SR_TXE) // USART_SR_TXE= 1 TX Buffer EMPTY
		// #define  USART_SR_TXE ((uint16_t)0x0080)    //  WRITE Data Register Empty     
	{
		if (Que_GetSize(&txQue[1]) != 0)
		{
			// MCU --> PC 
			Que_GetByte(&txQue[1], &ch1);
			USART1->DR = ch1;
		}
		else
			USART1->CR1 &= ~(1 << 7);  // TXE interrupt Disable
	}
}

void ADC_IRQHandler(void)
{
	ADC2->SR &= ~(1 << 1);      // EOC flag clear

	ADC_Value = ADC2->DR;      // Reading ADC result
	Voltage = ADC_Value * (3.3 * 10) / 4095;   // 3.3 : 4095 =  Volatge : ADC_Value 
	// 3.3 : Voltage = Temp : 50
	Temp = 3.3 * 50 / Voltage;  //  0 ~50 
	if (Temp <= 10)                       // 50   온도 변화에 따른 히터와 쿨러 변환
	{
		h = 2;
		c = 0;
	}
	else if (Temp >= 11 && Temp <= 20)
	{
		h = 1;
		c = 0;
	}
	else if (Temp >= 21 && Temp <= 30)
	{
		h = 0;
		c = 0;
	}
	else if (Temp >= 31 && Temp <= 40)
	{
		c = 1;
		h = 0;
	}
	else if (Temp >= 41 && Temp <= 50)
	{
		c = 2;
		h = 0;
	}
	if (Temp <= 10)
	{
		LCD_SetTextColor(RGB_GREEN);	//  글자색 : Black
		LCD_DisplayChar(1, 2, '-');		// -표현
		LCD_DisplayChar(1, 3, (Temp / 10) + 0x30);	//온도 10의 자리 써주기
		LCD_DisplayChar(1, 4, (Temp % 10) + 0x30);	//온도 1의 자리 써주기
	}
	else
	{
		Temp -= 10;
		LCD_SetTextColor(RGB_GREEN);	//  : Black
		LCD_DisplayChar(1, 4, ' ');	//-삭제
		LCD_DisplayChar(1, 2, (Temp / 10) + 0x30);	//온도 10의 자리 써주기
		LCD_DisplayChar(1, 3, (Temp % 10) + 0x30);	//온도 1의 자리 써주기
	}
	LCD_SetTextColor(RGB_RED);	//  : RED	
	LCD_DisplayChar(2, 2, h + 0x30);	//히터 쿨러 써주기
	LCD_SetTextColor(RGB_BLUE);	//  : Black	
	LCD_DisplayChar(2, 6, c + 0x30);	//히터 쿨러 써주기
	
	if (mode == 3)
	{
		if (h == 1)	//히터 1
		{
			TIM4->CCR1 = 2000;		// DR: 10%
		}
		else if (h == 2) //히터 2
		{
			TIM4->CCR1 = 18000;		// DR: 90%
		}
		else if (c == 1)	//쿨러 1
		{
			TIM4->CCR1 = 2000;		// DR: 10%
		}
		else if (c == 2) // 쿨러 2
		{
			TIM4->CCR1 = 18000;		// DR: 90%
		}
	}
}

void EXTI15_10_IRQHandler(void)      // EXTI 15~10  
{
	if (EXTI->PR & 0x8000)       // EXTI11 Interrupt Pending
	{
		EXTI->PR |= 0x8000;    // Pending bit Clear
		GPIOG->ODR &= 0x0F;
		GPIOG->ODR |= 0x80;   // LED0 On
		mode += 1;	//스위치 누를 때 마다 모드 변환
		if (mode == 4)	//4번 째 눌렀을 때
			mode = 1;	//1번 째로 돌아가도록 설정
		DisplayInitScreen();
		BEEP();
	}
}
void _ADC_Init(void)
{   // ADC2: PA1
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // (1<<0) ENABLE GPIOA CLK (stm32f4xx.h )
	GPIOA->MODER |= (3 << 2 * 1);      // CONFIG GPIOA PIN0(PA1) TO ANALOG IN MODE

	RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;   // (1<<9) ENABLE ADC2 CLK (stm32f4xx.h )

	ADC->CCR &= ~(0X1F << 0);      // MULTI[4:0]: ADC_Mode_Independent
	ADC->CCR |= (1 << 16);       // 0x00010000 ADCPRE:ADC_Prescaler_Div4 (ADC MAX Clock 36MHz, 84Mhz(APB2)/4 = 21MHz)

	ADC2->CR1 &= ~(3 << 24);      // RES[1:0]=0b00 : 12bit Resolution
	ADC2->CR1 &= ~(1 << 8);      // SCAN=0 : ADC_ScanCovMode Disable
	ADC2->CR1 |= (1 << 5);      // EOCIE=1: Interrupt enable for EOC

	ADC2->CR2 &= ~(1 << 1);      // CONT=0: ADC_Continuous ConvMode Disable
	ADC2->CR2 |= (3 << 28);      // EXTEN[1:0]=0b00: ADC_ExternalTrigConvEdge_Falling
	ADC2->CR2 |= (0x07 << 24);   // TIM3_CH1 CC EVENT -> TRIGGER
	ADC2->CR2 &= ~(1 << 11);      // ALIGN=0: ADC_DataAlign_Right
	ADC2->CR2 &= ~(1 << 10);      // EOCS=0: The EOC bit is set at the end of each sequence of regular conversions

	ADC2->SQR1 &= ~(0xF << 20);   // L[3:0]=0b0000: ADC Regular channel sequece length 
	// 0b0000:1 conversion)
	ADC2->SMPR2 |= (0x7 << (3 * 1));   // ADC2_CH1 Sample Time_480Cycles (3*Channel_1)
	//Channel selection, The Conversion Sequence of PIN1(ADC2_CH0) is first, Config sequence Range is possible from 0 to 17
	ADC2->SQR3 |= (1 << 0);   // SQ1[4:0]=0b0000 : CH1
	NVIC->ISER[0] |= (1 << 18);   // Enable ADC global Interrupt
	ADC2->CR2 |= (1 << 0);      // ADON: ADC ON
}
void _GPIO_Init(void)
{
	// LED (GPIO G) 
	RCC->AHB1ENR |= 0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
	GPIOG->MODER |= 0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
	GPIOG->OTYPER &= ~0x00FF;	// GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
	GPIOG->OSPEEDR |= 0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 

	// SW (GPIO H)  
	RCC->AHB1ENR |= 0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
	GPIOH->MODER &= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
	GPIOH->PUPDR &= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state

	// Buzzer (GPIO F)  
	RCC->AHB1ENR |= 0x00000020; // RCC_AHB1ENR : GPIOF(bit#5) Enable							
	GPIOF->MODER |= 0x00040000;	// GPIOF 9 : Output mode (0b01)						
	GPIOF->OTYPER &= ~0x0200;	// GPIOF 9 : Push-pull  	
	GPIOF->OSPEEDR |= 0x00040000;	// GPIOF 9 : Output speed 25MHZ Medium speed 
}

void _EXTI_Init(void)
{
	RCC->AHB1ENR |= 0x0080;	// RCC_AHB1ENR GPIOH Enable
	RCC->APB2ENR |= 0x4000;	// Enable System Configuration Controller Clock

	GPIOH->MODER &= 0x0000FFFF;	// GPIOH PIN8~PIN15 Input mode (reset state)				 

	SYSCFG->EXTICR[3] |= 0x7000; 	// EXTI15    GPIOH  (EXTI15) (reset value: 0x0000)	

	EXTI->FTSR |= 0x008000;		// Falling Trigger Enable  (EXTI15)

	EXTI->IMR |= 0x008000;  	// EXTI15  mask (Interrupt Enable)

	NVIC->ISER[1] |= (1 << 8);   	// Enable Interrupt EXTI15 Vector table Position 
}
void TIMER3_Init(void)
{
	RCC->APB1ENR |= (1 << 1);	// RCC_APB1ENR TIMER3 Enable

	// Setting CR1 : 0x0000 
	TIM3->CR1 &= ~(1 << 4);  // DIR=0(Up counter)(reset state)
	TIM3->CR1 &= ~(1 << 1);	// UDIS=0(Update event Enabled): By one of following events
							//  Counter Overflow/Underflow, 
							//  Setting the UG bit Set,
							//  Update Generation through the slave mode controller 
							// UDIS=1 : Only Update event Enabled by  Counter Overflow/Underflow,
	TIM3->CR1 &= ~(1 << 2);	// URS=0(Update Request Source  Selection):  By one of following events
							//	Counter Overflow/Underflow, 
							// Setting the UG bit Set,
							//	Update Generation through the slave mode controller 
							// URS=1 : Only Update Interrupt generated  By  Counter Overflow/Underflow,
	TIM3->CR1 &= ~(1 << 3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM3->CR1 &= ~(1 << 7);	// ARPE=0(ARR is NOT buffered) (reset state)
	TIM3->CR1 &= ~(3 << 8); 	// CKD(Clock division)=00(reset state)
	TIM3->CR1 &= ~(3 << 5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
							// Center-aligned mode: The counter counts UP and DOWN alternatively


	// Deciding the Period
	TIM3->PSC = 8400 - 1;	// Prescaler 84,000,000Hz/8400 = 10,000 Hz (0.1ms)  (1~65536)
	TIM3->ARR = 4000 - 1;	// Auto reload  0.1ms * 4000 = 400ms

	// Clear the Counter
	TIM3->EGR |= (1 << 0);	// UG(Update generation)=1 
						// Re-initialize the counter(CNT=0) & generates an update of registers   
	TIM3->CCER |= (1 << 0);   // CC3E=1: CC3 channel Output Enable
					 // OC3(TIM5_CH3) Active:   
	TIM3->CCER &= ~(1 << 1);   // CC3P=0: CC3 channel Output Polarity (OCPolarity_High : OC3  ) 

	TIM3->CCMR1 &= ~(3 << 0); // CC3S(CC3 channel) = '0b00' : Output 
	TIM3->CCMR1 &= ~(1 << 3); // OC3P=0: Output Compare 3 preload disable
	TIM3->CCMR1 |= (3 << 4);   // OC3M=0b011: Output Compare 3 Mode : toggle
						// OC3REF toggles when CNT = CCR3

	TIM3->CCR1 = 1000;   // TIM3 CCR1 TIM3_Pulse
	// Setting an UI(UEV) Interrupt 
	NVIC->ISER[0] |= (1 << 29); // Enable Timer3 global Interrupt
	TIM3->DIER |= (1 << 0);	// Enable the Tim3 Update interrupt

	TIM3->CR1 |= (1 << 0);	// Enable the Tim3 Counter (clock enable)   
}


void TIMER7_Init(void)
{
	RCC->APB1ENR |= (1 << 5);	// RCC_APB1ENR TIMER7 Enable

	// Setting CR1 : 0x0000 
	TIM7->CR1 &= ~(1 << 4);  // DIR=0(Up counter)(reset state)
	TIM7->CR1 &= ~(1 << 1);	// UDIS=0(Update event Enabled): By one of following events
							//  Counter Overflow/Underflow, 
							//  Setting the UG bit Set,
							//  Update Generation through the slave mode controller 
							// UDIS=1 : Only Update event Enabled by  Counter Overflow/Underflow,
	TIM7->CR1 &= ~(1 << 2);	// URS=0(Update Request Source  Selection):  By one of following events
							//	Counter Overflow/Underflow, 
							// Setting the UG bit Set,
							//	Update Generation through the slave mode controller 
							// URS=1 : Only Update Interrupt generated  By  Counter Overflow/Underflow,
	TIM7->CR1 &= ~(1 << 3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM7->CR1 &= ~(1 << 7);	// ARPE=0(ARR is NOT buffered) (reset state)
	TIM7->CR1 &= ~(3 << 8); 	// CKD(Clock division)=00(reset state)
	TIM7->CR1 &= ~(3 << 5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
							// Center-aligned mode: The counter counts UP and DOWN alternatively


	// Deciding the Period
	TIM7->PSC = 8400 - 1;	// Prescaler=84, 84MHz/8400 = 1KHz (0.1ms)
	TIM7->ARR = 10000 - 1;	// Auto reload  : 0.1ms * 10K = 1s(period) :    

	// Clear the Counter
	TIM7->EGR |= (1 << 0);	// UG(Update generation)=1 
						// Re-initialize the counter(CNT=0) & generates an update of registers   
	TIM7->EGR |= (1 << 1); 	// CC1G(C/C 1 gerneration)						
	TIM7->DIER |= (1 << 1);	// TIMER 3 CC1 INTERUPT EN
	// Setting an UI(UEV) Interrupt 
	NVIC->ISER[1] |= (1 << 23); // Enable Timer7 global Interrupt
	TIM7->DIER |= (1 << 0);	// Enable the TIM7 Update interrupt

	TIM7->CR1 |= (1 << 0);	// Enable the TIM7 Counter (clock enable)   
}

void TIM7_IRQHandler(void)  	// 1s Interrupt
{

	TIM7->SR &= ~(1 << 0);	// Interrupt flag Clear
	curm++;
	if (curm == 23)		//분이 17이 되면 0xF
	{
		curh++;		//시간 자리수 up
		curm = 0;	//분은 0으로 초기화
	}
	else if (curm == 10)	//분이 10되면
	{
		curm = 17;	//A로 변환
	}
	if (curh == 23)	//분이 17이 되면
	{
		curh = 0;	//분 0으로 초기화
	}
	else if (curh == 10)	//분이 10이면 
	{
		curh = 17;	//분 A
	}
	if (curh == alh)	//알람 설정 시간과 분 같을 시
	{
		if (curm == alm)
		{
			BEEP();	//부저 3번
			BEEP();
			BEEP();	
		}
	}
	LCD_SetTextColor(RGB_BLUE);	//  : Black
	LCD_DisplayChar(0, 15, curh + 0x30); //시간 써주기
	LCD_DisplayChar(0, 16, ':');	// :
	LCD_DisplayChar(0, 17, curm + 0x30);	//분 써주기
}
void TIMER4_PWM_Init(void)
{
	// TIM4 CH1 : PB6 (164 )
	// Clock Enable : GPIOB & TIMER4
	RCC->AHB1ENR |= (1 << 1);	// GPIOB CLOCK Enable
	RCC->APB1ENR |= (1 << 2);	// TIMER4 CLOCK Enable 

// PB8  Alternate function(TIM4_CH3)   : PWM 
	GPIOB->MODER |= (2 << 12);	// 0x00020000 PB8 Output Alternate function mode					
	GPIOB->OSPEEDR |= (3 << 12);	// 0x00030000 PB8 Output speed (100MHz High speed)
	GPIOB->OTYPER &= ~(1 << 6);	// PB8 Output type push-pull (reset state)
	GPIOB->AFR[0] |= (2 << 24);	// 0x00000002 (AFR[1].(3~0)=0b0010): Connect TIM4 pins(PB8) to AF2(TIM3..5)

// TIM4 Channel 3 : PWM 1 mode
	// Assign 'PWM Pulse Period'
	TIM4->PSC = 8400 - 1;	// Prescaler 84,000,000Hz/8400 = 10,000 Hz(0.1ms)  (1~65536)
	TIM4->ARR = 20000 - 1;	// Auto reload  (0.1ms * 20000 = 2s : PWM Period)

	// Setting CR1 : 0x0000 (Up counting)
	TIM4->CR1 &= ~(1 << 4);	// DIR=0(Up counter)(reset state)
	TIM4->CR1 &= ~(1 << 1);	// UDIS=0(Update event Enabled)
	TIM4->CR1 &= ~(1 << 2);	// URS=0(Update event source Selection)g events
	TIM4->CR1 &= ~(1 << 3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM4->CR1 |= (1 << 7);	// ARPE=1(ARR is buffered): ARR Preload Enable 
	TIM4->CR1 &= ~(3 << 8); 	// CKD(Clock division)=00(reset state)
	TIM4->CR1 &= ~(3 << 5); 	// CMS(Center-aligned mode Sel)=00 : Edge-aligned mode(reset state)
	TIM4->EGR |= (1 << 0);	// UG(Update generation)=1 			
	// Define the corresponding pin by 'Output'  
	// CCER(Capture/Compare Enable Register) : Enable "Channel 3" 
	TIM4->CCER |= (1 << 0);	// CC3E=1: OC3(TIM4_CH3) Active(Capture/Compare 3 output enable)
					// (167)  
	TIM4->CCER &= ~(1 << 1);	// CC3P=0: CC3 Output Polarity (OCPolarity_High : OC3  )

	// Duty Ratio 
	TIM4->CCR3 = 2000;		// CCR3 value

	// 'Mode' Selection : Output mode, PWM 1
	// CCMR2(Capture/Compare Mode Register 2) : Setting the MODE of Ch3 or Ch4
	TIM4->CCMR2 &= ~(3 << 0); // CC3S(CC3 channel)= '0b00' : Output 
	TIM4->CCMR2 |= (1 << 3); 	// OC3PE=1: Output Compare 3 preload Enable
	TIM4->CCMR2 |= (6 << 4);	// OC3M=0b110: Output compare 3 mode: PWM 1 mode
	TIM4->CCMR2 |= (1 << 7);	// OC3CE=1: Output compare 3 Clear enable

	//Counter TIM5 enable
	TIM4->CR1 |= (1 << 0);	// CEN: Counter TIM4 enable
}
void TIM4_IRQHandler(void)      //RESET: 0
{
	if ((TIM4->SR & 0x01) != RESET)	// Update interrupt flag (10ms)
	{
		TIM4->SR &= ~(1 << 0);	// Update Interrupt Claer
		GPIOG->ODR |= 0x01;	// LED0 On
		
	}


}
void USART1_Init(void)
{
	// USART1 : TX(PA9)
	RCC->AHB1ENR |= (1 << 0);	// RCC_AHB1ENR GPIOA Enable
	GPIOA->MODER |= (2 << 2 * 9);	// GPIOA PIN9 Output Alternate function mode					
	GPIOA->OSPEEDR |= (3 << 2 * 9);	// GPIOA PIN9 Output speed (100MHz Very High speed)
	GPIOA->PUPDR |= (1 << 2 * 9);	// GPIOA  PIN9 : Pull-up   
	GPIOA->AFR[1] |= (7 << 4);	// Connect GPIOA pin9 to AF7(USART1)

	// USART1 : RX(PA10)
	GPIOA->MODER |= (2 << 2 * 10);	// GPIOA PIN10 Output Alternate function mode
	GPIOA->OSPEEDR |= (3 << 2 * 10);	// GPIOA PIN10 Output speed (100MHz Very High speed
	GPIOA->PUPDR |= (1 << 2 * 10);	// GPIOA  PIN10 : Pull-up   
	GPIOA->AFR[1] |= (7 << 8);	// Connect GPIOA pin10 to AF7(USART1)

	RCC->APB2ENR |= (1 << 4);	// RCC_APB2ENR USART1 Enable

	USART1_BRR_Configuration(9600); // USART1 Baud rate Configuration

	USART1->CR1 &= ~(1 << 12);	// USART_WordLength 8 Data bit
	USART1->CR1 &= ~(1 << 10);	// NO USART_Parity

	USART1->CR1 |= (1 << 2);	// 0x0004, USART_Mode_RX Enable
	USART1->CR1 |= (1 << 3);	// 0x0008, USART_Mode_Tx Enable
	USART1->CR2 &= ~(3 << 12);	// 0b00, USART_StopBits_1
	USART1->CR3 = 0x0000;	// No HardwareFlowControl, No DMA

	USART1->CR1 |= (1 << 5);	// 0x0020, RXNE interrupt Enable
	USART1->CR1 &= ~(1 << 7);	// 0x0080, TXE interrupt Disable

	NVIC->ISER[1] |= (1 << (37 - 32));// Enable Interrupt USART1 (NVIC 37)
	USART1->CR1 |= (1 << 13);	//  0x2000, USART1 Enable
}
// USART1 Baud rate 
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
		integerdivider = ((25 * APB2clock) / (2 * USART_BaudRate));  //  100  (   )  
	}
	else  // USART1->CR1.OVER8 = 0 (16 oversampling)
	{	// Computing 'Integer part' when the oversampling mode is 16 Samples 
		integerdivider = ((25 * APB2clock) / (4 * USART_BaudRate));  //  100  (   )    
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
{
	GPIOF->ODR |= 0x0200;	// PF9 'H' Buzzer on
	DelayMS(20);		// Delay 20 ms
	GPIOF->ODR &= ~0x0200;	// PF9 'L' Buzzer off
}

void DelayMS(unsigned short wMS)
{
	register unsigned short i;
	for (i = 0; i < wMS; i++)
		DelayUS(1000);   // 1000us => 1ms
}
void SerialSendChar_PC(uint8_t Ch) // 1  
{
	// USART_SR_TXE(1<<7)=0, TX Buffer NOT Empty 
	// TX buffer Empty   (   )
	while ((USART1->SR & USART_SR_TXE) == RESET);
	USART1->DR = (Ch & 0x01FF);	//  ( 9bit  0x01FF masking)
}
void SerialSendString_PC(char* str) //   
{
	while (*str != '\0') //    ,       .
	{
		SerialSendChar_PC(*str);//     
		str++; 			//   
	}
}
void DelayUS(unsigned short wUS)
{
	volatile int Dly = (int)wUS * 17;
	for (; Dly; Dly--);
}

void DisplayInitScreen(void)
{
	LCD_Clear(RGB_WHITE);		//  바탕 흰색 초기화
	LCD_SetFont(&Gulim8);		//  굴림  8
	LCD_SetBackColor(RGB_WHITE);	//  : WHITE
	LCD_SetTextColor(RGB_BLACK);	//  : Black
	LCD_SetPenColor(RGB_GREEN);		//  GREEN
	LCD_DrawRectangle(1, 1, 158, 60);	//사각형 그려주기  

	if (mode == 1)	//알람모드
	{
		LCD_DisplayText(0, 0, "1.ALARM");
		LCD_DisplayText(1, 0, "ALARM");
		LCD_SetTextColor(RGB_RED);	//  글자색 : RED
		LCD_DisplayChar(1, 6, alh + 0x30);	//알람 시간 써주기
		LCD_DisplayChar(1, 7, ':');	//:
		LCD_DisplayChar(1, 8, alm + 0x30); // 알람 분 써주기
		LCD_SetTextColor(RGB_BLUE);	//  글자색 : BLUE
		LCD_DisplayChar(0, 15, curh + 0x30);	//현재 시 써주기
		LCD_DisplayChar(0, 16, ':');	// :
		LCD_DisplayChar(0, 17, curm + 0x30); //현재 분 써주기
		LCD_SetTextColor(RGB_BLUE);	//  : Black
	}
	else if (mode == 2)	//계산 모드
	{
		LCD_DisplayText(0, 0, "2.Calculator");
		LCD_SetTextColor(RGB_RED);	//  : RED
		LCD_DisplayChar(1, 2, fnum + 0x30);
		LCD_DisplayChar(1, 6, snum + 0x30);
		LCD_DisplayChar(1, 10, (calres / 10) + 0x30);
		LCD_DisplayChar(1, 11, (calres % 10) + 0x30);
		LCD_SetTextColor(RGB_BLACK);	//  : Black	
		LCD_DisplayChar(1, 4, '+');
		LCD_DisplayChar(1, 8, '=');
		LCD_SetTextColor(RGB_BLUE);	//  : Black
		LCD_DisplayChar(0, 15, curh + 0x30);
		LCD_DisplayChar(0, 16, ':');
		LCD_DisplayChar(0, 17, curm + 0x30);
	}
	else if (mode == 3) //온도계 모드
	{
		LCD_SetTextColor(RGB_BLACK);	// 글자색 : Black	
		LCD_DisplayText(0, 0, "3.Thermostat");
		LCD_DisplayText(1, 0, "T:");
		LCD_DisplayText(2, 0, "H:");
		LCD_DisplayChar(2, 4, 'C');
		LCD_DisplayChar(2, 5, ':');

		LCD_SetTextColor(RGB_GREEN);	// 글자색 : Black	
		LCD_DisplayChar(1, 2, (Temp / 10) + 0x30);	//온도 써주기
		LCD_DisplayChar(1, 3, (Temp & 10) + 0x30);

		LCD_SetTextColor(RGB_RED);	//  글자색 : RED	
		LCD_DisplayChar(2, 2, h + 0x30);

		LCD_SetTextColor(RGB_BLUE);	// 글자색 : Black	
		LCD_DisplayChar(2, 6, c + 0x30);
		LCD_SetTextColor(RGB_BLUE);	// 글자색 : Black
		LCD_DisplayChar(0, 15, curh + 0x30);
		LCD_DisplayChar(0, 16, ':');
		LCD_DisplayChar(0, 17, curm + 0x30);
	}
}

uint8_t key_flag = 0;
uint16_t KEY_Scan(void)	// input key SW0 - SW7 
{
	uint16_t key;
	key = GPIOH->IDR & 0xFF00;	// any key pressed 
	if (key == 0xFF00)		// if no key, check key off
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
	else				// if key input, check continuous key
	{
		if (key_flag != 0)	// if continuous key, treat as no key input
			return 0xFF00;
		else			// if new key,delay for debounce
		{
			key_flag = 1;
			DelayMS(10);
			return key;
		}
	}
}

