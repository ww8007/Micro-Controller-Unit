# ADC
    Analog to Digital Converter
--------------------------------------------
### 구조
    1. Sensor-ADC(Analog to Digital Converter)
        실제 물리량 -> 전기신호(Analog) -> 전기신호(Digital)
    2. Communication
        실제 물리량 -> 전기신호(Analog) -> 전기신호(Digital)
                      Sensor Module       MCU

### 변환
    고려요소 1. 전압범위(ex -5~5V), 표현크기(8bit, 10bit)
    * 분해능 : 입력값 얼마나 세밀하게 변환가능
        ex 0~5v 분해능 10bit -> 5 / 1024 = 0.005V

### STM32F4 ADC
    1. SAR(Successive-apporoximation-register) ADC
        연속        근사           레지스터
    2. 12bit Resolution(분해능)
    3. Ref 전압 Max 3.3V
    4. 19개 아날로그 신호 입력채널 <- 동시다발적 아닌 한번에 하나 처리
        외부입력 16개
        내부입력 3개
        1. TempSensor - ADC_IN16(내부) 온도측정
        2. Vreflntx   - ADC_IN17(내부) Internal Refe
        3. Vbat       - ADC_IN18(내부) Vbat 전압 측정
        * MCU 총 외부입력 체널 24개 ADC 모듈에서 이 중 16개 사용
    5. 종료시 **인터럽트** 발생 <- 안써도 된다(Polling 으로 사용)
    6. 여러 모드
        1. 단일(single) 2. 연속(continuous), 3.스캔(scan)
    7. 전압 요구사항(2.4V ~ 3.6V)
    8. 전압 Input 범위 Vref-Vssa ~ Vref+(Vdda)
    9. 3개의 ADC모듈 (ADC1, ADC2, ADC3)
    10. 각 ADCx 내부의 AD 변환기(모듈) 구성 : 2개의 ADC 엔진
    
### 저장
    2개의 16비트 데이터 레지스터에 저장

### 시작신호 입력
    1. 기본 : ADCx->CR2.SWSTART=1 (on)
    2. 외부 : 외부인터럽트 입력(스위치)
    3. 내부 : Timer

### 완료시점 파악
    이벤트가 발생해 Flags에서 기록 
    EOC Flags -> 1 , NVIC set
    1. Polling - 계속 파악
    2. INT(인터럽트) - 인터럽트 출력
![image](https://user-images.githubusercontent.com/54137044/102693367-79ee8a80-425d-11eb-8680-4d8b4c6a4fd6.png)

### 주요기능
    1. ADC on-off control
        power on         ->   ADCx->CR2.ADON = 1;
        power off        ->   ADCx->CR2.ADOn = 0;
        Conversion Start ->   ADCx->CR2.SWSTART = 1;
    2. ADC clock
        1. 아날로그 : ADCCLK(ADC1,2,3)
            APB2 clock으로부터 분주 
        2. 디지털   : RCC->APB2ENR EN
    3. Channel Selection
        외부입력 받는 16개 채널선택과 순서 지정
        ADCx->SQR1.L[3:0] bits : 변환채널 수
        ADCx->SQRy.SQz[4:0] bits : 변환순서 설정 (y=1~3, z=1~16)
    4. 단일, 연속(SW, 또는 외부 인터럽트로 시작)
        1. 단일   
            ADCx->CR2.CONT = 0;
            ADCx->CR2.SWSTART = 1;
        2. 연속(자동시작) - 인터럽트로 하는게 좋음
            ADCx->CR2.CONT = 1;
            반드시 EOCS = 0
        체크는 Polling 또는 INT
            Polling 시 자동으로 EOC flags 읽어옴
            INT 시 NVIC 설정
    5. 외부트리거 신호 탐지
        ADCx->CR2.EXTEN[1:0] = 0x0;
    6. 외부트리거에 의한 변환 시작
        ADCx->CR2.EXTSEL[3:0]
![image](https://user-images.githubusercontent.com/54137044/102693741-321d3280-4260-11eb-8532-12037545f086.png)    
### 주요 레지스터
    1. ADC_SR  : EOC(END OF CONVERSION)
    2. ADC_CR1
        [25:24] : 12BIT, 10BIT , 8BIT
        [8]     : SCANMODE
        [5]     : EOC mask interrupt enable or disable
    3. ADC_CR2
        [30]    : SWSTART
        [29:28] : Trigger falling or rising
        [27:24] : External even select (ex: timer, EXTI LINE 11) 
        [11]    : ALIGN
        [10]    : EOCS(End of conversion) 

    4. ADC_SMPR1 : Sampling time 얼마나 빠르게
        [23:20] : 0000(1채널 Conversion)
    5. ADC_SMPR2 : Sampling time   
    6. ADC_SQR1 : 세번 째
    6. ADC_SQR2 : 두번 째
    7. ADC_SQR3 : 가장 순서 빠름 
        [4:0] = 00001
    8. ADC_DR   : 데이터 들어오는 곳
    9. ADC_CCR  
### 코딩 방법
    1. ADCx 모듈 체널 선택, 관련 GPIO clock enable
    2. GPIOx -> MODER <- Analog IN(11(Analog input mode))
    3. ADCx clock enable (APB2ENR)



    6. 
    7. CR2.ADON = 1;        - ADC Power On
        코딩 중간에 ADC 중단 하고 싶다면
        CR2.ADON = 0;
    8. CR2.SWSTART = 1;     - 스타트 신호
