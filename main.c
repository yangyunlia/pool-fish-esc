
/*------------------------------------------------------------------*/
/* --- STC MCU International Limited -------------------------------*/
/* --- STC 1T Series MCU Demo --------------------------------------*/
/* --- Fax: 86-0513-55012956,55012947,55012969 ---------------------*/
/* --- Tel: 86-0513-55012928,55012929,55012966 ---------------------*/
/* --- Web: www.stcai.com ------------------------------------------*/
/* --- BBS: www.stcaimcu.com ---------------------------------------*/
/* If you want to use the program or the program referenced in the  */
/* article, please specify in which data and procedures from STC    */
/* ���Ҫ�ڳ�����ʹ�ô˴���,���ڳ�����ע��ʹ���˺꾧�Ƽ������ϼ�����*/
/*------------------------------------------------------------------*/


/*************	����˵��	**************
ʹ��STC8H1K28-LQFP32�������޴�������ˢ����ֱ�����.

******************************************/
#include "fw_hal.h"

#include<math.h>

#define MAIN_Fosc		24000000L	//������ʱ��24MHZ
#define ADC_START	(1<<6)	/* �Զ���0 */
#define ADC_FLAG	(1<<5)	/* �����0 */
#define	ADC_SPEED	1		/* 0~15, ADCʱ�� = SYSclk/2/(n+1) */
#define	RES_FMT		(1<<5)	/* ADC�����ʽ 0: �����, ADC_RES: D9 D8 D7 D6 D5 D4 D3 D2, ADC_RESL: D1 D0 0  0  0  0  0  0 */
							/*             1: �Ҷ���, ADC_RES: 0  0  0  0  0  0  D9 D8, ADC_RESL: D7 D6 D5 D4 D3 D2 D1 D0 */
#define CSSETUP		(0<<7)	/* 0~1,  ADC9ͨ��ѡ��ʱ��      0: 1��ADCʱ��, 1: 2��ADCʱ��,  Ĭ��0(Ĭ��1��ADCʱ��) */
#define CSHOLD		(1<<5)	/* 0~3,  ADCͨ��ѡ�񱣳�ʱ��  (n+1)��ADCʱ��, Ĭ��1(Ĭ��2��ADCʱ��)                */
#define SMPDUTY		20		/* 10~31, ADCģ���źŲ���ʱ��  (n+1)��ADCʱ��, Ĭ��10(Ĭ��11��ADCʱ��)				*/
							/* ADCת��ʱ��: 10λADC�̶�Ϊ10��ADCʱ��, 12λADC�̶�Ϊ12��ADCʱ��. 				*/
#define PWM_RANGE (500) //pwm�������us
#define V (18L) //���������ѹ
#define KV (480L) //���kvֵ
#define START_PWM_RATE (0.05) //�������ʱ���
#define START_TURN_RATE (8L) //����Ŀ��ת�٣�һ���ӵĻ�еȦ��
#define POLES (7L) //�������

sbit PWM1   = P1^0;
sbit PWM1_L = P1^1;
sbit PWM2   = P1^2;
sbit PWM2_L = P1^3;
sbit PWM3   = P1^4;
sbit PWM3_L = P1^5;

bit	B_RUN;		//���б�־
bit	B_2ms;		//4ms��ʱ��־
bit	B_start;	//����ģʽ
bit	B_Timer3_OverFlow; //Timer3��ʱ��Ҳ���ǻ���ʱ�������ʱ

uint8_t	step;		//�л�����,0~5����״̬
uint8_t	TimeOut;	//��ת��ʱ����
uint16_t input_count = 0; //������
uint8_t	TimeIndex;		//����ʱ�䱣������
uint8_t	XiaoCiCnt;		//1:��Ҫ����, 2:��������, 0�Ѿ�����

int16_t	PWM_Set;	//Ŀ��PWM���ã���ȡPWM����������
int16_t	PWM_Value;	// ����PWMռ�ձȵ�ֵ����ֵ����ת����ֵ��ת
uint16_t	PhaseTimeTmp[8];//8������ʱ��us, �� sum/16 ����30�ȵ�Ƕ�
uint16_t	PhaseTime;		//����ʱ��us
uint16_t input_pwm; //�����pwm�ߵ�ƽʱ��us
uint16_t D_START_PWM; //�������ռ�ձ�
uint32_t max_num; //ÿ������������
uint32_t start_max_phasetime; //�������������us,��ʼ�����Ļ�������
uint32_t start_min_phasetime; //������С��������us�����������Ļ�������
uint32_t min_phasetime; //��С��������us���������е���С��������
uint32_t max_phasetime; //���������us, �������е����������

void start_config() {
    
    //����ת�ټ�������������
    max_num = V*KV*POLES*6L; //ÿ�������������
    
    //���ƻ���ʱ�������Сֵ
    min_phasetime = 60L*1000L*1000L/max_num*0.5; //��С����ʱ��us, 198us
    max_phasetime = min_phasetime*100; //�����ʱ�䣬���100��,
    
    //ȷ��������pwm��Ƶ��
    D_START_PWM = PWM_RANGE*START_PWM_RATE; //����Ƶ��
    
    //��һ��һȦת��Ŀ��Ȧ��
    start_min_phasetime = 1000L*1000L/(POLES*6L*START_TURN_RATE); //������С��������
    start_max_phasetime = 1000L*1000L/(POLES*6L*2); //�������������
}

void set_compare_int() {
    if(step & 1){
        if(PWM_Value > 0) {
            CMPCR1 = 0x8c + 0x20;	//�Ƚ����������ж�
        } else {
            CMPCR1 = 0x8c + 0x10;	//�Ƚ����½����ж�
        }
    } else {
        if(PWM_Value > 0) {
            CMPCR1 = 0x8c + 0x10;	//�Ƚ����½����ж� 
        } else {
            CMPCR1 = 0x8c + 0x20;	//�Ƚ����������ж�
        }
    }
}

void StepMotor(void) // �������к���
{   
    //step�ǵ�ǰ��λ���������һ����λȻ������
    if(PWM_Value > 0) {
        if(++step >= 6) {
            step = 0; 
        }
    } else {
        if(--step >= 6) {
            step = 5; 
        }
    }   
    
	switch(step)
	{
	case 0:  // AB  PWM1, PWM2_L=1
			PWMA_ENO = 0x00;	PWM1_L=0;	PWM3_L=0;
			PWMA_ENO = 0x01;		// ��A��ĸ߶�PWM
			PWM2_L = 1;				// ��B��ĵͶ�
			ADC_CONTR = 0x80+10;	// ѡ��P0.2��ΪADC���� ��C���ѹ
			break;
	case 1:  // AC  PWM1, PWM3_L=1
			PWMA_ENO = 0x01;	PWM1_L=0;	PWM2_L=0;	// ��A��ĸ߶�PWM
			PWM3_L = 1;				// ��C��ĵͶ�
			ADC_CONTR = 0x80+9;		// ѡ��P0.1��ΪADC���� ��B���ѹ
			break;
	case 2:  // BC  PWM2, PWM3_L=1
			PWMA_ENO = 0x00;	PWM1_L=0;	PWM2_L=0;
			PWMA_ENO = 0x04;		// ��B��ĸ߶�PWM
			PWM3_L = 1;				// ��C��ĵͶ�
			ADC_CONTR = 0x80+8;		// ѡ��P0.0��ΪADC���� ��A���ѹ
			break;
	case 3:  // BA  PWM2, PWM1_L=1
			PWMA_ENO = 0x04;	PWM2_L=0;	PWM3_L=0;	// ��B��ĸ߶�PWM
			PWM1_L = 1;				// ��C��ĵͶ�
			ADC_CONTR = 0x80+10;	// ѡ��P0.2��ΪADC���� ��C���ѹ
			break;
	case 4:  // CA  PWM3, PWM1_L=1
			PWMA_ENO = 0x00;	PWM2_L=0;	PWM3_L=0;
			PWMA_ENO = 0x10;		// ��C��ĸ߶�PWM
			PWM1_L = 1;				// ��A��ĵͶ�
			ADC_CONTR = 0x80+9;		// ѡ��P0.1��ΪADC���� ��B���ѹ
			break;
	case 5:  // CB  PWM3, PWM2_L=1
			PWMA_ENO = 0x10;	PWM1_L=0;	PWM3_L=0;	// ��C��ĸ߶�PWM
			PWM2_L = 1;				// ��B��ĵͶ�
			ADC_CONTR = 0x80+8;		// ѡ��P0.0��ΪADC���� ��A���ѹ
			break;

	default:
			break;
	}
    set_compare_int();

	if(B_start)		CMPCR1 = 0x8C;	// ����ʱ��ֹ�½��غ��������ж�
}



void PWMA_config(void)
{
	P_SW2 |= 0x80;		//SFR enable   

	PWM1   = 0;
	PWM1_L = 0;
	PWM2   = 0;
	PWM2_L = 0;
	PWM3   = 0;
	PWM3_L = 0;
    GPIO_P1_SetMode(0x3f, GPIO_Mode_Output_PP);
	PWMA_PSCR = 3;		// Ԥ��Ƶ�Ĵ���, ��Ƶ Fck_cnt = Fck_psc/(PSCR[15:0}+1), ���ض���PWMƵ�� = SYSclk/((PSCR+1)*(AAR+1)), �������PWMƵ�� = SYSclk/((PSCR+1)*(AAR+1)*2).
	PWMA_DTR  = 24;		// ����ʱ������, n=0~127: DTR= n T,   0x80 ~(0x80+n), n=0~63: DTR=(64+n)*2T,  
						//				0xc0 ~(0xc0+n), n=0~31: DTR=(32+n)*8T,   0xE0 ~(0xE0+n), n=0~31: DTR=(32+n)*16T,
	PWMA_ARR    = PWM_RANGE;	// �Զ���װ�ؼĴ���,  ����PWM����
	PWMA_CCER1  = 0;
	PWMA_CCER2  = 0;
	PWMA_SR1    = 0;
	PWMA_SR2    = 0;
	PWMA_ENO    = 0;
	PWMA_PS     = 0;
	PWMA_IER    = 0;
//	PWMA_ISR_En = 0;

	PWMA_CCMR1  = 0x68;		// ͨ��ģʽ����, PWMģʽ1, Ԥװ������
	PWMA_CCR1   = 0;		// �Ƚ�ֵ, ����ռ�ձ�(�ߵ�ƽʱ����)
	PWMA_CCER1 |= 0x05;		// �����Ƚ����, �ߵ�ƽ��Ч
	PWMA_PS    |= 0;		// ѡ��IO, 0:ѡ��P1.0 P1.1, 1:ѡ��P2.0 P2.1, 2:ѡ��P6.0 P6.1, 
//	PWMA_ENO   |= 0x01;		// IO�������,  bit7: ENO4N, bit6: ENO4P, bit5: ENO3N, bit4: ENO3P,  bit3: ENO2N,  bit2: ENO2P,  bit1: ENO1N,  bit0: ENO1P
//	PWMA_IER   |= 0x02;		// ʹ���ж�

	PWMA_CCMR2  = 0x68;		// ͨ��ģʽ����, PWMģʽ1, Ԥװ������
	PWMA_CCR2   = 0;		// �Ƚ�ֵ, ����ռ�ձ�(�ߵ�ƽʱ����)
	PWMA_CCER1 |= 0x50;		// �����Ƚ����, �ߵ�ƽ��Ч
	PWMA_PS    |= (0<<2);	// ѡ��IO, 0:ѡ��P1.2 P1.3, 1:ѡ��P2.2 P2.3, 2:ѡ��P6.2 P6.3, 
//	PWMA_ENO   |= 0x04;		// IO�������,  bit7: ENO4N, bit6: ENO4P, bit5: ENO3N, bit4: ENO3P,  bit3: ENO2N,  bit2: ENO2P,  bit1: ENO1N,  bit0: ENO1P
//	PWMA_IER   |= 0x04;		// ʹ���ж�

	PWMA_CCMR3  = 0x68;		// ͨ��ģʽ����, PWMģʽ1, Ԥװ������
	PWMA_CCR3   = 0;		// �Ƚ�ֵ, ����ռ�ձ�(�ߵ�ƽʱ����)
	PWMA_CCER2 |= 0x05;		// �����Ƚ����, �ߵ�ƽ��Ч
	PWMA_PS    |= (0<<4);	// ѡ��IO, 0:ѡ��P1.4 P1.5, 1:ѡ��P2.4 P2.5, 2:ѡ��P6.4 P6.5, 
//	PWMA_ENO   |= 0x10;		// IO�������,  bit7: ENO4N, bit6: ENO4P, bit5: ENO3N, bit4: ENO3P,  bit3: ENO2N,  bit2: ENO2P,  bit1: ENO1N,  bit0: ENO1P
//	PWMA_IER   |= 0x08;		// ʹ���ж�

	PWMA_BKR    = 0x80;		// �����ʹ�� �൱���ܿ���
	PWMA_CR1    = 0x81;		// ʹ�ܼ�����, �����Զ���װ�ؼĴ�������, ���ض���ģʽ, ���ϼ���,  bit7=1:д�Զ���װ�ؼĴ�������(�����ڲ��ᱻ����), =0:ֱ��д�Զ���װ�ؼĴ�����(���ڿ��ܻ��ҵ�)
	PWMA_EGR    = 0x01;		//����һ�θ����¼�, ��������������Ƶ������, װ��Ԥ��Ƶ�Ĵ�����ֵ
//	PWMA_ISR_En = PWMA_IER;	//���ñ�־����ͨ��1~4�жϴ���
}

//	PWMA_PS   = (0<<6)+(0<<4)+(0<<2)+0;	//ѡ��IO, 4��Ӹߵ���(������)��ӦPWM1 PWM2 PWM3 PWM4, 0:ѡ��P1.x, 1:ѡ��P2.x, 2:ѡ��P6.x, 
//  PWMA_PS    PWM4N PWM4P    PWM3N PWM3P    PWM2N PWM2P    PWM1N PWM1P
//    00       P1.7  P1.6     P1.5  P1.4     P1.3  P1.2     P1.1  P1.0
//    01       P2.7  P2.6     P2.5  P2.4     P2.3  P2.2     P2.1  P2.0
//    02       P6.7  P6.6     P6.5  P6.4     P6.3  P6.2     P6.1  P6.0
//    03       P3.3  P3.4      --    --       --    --       --    --


void ADC_config(void)	//ADC��ʼ������(Ϊ��ʹ��ADC��������Ƚ����ź�, ʵ��û������ADCת��)
{
    GPIO_P1_SetMode(0xc0, GPIO_Mode_Input_HIP);
    GPIO_P0_SetMode(0x0f, GPIO_Mode_Input_HIP);
	ADC_CONTR = 0x80 + 6;	//ADC on + channel
	ADCCFG = RES_FMT + ADC_SPEED;
	P_SW2 |=  0x80;	//����XSFR
	ADCTIM = CSSETUP + CSHOLD + SMPDUTY;
}

void CMP_config(void)	//�Ƚ�����ʼ������
{
	CMPCR1 = 0x8C;			// 1000 1100 �򿪱Ƚ�����P3.6��Ϊ�Ƚ����ķ�������ˣ�ADC������Ϊ������� 
	CMPCR2 = 60;			//60��ʱ���˲�   �ȽϽ���仯��ʱ������, 0~63
	GPIO_P3_SetMode(0x40, GPIO_Mode_Input_HIP);
	P_SW2 |= 0x80;		//SFR enable   
//	CMPEXCFG |= (0<<6);	//bit7 bit6: �Ƚ�����������ѡ��: 0: 0mV,  1: 10mV, 2: 20mV, 3: 30mV
//	CMPEXCFG |= (0<<2);	//bit2: ���븺����ѡ��, 0: ѡ��P3.6������,   1: ѡ���ڲ�BandGap��ѹBGv��������.
//	CMPEXCFG |=  0;		//bit1 bit0: ����������ѡ��, 0: ѡ��P3.7������,   1: ѡ��P5.0������,  2: ѡ��P5.1������,  3: ѡ��ADC����(��ADC_CHS[3:0]��ѡ���ADC�������������).
//	CMPEXCFG = (0<<6)+(0<<2)+3;
}

//����Ƿ��ת
int checkPhaseTime() {
    uint16_t min = -1;
    uint16_t max = 0; 
    uint16_t ave = 0;
    int8_t	i;
    for(PhaseTime=0, i=0; i<8; i++)	{ 
        PhaseTime += PhaseTimeTmp[i];	//��8�λ���ʱ���ۼӺ�
        if(PhaseTimeTmp[i] < min) {
            min = PhaseTimeTmp[i];
        }
        if(PhaseTimeTmp[i] > max) {
            max = PhaseTimeTmp[i];
        }        
    }
    PhaseTime = PhaseTime >> 3;		//�˴ε�Ƕ�60��ʱ���ֵ
    if(PhaseTime > (min<<1)) {
        return 0;
    }
    
    if(PhaseTime < (max>>1)) {
        return 0;
    }
    
    if(PhaseTime < min_phasetime) {
        return 0;
    }
    
    if(PhaseTime > max_phasetime) {
        return 0;
    }
    
    return 1;
}

void CMP_ISR(void) interrupt 21		//�Ƚ����жϺ���, ��⵽���綯�ƹ�0�¼�
{
	CMPCR1 &= ~0x40;	// ���������жϱ�־λ

	if(XiaoCiCnt == 0)	//���ź�ż���0�¼�,   XiaoCiCnt=1:��Ҫ����, =2:��������, =0�Ѿ�����
	{
		T4T3M &= ~(1<<3);	// Timer3ֹͣ����
		if(B_Timer3_OverFlow)	//�л�ʱ����(Timer3)�����
		{
			B_Timer3_OverFlow = 0;
			PhaseTime = max_phasetime;	//�����ʱ��
		} else {
			PhaseTime = (((uint16_t)T3H << 8) + T3L) >> 1;	//��λΪ1us
			if(PhaseTime >= max_phasetime)	PhaseTime = max_phasetime;	//����ʱ��
		}
        
        //timer3���㣬���¿�ʼ���������ڼ�¼���ι���ʱ��ļ�࣬��60��ʱ��
		T3H = 0;	T3L = 0;
		T4T3M |=  (1<<3);
        
		PhaseTimeTmp[TimeIndex] = PhaseTime;	//����һ�λ���ʱ��
		if(++TimeIndex >= 8)	TimeIndex = 0;	//�ۼ�8��
        
        //�жϻ���ʱ���Ƿ���������Χ��������ǾͿ����Ƕ�ת
		if(checkPhaseTime()) {
            TimeOut = TimeOut + 5;	//�����������ʱ����������
            if(TimeOut > 100) {
                TimeOut = 100;
            } 
        } else {
            TimeOut = TimeOut - 10; //�쳣�������ʱ��������С
        }
        
        //���������˲�����������ͺ�ʱ��
		if( PhaseTime >= min_phasetime) {
            PhaseTime -= 40;
        } else {
            PhaseTime  = min_phasetime - 40;
        }
        
		T4T3M &= ~(1<<7);				//Timer4ֹͣ����
        
		PhaseTime  = PhaseTime;	//60��ʱ��պ���30��������1usǡ������ʱ�ӣ�����30�ȵļ�������ǡ�õ���60�ȵ�us��
		PhaseTime = 0 - PhaseTime;
		T4H = (uint8_t)(PhaseTime >> 8);		//װ��30�Ƚ���ʱ
		T4L = (uint8_t)PhaseTime;
		T4T3M |=  (1<<7);	//Timer4��ʼ����
		XiaoCiCnt = 1;		//1:��Ҫ����, 2:��������, 0�Ѿ�����
	}
}

//����TimerOΪ4ms�ж�һ�Σ�Ҳ����250HZ����������250HZˢ��
void Timer0_config(void)	//Timer0��ʼ������
{
    
    TIM_Timer0_Config(HAL_State_OFF, TIM_TimerMode_16BitAuto, 500);
	TIM_Timer0_SetRunState(HAL_State_ON); //��ʼִ��
	EXTI_Timer0_SetIntState(HAL_State_ON); //�����ж�
}
void Timer0_ISR(void) interrupt 1	//Timer0�жϺ���
{
	B_2ms = 1;	//2ms��ʱ��־
}

//����ģʽ��Timer3���ڼ������ι���������ʱ�䣬2MHZƵ�ʣ�1us��������
//============================ timer3��ʼ������ ============================================
void Timer3_Config(void)
{
	TIM_Timer3_SetRunState(HAL_State_OFF); //ֹͣ
    TIM_Timer3_Set1TMode(HAL_State_OFF); //12T
    TIM_Timer3_SetInitValue(0 ,0); //��ʼ��Ϊ0��0
	EXTI_Timer3_SetIntState(HAL_State_ON); //�����ж�
	TIM_Timer3_SetRunState(HAL_State_ON); //��ʼ����
}

//�����ӳ�30��ʱ��Ȼ���࣬2MHZ���У�Ҳ���ڵȴ�����ʱ��
//============================ timer4��ʼ������ ============================================
void Timer4_Config(void)
{
	TIM_Timer4_SetRunState(HAL_State_OFF); //ֹͣ
    TIM_Timer4_Set1TMode(HAL_State_OFF); //12T
    TIM_Timer4_SetInitValue(0 ,0); //��ʼ��Ϊ0��0
	EXTI_Timer4_SetIntState(HAL_State_ON); //�����ж�
}

//=========================== timer3�жϺ��� =============================================
void timer3_ISR (void) interrupt 19
{
	B_Timer3_OverFlow = 1;	//�����־
}

//=========================== timer4�жϺ��� =============================================
void timer4_ISR (void) interrupt 20
{
	T4T3M &= ~(1<<7);	//Timer4ֹͣ����
	if(XiaoCiCnt == 1)		//�����Ҫ����. ÿ�μ�⵽��0�¼����һ���ж�Ϊ30�Ƚ���ʱ, ����������ʱ.
	{
		XiaoCiCnt = 2;		//1:��Ҫ����, 2:��������, 0�Ѿ�����
		if(B_RUN)	//�����������
		{
			StepMotor();
		}
		
        //����ʱ��, �������Ȧ(���)������С��0�Ĺ�����, ���ַ��綯��, ����Խ������ʱ��Խ��, ��0���Ҫ�����ʱ��֮��
		//100%ռ�ձ�ʱʩ�ӽ��ظ���, �����������, ����ʾ����������ʱ��.
		//PhaseTimeһ��ļ���������30��ʱ���һ��
		T4H = (uint8_t)((65536UL - 40*2) >> 8);	//װ��������ʱ
		T4L = (uint8_t)(65536UL - 40*2);
		T4T3M |=  (1<<7); //Timer4��ʼ����
	}
	else if(XiaoCiCnt == 2)	XiaoCiCnt = 0;		//1:��Ҫ����, 2:��������, 0�Ѿ�����
}


//ֹͣת��
void stop() {
    B_RUN  = 0; //�������б�־ֹͣ����
    PWM_Value = 0; //��ʼ����ǰpwmֵ               
    CMPCR1 = 0x8C;	// �رȽ����ж�               
    PWMA_ENO  = 0;  //�ر�pwm���          
    //pwmռ�ձȼ�������
    PWMA_CCR1 = 0;	PWMA_CCR2 = 0;	PWMA_CCR3 = 0;   
    //�ر��¹�  
    PWM1_L=0;	PWM2_L=0;	PWM3_L=0; 
    SYS_Delay(80);  //�ȴ���Ȼɲ��
    
    //���¹�,������ͨ������ɲ��    
    PWM1_L=1;	PWM2_L=1;	PWM3_L=1;       
    SYS_DelayUs(100);    
    //�ر��¹�  
    PWM1_L=0;	PWM2_L=0;	PWM3_L=0;   
}

/******************* ǿ�Ƶ���������� ***************************/
void StartMotor(void)
{   
	uint16_t timer,i, min;
    
	CMPCR1 = 0x8C;	// �رȽ����ж�
    PWM_Value = PWM_Value<< 1;
	PWMA_CCR1 = abs(PWM_Value);
	PWMA_CCR2 = abs(PWM_Value);
	PWMA_CCR3 = abs(PWM_Value);
	StepMotor();	SYS_Delay(20); 	//Delay_n_ms(250);// ��ʼλ��
	timer = start_max_phasetime/100;	//���ȵ������
    min = start_min_phasetime/100;
	while(1)
	{
		for(i=0; i<timer; i++)	SYS_DelayUs(100);  //���ݵ����������, ���ת�ٵȵȵ������������ٶ�
		timer -= timer /16;
		StepMotor();
		if(timer < min)	return;
	}
}


//PWMA��������
void PWMB_Init(void) {
	PWMB_SetPrescaler(23); //ʱ��24MHZ��
	PWMB_PWM1_SetPort(PWMB_PWM5_AlterPort_P20); //��׽1ͨ�������ź�
	
	PWMB_PWM1_SetPortState(0);
	PWMB_PWM2_SetPortState(0); 
    
	PWMB_PWM1_SetPortDirection(PWMB_PortDirIn_TI5FP5_TI6FP6_TI7FP7_TI8FP8);  //��׽PWMA5 ����
	PWMB_PWM2_SetPortDirection(PWMB_PortDirIn_TI6FP5_TI5FP6_TI8FP7_TI7FP8); //��׽PWMA5 ��
	
	//����ʹ��
	PWMB_PWM1_SetPortState(1);
	PWMB_PWM2_SetPortState(1);  
	
	PWMB_PWM1_SetPortPolar(0); //��׽5ͨ��������
	PWMB_PWM2_SetPortPolar(1); //��׽6ͨ���½���
	
	PWMB_SetCounterState(1);
	
    //���ж�,ֻ���½����жϣ��½��ص�ʱ���ټ���ռ�ձ�
	EXTI_INT_PWMB_CapCH2_ON;
}

//PWMB�ж�
INTERRUPT(PWMB_Routine, EXTI_VectPWMB) {
	P_SW2 |= 0x80;
	if(PWMB_SR1 & 0X02) {
		PWMB_SR1 &= ~0X02;
	}
	if(PWMB_SR1 & 0X04) {
		PWMB_SR1 &= ~0X04;
		input_pwm = (uint16_t)PWMB_CCR6 - (uint16_t)PWMB_CCR5;
        //�������뷶Χ֮�⣬������,��20us�Ŀ�ԣ��
        if(input_pwm < 980 || input_pwm > 2020) {
            PWM_Set = 0;
        } else {
            PWM_Set = (int16_t)input_pwm - 1500; 
        }        
	}
}

/**********************************************/
void main(void)
{
	uint8_t	i;
	start_config();
	GPIO_P2_SetMode(0xf8, GPIO_Mode_InOut_QBD);
    GPIO_P3_SetMode(0xbf, GPIO_Mode_InOut_QBD);
    GPIO_P5_SetMode(0x10, GPIO_Mode_InOut_QBD);
	PWMA_config();
    PWMB_Init();
	ADC_config();
	CMP_config();
	Timer0_config();	// Timer0��ʼ������
	Timer3_Config();	// Timer3��ʼ������
	Timer4_Config();	// Timer4��ʼ������
	PWM_Set = 0;
	TimeOut = 0;
    input_count = 0;
    PWM_Value = 0;
    UART1_Config8bitUart(UART1_BaudSource_Timer1, HAL_State_ON, 115200);
	EA  = 1; // �����ж� 
	while (1)
	{
		if(B_2ms)		// 2msʱ϶
		{
            B_2ms = 0;
            if(input_count != 500) {
                if(abs(PWM_Set) < (D_START_PWM)) {
                    input_count++;
                } else {
                    input_count = 0;
                }
                continue;
            }
            if(PWM_Value < PWM_Set)	PWM_Value += 1;	//���Ÿ���pwm����	
            if(PWM_Value > PWM_Set)	PWM_Value -= 1;
            
            //������
            if(B_RUN) {
                if(--TimeOut <= 0)	//��ת��ʱ
                {
                    stop(); //ֹͣ���
                    SYS_Delay(1000);	//��תʱ,��ʱһ��ʱ��������
                }
            }

			if(!B_RUN && (abs(PWM_Value) >= (D_START_PWM)))	// ռ�ձȴ����趨ֵ, ���ҵ��δ����, ���������
			{
				B_start = 1;		//����ģʽ
				for(i=0; i<8; i++)	PhaseTimeTmp[i] = start_min_phasetime;
				StartMotor();		// �������
				B_start = 0;
				XiaoCiCnt = 0;		//��ʼ����ʱ
				CMPCR1 &= ~0x40;	// ����жϱ�־λ
				set_compare_int();
				B_RUN = 1;
				SYS_Delay(5);	//��ʱһ��, ����������
				TimeOut = 100;		//������ʱʱ��
			}

			if(B_RUN)	//����������
			{
				if(abs(PWM_Value) < (D_START_PWM))	// ͣת
                {
					stop();
				}
				else
				{
					PWMA_CCR1 = abs(PWM_Value);
					PWMA_CCR2 = abs(PWM_Value);
					PWMA_CCR3 = abs(PWM_Value);
				}
			}
		}
	}
}



