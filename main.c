
/*------------------------------------------------------------------*/
/* --- STC MCU International Limited -------------------------------*/
/* --- STC 1T Series MCU Demo --------------------------------------*/
/* --- Fax: 86-0513-55012956,55012947,55012969 ---------------------*/
/* --- Tel: 86-0513-55012928,55012929,55012966 ---------------------*/
/* --- Web: www.stcai.com ------------------------------------------*/
/* --- BBS: www.stcaimcu.com ---------------------------------------*/
/* If you want to use the program or the program referenced in the  */
/* article, please specify in which data and procedures from STC    */
/* 如果要在程序中使用此代码,请在程序中注明使用了宏晶科技的资料及程序*/
/*------------------------------------------------------------------*/


/*************	功能说明	**************
使用STC8H1K28-LQFP32来驱动无传感器无刷三相直流电机.

******************************************/
#include "fw_hal.h"

#include<math.h>

#define MAIN_Fosc		24000000L	//定义主时钟24MHZ
#define ADC_START	(1<<6)	/* 自动清0 */
#define ADC_FLAG	(1<<5)	/* 软件清0 */
#define	ADC_SPEED	1		/* 0~15, ADC时钟 = SYSclk/2/(n+1) */
#define	RES_FMT		(1<<5)	/* ADC结果格式 0: 左对齐, ADC_RES: D9 D8 D7 D6 D5 D4 D3 D2, ADC_RESL: D1 D0 0  0  0  0  0  0 */
							/*             1: 右对齐, ADC_RES: 0  0  0  0  0  0  D9 D8, ADC_RESL: D7 D6 D5 D4 D3 D2 D1 D0 */
#define CSSETUP		(0<<7)	/* 0~1,  ADC9通道选择时间      0: 1个ADC时钟, 1: 2个ADC时钟,  默认0(默认1个ADC时钟) */
#define CSHOLD		(1<<5)	/* 0~3,  ADC通道选择保持时间  (n+1)个ADC时钟, 默认1(默认2个ADC时钟)                */
#define SMPDUTY		20		/* 10~31, ADC模拟信号采样时间  (n+1)个ADC时钟, 默认10(默认11个ADC时钟)				*/
							/* ADC转换时间: 10位ADC固定为10个ADC时钟, 12位ADC固定为12个ADC时钟. 				*/
#define PWM_RANGE (500) //pwm输出周期us
#define V (18L) //电机工作电压
#define KV (480L) //电机kv值
#define START_PWM_RATE (0.05) //启动功率比例
#define START_TURN_RATE (8L) //启动目标转速，一秒钟的机械圈数
#define POLES (7L) //电机极数

sbit PWM1   = P1^0;
sbit PWM1_L = P1^1;
sbit PWM2   = P1^2;
sbit PWM2_L = P1^3;
sbit PWM3   = P1^4;
sbit PWM3_L = P1^5;

bit	B_RUN;		//运行标志
bit	B_2ms;		//4ms定时标志
bit	B_start;	//启动模式
bit	B_Timer3_OverFlow; //Timer3超时，也就是换相时间计数超时

uint8_t	step;		//切换步骤,0~5六个状态
uint8_t	TimeOut;	//堵转超时计数
uint16_t input_count = 0; //输入检测
uint8_t	TimeIndex;		//换相时间保存索引
uint8_t	XiaoCiCnt;		//1:需要消磁, 2:正在消磁, 0已经消磁

int16_t	PWM_Set;	//目标PWM设置，读取PWM输入来设置
int16_t	PWM_Value;	// 决定PWM占空比的值，正值，正转，负值反转
uint16_t	PhaseTimeTmp[8];//8个换相时间us, 其 sum/16 就是30度电角度
uint16_t	PhaseTime;		//换相时间us
uint16_t input_pwm; //输入的pwm高电平时间us
uint16_t D_START_PWM; //启动输出占空比
uint32_t max_num; //每分钟最大换相次数
uint32_t start_max_phasetime; //启动最大换相周期us,开始启动的换相周期
uint32_t start_min_phasetime; //启动最小换相周期us，启动结束的换相周期
uint32_t min_phasetime; //最小换相周期us，正常运行的最小换相周期
uint32_t max_phasetime; //最大换相周期us, 正常运行的最大换相周期

void start_config() {
    
    //根据转速计算理论最大参数
    max_num = V*KV*POLES*6L; //每分钟最大换相数量
    
    //估计换相时间最大最小值
    min_phasetime = 60L*1000L*1000L/max_num*0.5; //最小换相时间us, 198us
    max_phasetime = min_phasetime*100; //最大换相时间，相差100倍,
    
    //确定启动的pwm波频率
    D_START_PWM = PWM_RANGE*START_PWM_RATE; //启动频率
    
    //从一秒一圈转到目标圈数
    start_min_phasetime = 1000L*1000L/(POLES*6L*START_TURN_RATE); //启动最小换相周期
    start_max_phasetime = 1000L*1000L/(POLES*6L*2); //启动最大换相周期
}

void set_compare_int() {
    if(step & 1){
        if(PWM_Value > 0) {
            CMPCR1 = 0x8c + 0x20;	//比较器上升沿中断
        } else {
            CMPCR1 = 0x8c + 0x10;	//比较器下降沿中断
        }
    } else {
        if(PWM_Value > 0) {
            CMPCR1 = 0x8c + 0x10;	//比较器下降沿中断 
        } else {
            CMPCR1 = 0x8c + 0x20;	//比较器上升沿中断
        }
    }
}

void StepMotor(void) // 换相序列函数
{   
    //step是当前相位，计算出下一个相位然后运行
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
			PWMA_ENO = 0x01;		// 打开A相的高端PWM
			PWM2_L = 1;				// 打开B相的低端
			ADC_CONTR = 0x80+10;	// 选择P0.2作为ADC输入 即C相电压
			break;
	case 1:  // AC  PWM1, PWM3_L=1
			PWMA_ENO = 0x01;	PWM1_L=0;	PWM2_L=0;	// 打开A相的高端PWM
			PWM3_L = 1;				// 打开C相的低端
			ADC_CONTR = 0x80+9;		// 选择P0.1作为ADC输入 即B相电压
			break;
	case 2:  // BC  PWM2, PWM3_L=1
			PWMA_ENO = 0x00;	PWM1_L=0;	PWM2_L=0;
			PWMA_ENO = 0x04;		// 打开B相的高端PWM
			PWM3_L = 1;				// 打开C相的低端
			ADC_CONTR = 0x80+8;		// 选择P0.0作为ADC输入 即A相电压
			break;
	case 3:  // BA  PWM2, PWM1_L=1
			PWMA_ENO = 0x04;	PWM2_L=0;	PWM3_L=0;	// 打开B相的高端PWM
			PWM1_L = 1;				// 打开C相的低端
			ADC_CONTR = 0x80+10;	// 选择P0.2作为ADC输入 即C相电压
			break;
	case 4:  // CA  PWM3, PWM1_L=1
			PWMA_ENO = 0x00;	PWM2_L=0;	PWM3_L=0;
			PWMA_ENO = 0x10;		// 打开C相的高端PWM
			PWM1_L = 1;				// 打开A相的低端
			ADC_CONTR = 0x80+9;		// 选择P0.1作为ADC输入 即B相电压
			break;
	case 5:  // CB  PWM3, PWM2_L=1
			PWMA_ENO = 0x10;	PWM1_L=0;	PWM3_L=0;	// 打开C相的高端PWM
			PWM2_L = 1;				// 打开B相的低端
			ADC_CONTR = 0x80+8;		// 选择P0.0作为ADC输入 即A相电压
			break;

	default:
			break;
	}
    set_compare_int();

	if(B_start)		CMPCR1 = 0x8C;	// 启动时禁止下降沿和上升沿中断
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
	PWMA_PSCR = 3;		// 预分频寄存器, 分频 Fck_cnt = Fck_psc/(PSCR[15:0}+1), 边沿对齐PWM频率 = SYSclk/((PSCR+1)*(AAR+1)), 中央对齐PWM频率 = SYSclk/((PSCR+1)*(AAR+1)*2).
	PWMA_DTR  = 24;		// 死区时间配置, n=0~127: DTR= n T,   0x80 ~(0x80+n), n=0~63: DTR=(64+n)*2T,  
						//				0xc0 ~(0xc0+n), n=0~31: DTR=(32+n)*8T,   0xE0 ~(0xE0+n), n=0~31: DTR=(32+n)*16T,
	PWMA_ARR    = PWM_RANGE;	// 自动重装载寄存器,  控制PWM周期
	PWMA_CCER1  = 0;
	PWMA_CCER2  = 0;
	PWMA_SR1    = 0;
	PWMA_SR2    = 0;
	PWMA_ENO    = 0;
	PWMA_PS     = 0;
	PWMA_IER    = 0;
//	PWMA_ISR_En = 0;

	PWMA_CCMR1  = 0x68;		// 通道模式配置, PWM模式1, 预装载允许
	PWMA_CCR1   = 0;		// 比较值, 控制占空比(高电平时钟数)
	PWMA_CCER1 |= 0x05;		// 开启比较输出, 高电平有效
	PWMA_PS    |= 0;		// 选择IO, 0:选择P1.0 P1.1, 1:选择P2.0 P2.1, 2:选择P6.0 P6.1, 
//	PWMA_ENO   |= 0x01;		// IO输出允许,  bit7: ENO4N, bit6: ENO4P, bit5: ENO3N, bit4: ENO3P,  bit3: ENO2N,  bit2: ENO2P,  bit1: ENO1N,  bit0: ENO1P
//	PWMA_IER   |= 0x02;		// 使能中断

	PWMA_CCMR2  = 0x68;		// 通道模式配置, PWM模式1, 预装载允许
	PWMA_CCR2   = 0;		// 比较值, 控制占空比(高电平时钟数)
	PWMA_CCER1 |= 0x50;		// 开启比较输出, 高电平有效
	PWMA_PS    |= (0<<2);	// 选择IO, 0:选择P1.2 P1.3, 1:选择P2.2 P2.3, 2:选择P6.2 P6.3, 
//	PWMA_ENO   |= 0x04;		// IO输出允许,  bit7: ENO4N, bit6: ENO4P, bit5: ENO3N, bit4: ENO3P,  bit3: ENO2N,  bit2: ENO2P,  bit1: ENO1N,  bit0: ENO1P
//	PWMA_IER   |= 0x04;		// 使能中断

	PWMA_CCMR3  = 0x68;		// 通道模式配置, PWM模式1, 预装载允许
	PWMA_CCR3   = 0;		// 比较值, 控制占空比(高电平时钟数)
	PWMA_CCER2 |= 0x05;		// 开启比较输出, 高电平有效
	PWMA_PS    |= (0<<4);	// 选择IO, 0:选择P1.4 P1.5, 1:选择P2.4 P2.5, 2:选择P6.4 P6.5, 
//	PWMA_ENO   |= 0x10;		// IO输出允许,  bit7: ENO4N, bit6: ENO4P, bit5: ENO3N, bit4: ENO3P,  bit3: ENO2N,  bit2: ENO2P,  bit1: ENO1N,  bit0: ENO1P
//	PWMA_IER   |= 0x08;		// 使能中断

	PWMA_BKR    = 0x80;		// 主输出使能 相当于总开关
	PWMA_CR1    = 0x81;		// 使能计数器, 允许自动重装载寄存器缓冲, 边沿对齐模式, 向上计数,  bit7=1:写自动重装载寄存器缓冲(本周期不会被打扰), =0:直接写自动重装载寄存器本(周期可能会乱掉)
	PWMA_EGR    = 0x01;		//产生一次更新事件, 清除计数器和与分频计数器, 装载预分频寄存器的值
//	PWMA_ISR_En = PWMA_IER;	//设置标志允许通道1~4中断处理
}

//	PWMA_PS   = (0<<6)+(0<<4)+(0<<2)+0;	//选择IO, 4项从高到低(从左到右)对应PWM1 PWM2 PWM3 PWM4, 0:选择P1.x, 1:选择P2.x, 2:选择P6.x, 
//  PWMA_PS    PWM4N PWM4P    PWM3N PWM3P    PWM2N PWM2P    PWM1N PWM1P
//    00       P1.7  P1.6     P1.5  P1.4     P1.3  P1.2     P1.1  P1.0
//    01       P2.7  P2.6     P2.5  P2.4     P2.3  P2.2     P2.1  P2.0
//    02       P6.7  P6.6     P6.5  P6.4     P6.3  P6.2     P6.1  P6.0
//    03       P3.3  P3.4      --    --       --    --       --    --


void ADC_config(void)	//ADC初始化函数(为了使用ADC输入端做比较器信号, 实际没有启动ADC转换)
{
    GPIO_P1_SetMode(0xc0, GPIO_Mode_Input_HIP);
    GPIO_P0_SetMode(0x0f, GPIO_Mode_Input_HIP);
	ADC_CONTR = 0x80 + 6;	//ADC on + channel
	ADCCFG = RES_FMT + ADC_SPEED;
	P_SW2 |=  0x80;	//访问XSFR
	ADCTIM = CSSETUP + CSHOLD + SMPDUTY;
}

void CMP_config(void)	//比较器初始化程序
{
	CMPCR1 = 0x8C;			// 1000 1100 打开比较器，P3.6作为比较器的反相输入端，ADC引脚作为正输入端 
	CMPCR2 = 60;			//60个时钟滤波   比较结果变化延时周期数, 0~63
	GPIO_P3_SetMode(0x40, GPIO_Mode_Input_HIP);
	P_SW2 |= 0x80;		//SFR enable   
//	CMPEXCFG |= (0<<6);	//bit7 bit6: 比较器迟滞输入选择: 0: 0mV,  1: 10mV, 2: 20mV, 3: 30mV
//	CMPEXCFG |= (0<<2);	//bit2: 输入负极性选择, 0: 选择P3.6做输入,   1: 选择内部BandGap电压BGv做负输入.
//	CMPEXCFG |=  0;		//bit1 bit0: 输入正极性选择, 0: 选择P3.7做输入,   1: 选择P5.0做输入,  2: 选择P5.1做输入,  3: 选择ADC输入(由ADC_CHS[3:0]所选择的ADC输入端做正输入).
//	CMPEXCFG = (0<<6)+(0<<2)+3;
}

//检测是否堵转
int checkPhaseTime() {
    uint16_t min = -1;
    uint16_t max = 0; 
    uint16_t ave = 0;
    int8_t	i;
    for(PhaseTime=0, i=0; i<8; i++)	{ 
        PhaseTime += PhaseTimeTmp[i];	//求8次换相时间累加和
        if(PhaseTimeTmp[i] < min) {
            min = PhaseTimeTmp[i];
        }
        if(PhaseTimeTmp[i] > max) {
            max = PhaseTimeTmp[i];
        }        
    }
    PhaseTime = PhaseTime >> 3;		//八次电角度60度时间均值
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

void CMP_ISR(void) interrupt 21		//比较器中断函数, 检测到反电动势过0事件
{
	CMPCR1 &= ~0x40;	// 需软件清除中断标志位

	if(XiaoCiCnt == 0)	//消磁后才检测过0事件,   XiaoCiCnt=1:需要消磁, =2:正在消磁, =0已经消磁
	{
		T4T3M &= ~(1<<3);	// Timer3停止运行
		if(B_Timer3_OverFlow)	//切换时间间隔(Timer3)有溢出
		{
			B_Timer3_OverFlow = 0;
			PhaseTime = max_phasetime;	//最大换相时间
		} else {
			PhaseTime = (((uint16_t)T3H << 8) + T3L) >> 1;	//单位为1us
			if(PhaseTime >= max_phasetime)	PhaseTime = max_phasetime;	//换相时间
		}
        
        //timer3清零，重新开始计数，用于记录两次过零时间的间距，即60度时间
		T3H = 0;	T3L = 0;
		T4T3M |=  (1<<3);
        
		PhaseTimeTmp[TimeIndex] = PhaseTime;	//保存一次换相时间
		if(++TimeIndex >= 8)	TimeIndex = 0;	//累加8次
        
        //判断换相时间是否在正常范围，如果不是就可能是堵转
		if(checkPhaseTime()) {
            TimeOut = TimeOut + 5;	//正常情况，超时计数，增加
            if(TimeOut > 100) {
                TimeOut = 100;
            } 
        } else {
            TimeOut = TimeOut - 10; //异常情况，超时计数，减小
        }
        
        //修正由于滤波电容引起的滞后时间
		if( PhaseTime >= min_phasetime) {
            PhaseTime -= 40;
        } else {
            PhaseTime  = min_phasetime - 40;
        }
        
		T4T3M &= ~(1<<7);				//Timer4停止运行
        
		PhaseTime  = PhaseTime;	//60度时间刚好是30度两倍，1us恰好两个时钟，所以30度的计数周期恰好等于60度的us数
		PhaseTime = 0 - PhaseTime;
		T4H = (uint8_t)(PhaseTime >> 8);		//装载30度角延时
		T4L = (uint8_t)PhaseTime;
		T4T3M |=  (1<<7);	//Timer4开始运行
		XiaoCiCnt = 1;		//1:需要消磁, 2:正在消磁, 0已经消磁
	}
}

//设置TimerO为4ms中断一次，也就是250HZ，主函数已250HZ刷新
void Timer0_config(void)	//Timer0初始化函数
{
    
    TIM_Timer0_Config(HAL_State_OFF, TIM_TimerMode_16BitAuto, 500);
	TIM_Timer0_SetRunState(HAL_State_ON); //开始执行
	EXTI_Timer0_SetIntState(HAL_State_ON); //允许中断
}
void Timer0_ISR(void) interrupt 1	//Timer0中断函数
{
	B_2ms = 1;	//2ms定时标志
}

//计数模式，Timer3用于计算两次过零检测间隔的时间，2MHZ频率，1us计数两次
//============================ timer3初始化函数 ============================================
void Timer3_Config(void)
{
	TIM_Timer3_SetRunState(HAL_State_OFF); //停止
    TIM_Timer3_Set1TMode(HAL_State_OFF); //12T
    TIM_Timer3_SetInitValue(0 ,0); //初始化为0，0
	EXTI_Timer3_SetIntState(HAL_State_ON); //启动中断
	TIM_Timer3_SetRunState(HAL_State_ON); //开始运行
}

//用于延迟30度时间然后换相，2MHZ运行，也用于等待消磁时间
//============================ timer4初始化函数 ============================================
void Timer4_Config(void)
{
	TIM_Timer4_SetRunState(HAL_State_OFF); //停止
    TIM_Timer4_Set1TMode(HAL_State_OFF); //12T
    TIM_Timer4_SetInitValue(0 ,0); //初始化为0，0
	EXTI_Timer4_SetIntState(HAL_State_ON); //启动中断
}

//=========================== timer3中断函数 =============================================
void timer3_ISR (void) interrupt 19
{
	B_Timer3_OverFlow = 1;	//溢出标志
}

//=========================== timer4中断函数 =============================================
void timer4_ISR (void) interrupt 20
{
	T4T3M &= ~(1<<7);	//Timer4停止运行
	if(XiaoCiCnt == 1)		//标记需要消磁. 每次检测到过0事件后第一次中断为30度角延时, 设置消磁延时.
	{
		XiaoCiCnt = 2;		//1:需要消磁, 2:正在消磁, 0已经消磁
		if(B_RUN)	//电机正在运行
		{
			StepMotor();
		}
		
        //消磁时间, 换相后线圈(电感)电流减小到0的过程中, 出现反电动势, 电流越大消磁时间越长, 过0检测要在这个时间之后
		//100%占空比时施加较重负载, 电机电流上升, 可以示波器看消磁时间.
		//PhaseTime一半的计数，等于30度时间的一半
		T4H = (uint8_t)((65536UL - 40*2) >> 8);	//装载消磁延时
		T4L = (uint8_t)(65536UL - 40*2);
		T4T3M |=  (1<<7); //Timer4开始运行
	}
	else if(XiaoCiCnt == 2)	XiaoCiCnt = 0;		//1:需要消磁, 2:正在消磁, 0已经消磁
}


//停止转动
void stop() {
    B_RUN  = 0; //设置运行标志停止运行
    PWM_Value = 0; //初始化当前pwm值               
    CMPCR1 = 0x8C;	// 关比较器中断               
    PWMA_ENO  = 0;  //关闭pwm输出          
    //pwm占空比计数清零
    PWMA_CCR1 = 0;	PWMA_CCR2 = 0;	PWMA_CCR3 = 0;   
    //关闭下管  
    PWM1_L=0;	PWM2_L=0;	PWM3_L=0; 
    SYS_Delay(80);  //等待自然刹车
    
    //打开下管,电机三项导通来主动刹车    
    PWM1_L=1;	PWM2_L=1;	PWM3_L=1;       
    SYS_DelayUs(100);    
    //关闭下管  
    PWM1_L=0;	PWM2_L=0;	PWM3_L=0;   
}

/******************* 强制电机启动函数 ***************************/
void StartMotor(void)
{   
	uint16_t timer,i, min;
    
	CMPCR1 = 0x8C;	// 关比较器中断
    PWM_Value = PWM_Value<< 1;
	PWMA_CCR1 = abs(PWM_Value);
	PWMA_CCR2 = abs(PWM_Value);
	PWMA_CCR3 = abs(PWM_Value);
	StepMotor();	SYS_Delay(20); 	//Delay_n_ms(250);// 初始位置
	timer = start_max_phasetime/100;	//风扇电机启动
    min = start_min_phasetime/100;
	while(1)
	{
		for(i=0; i<timer; i++)	SYS_DelayUs(100);  //根据电机加速特性, 最高转速等等调整启动加速速度
		timer -= timer /16;
		StepMotor();
		if(timer < min)	return;
	}
}


//PWMA捕获设置
void PWMB_Init(void) {
	PWMB_SetPrescaler(23); //时钟24MHZ，
	PWMB_PWM1_SetPort(PWMB_PWM5_AlterPort_P20); //捕捉1通道油门信号
	
	PWMB_PWM1_SetPortState(0);
	PWMB_PWM2_SetPortState(0); 
    
	PWMB_PWM1_SetPortDirection(PWMB_PortDirIn_TI5FP5_TI6FP6_TI7FP7_TI8FP8);  //捕捉PWMA5 输入
	PWMB_PWM2_SetPortDirection(PWMB_PortDirIn_TI6FP5_TI5FP6_TI8FP7_TI7FP8); //捕捉PWMA5 输
	
	//输入使能
	PWMB_PWM1_SetPortState(1);
	PWMB_PWM2_SetPortState(1);  
	
	PWMB_PWM1_SetPortPolar(0); //捕捉5通道上升沿
	PWMB_PWM2_SetPortPolar(1); //捕捉6通道下降沿
	
	PWMB_SetCounterState(1);
	
    //开中断,只开下降沿中断，下降沿的时候再计算占空比
	EXTI_INT_PWMB_CapCH2_ON;
}

//PWMB中断
INTERRUPT(PWMB_Routine, EXTI_VectPWMB) {
	P_SW2 |= 0x80;
	if(PWMB_SR1 & 0X02) {
		PWMB_SR1 &= ~0X02;
	}
	if(PWMB_SR1 & 0X04) {
		PWMB_SR1 &= ~0X04;
		input_pwm = (uint16_t)PWMB_CCR6 - (uint16_t)PWMB_CCR5;
        //正常输入范围之外，就清零,留20us的宽裕，
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
	Timer0_config();	// Timer0初始化函数
	Timer3_Config();	// Timer3初始化函数
	Timer4_Config();	// Timer4初始化函数
	PWM_Set = 0;
	TimeOut = 0;
    input_count = 0;
    PWM_Value = 0;
    UART1_Config8bitUart(UART1_BaudSource_Timer1, HAL_State_ON, 115200);
	EA  = 1; // 打开总中断 
	while (1)
	{
		if(B_2ms)		// 2ms时隙
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
            if(PWM_Value < PWM_Set)	PWM_Value += 1;	//油门跟随pwm输入	
            if(PWM_Value > PWM_Set)	PWM_Value -= 1;
            
            //运行中
            if(B_RUN) {
                if(--TimeOut <= 0)	//堵转超时
                {
                    stop(); //停止电机
                    SYS_Delay(1000);	//堵转时,延时一点时间再启动
                }
            }

			if(!B_RUN && (abs(PWM_Value) >= (D_START_PWM)))	// 占空比大于设定值, 并且电机未运行, 则启动电机
			{
				B_start = 1;		//启动模式
				for(i=0; i<8; i++)	PhaseTimeTmp[i] = start_min_phasetime;
				StartMotor();		// 启动电机
				B_start = 0;
				XiaoCiCnt = 0;		//初始进入时
				CMPCR1 &= ~0x40;	// 清除中断标志位
				set_compare_int();
				B_RUN = 1;
				SYS_Delay(5);	//延时一下, 先启动起来
				TimeOut = 100;		//启动超时时间
			}

			if(B_RUN)	//正在运行中
			{
				if(abs(PWM_Value) < (D_START_PWM))	// 停转
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



