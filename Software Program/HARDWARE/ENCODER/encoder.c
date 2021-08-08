#include "encoder.h" 

//外部变量 extern说明改变量已在其它文件定义
extern double   TargetVelocity, Encoder,PWM; //目标速度、编码器读数、PWM控制变量
extern float Velcity_Kp,  Velcity_Ki,  Velcity_Kd; //相关速度PID参数
extern int   MortorRun;  //允许电机控制标志位

/**************************************************************************
函数功能：编码器初始化函数
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; //定义一个引脚初始化的结构体  
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;//定义一个定时器初始化的结构体
	TIM_ICInitTypeDef TIM_ICInitStructure; //定义一个定时器编码器模式初始化的结构体
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //使能TIM2时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能GPIOA时钟
	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
//	GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE);
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;	//TIM2_CH1、TIM2_CH2,编码器模式只适用于定时器的CH1和CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);	//根据GPIO_InitStructure的参数初始化GPIO

	TIM_TimeBaseStructure.TIM_Period = 0xffff; //设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_Prescaler = 0; // 预分频器 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct的参数初始化定时器TIM2
			
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //使用编码器模式3：CH1、CH2同时计数，为四分频
	
	TIM_ICStructInit(&TIM_ICInitStructure); //把TIM_ICInitStruct 中的每一个参数按缺省值填入
	TIM_ICInitStructure.TIM_ICFilter = 10;  //设置滤波器长度
	TIM_ICInit(TIM2, &TIM_ICInitStructure); //根TIM_ICInitStructure参数初始化定时器TIM2编码器模式
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);//清除TIM2的更新标志位
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //更新中断使能
	TIM_SetCounter(TIM2,0); //初始化清空编码器数值
	
	TIM_Cmd(TIM2, ENABLE); //使能定时器2
}

/**************************************************************************
函数功能：读取TIM2编码器数值，单位时间读取编码器计数
入口参数：无
返回  值：无
**************************************************************************/
int Read_Encoder(void)
{
	int Encoder_TIM;
	Encoder_TIM = TIM2->CNT; //读取计数
	if(Encoder_TIM > 0xEFFF) Encoder_TIM = Encoder_TIM-0xFFFF; //转化计数值为有方向的值，大于0正转，小于0反转。
	                                                      //TIM2->CNT范围为0-0xffff，初值为0。
	TIM2->CNT = 0; //读取完后计数清零
	return Encoder_TIM; //返回值
}

/**************************************************************************
函数功能：TIM2中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM2->SR & 0X0001)//溢出中断
	{    				   				     	    	
	}				   
	TIM2->SR &= ~(1<<0);//清除中断标志位 	    
}

/**************************************************************************
函数功能：通用定时器4初始化函数，
入口参数：自动重装载值 预分频系数 默认定时时钟为72MHZ时，两者共同决定定时中断时间
返回  值：无
**************************************************************************/
void EncoderRead_TIM4(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue; //定义一个定时中断的结构体
	NVIC_InitTypeDef NVIC_InitStrue; //定义一个中断优先级初始化的结构体
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //使能通用定时器4时钟
	
	TIM_TimeBaseInitStrue.TIM_Period = arr; //计数模式为向上计数时，定时器从0开始计数，计数超过到arr时触发定时中断服务函数
	TIM_TimeBaseInitStrue.TIM_Prescaler = psc; //预分频系数，决定每一个计数的时长
	TIM_TimeBaseInitStrue.TIM_CounterMode = TIM_CounterMode_Up; //计数模式：向上计数
	TIM_TimeBaseInitStrue.TIM_ClockDivision = TIM_CKD_DIV1; //一般不使用，默认TIM_CKD_DIV1
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStrue); //根据TIM_TimeBaseInitStrue的参数初始化定时器TIM4
	
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); //使能TIM4中断，中断模式为更新中断：TIM_IT_Update
	
	NVIC_InitStrue.NVIC_IRQChannel = TIM4_IRQn; //属于TIM4中断
	NVIC_InitStrue.NVIC_IRQChannelCmd = ENABLE; //中断使能
	NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级为1级，值越小优先级越高，0级优先级最高
	NVIC_InitStrue.NVIC_IRQChannelSubPriority = 1; //响应优先级为1级，值越小优先级越高，0级优先级最高
	NVIC_Init(&NVIC_InitStrue); //根据NVIC_InitStrue的参数初始化VIC寄存器，设置TIM4中断
	
	TIM_Cmd(TIM4, ENABLE); //使能定时器TIM4
}

