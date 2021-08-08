#include "TB6612.h"  

/**************************************************************************
函数功能：TB6612方向控制引脚初始化函数
入口参数：无
返回  值：无
**************************************************************************/
void Tb6612_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; //定义一个引脚初始化的结构体
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能GPIOB时钟，GPIOB挂载在APB2时钟下，在STM32中使用IO口前都要使能对应时钟
	
	//初始化电机1和电机2的方向控制引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure); //初始化引脚GPIOB4
	
	GPIO_ResetBits(GPIOB, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);//初始化电平
}




/**************************************************************************
函数功能：TB6612的PWM初始化函数
入口参数：定时器3计数上限,定时器3预分频系数
返回  值：无
**************************************************************************/
void TIM3_PWM_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure; //定义一个引脚初始化的结构体
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue; //定义一个定时中断的结构体	
	TIM_OCInitTypeDef TIM_OCInitTypeStrue; //定义一个PWM输出比较的结构体
	
	//第一步，使能定时器3时钟和GPIOB的时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //使能通用定时器3时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能GPIOB时钟，GPIOB挂载在APB2时钟下，在STM32中使用IO口前都要使能对应时钟
	
	//第二步，将GPIOB作为PWM输出使用，所用要要使用端口重映射开启AFIO时钟，同时设置重映射
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);//使能AFIO时钟,重映射必须使能AFIO时钟
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);//开启重映射

	//第三步，初始化IO口为复用功能输出,复用推挽输出，TB6612PWM输出引脚_PWMA和PWMB
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;//PB4和PB5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //初始化IO口为复用功能输出,复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //定义该引脚输出速度为50MHZ
    GPIO_Init(GPIOB, &GPIO_InitStructure); //初始化引脚GPIOB4
	
	//第四步，初始化定时器，arr，psc
	TIM_TimeBaseInitStrue.TIM_Period = arr; //计数模式为向上计数时，定时器从0开始计数，计数超过到arr时触发定时中断服务函数
	TIM_TimeBaseInitStrue.TIM_Prescaler = psc; //预分频系数，决定每一个计数的时长
	TIM_TimeBaseInitStrue.TIM_CounterMode = TIM_CounterMode_Up; //计数模式：向上计数
	TIM_TimeBaseInitStrue.TIM_ClockDivision = TIM_CKD_DIV1; //一般不使用，默认TIM_CKD_DIV1
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStrue); //根据TIM_TimeBaseInitStrue的参数初始化定时器TIM3
	
	//第五步，初始化输出比较参数，设置通道1和2
	TIM_OCInitTypeStrue.TIM_OCMode = TIM_OCMode_PWM1; //PWM模式1，当定时器计数小于TIM_Pulse时，定时器对应IO输出有效电平
	TIM_OCInitTypeStrue.TIM_OCPolarity = TIM_OCNPolarity_High; //输出有效电平为高电平
	TIM_OCInitTypeStrue.TIM_OutputState = TIM_OutputState_Enable; //使能PWM输出
	TIM_OC1Init(TIM3, &TIM_OCInitTypeStrue); //根TIM_OCInitTypeStrue参数初始化定时器3通道1
	TIM_OC2Init(TIM3, &TIM_OCInitTypeStrue); //根TIM_OCInitTypeStrue参数初始化定时器3通道2
	
//	TIM_CtrlPWMOutputs(TIM3,ENABLE);	//MOE 主输出使能

	//第六步，使能预装载寄存器
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable); //CH1预装载使能,使能后改变TIM_Pulse(即PWM)的值立刻生效，不使能则下个周期生效
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable); //CH2预装载使能,使能后改变TIM_Pulse(即PWM)的值立刻生效，不使能则下个周期生效
	
	TIM_ARRPreloadConfig(TIM3, ENABLE); //TIM3预装载使能
	
	//使能定时器
	TIM_Cmd(TIM3, ENABLE); 
	Tb6612_Init();
}

/**************************************************************************
函数功能：设置TIM3通道2的PWM值
入口参数：PWM值
返回  值：无
**************************************************************************/
void SetPWM(int pwm)
{
  if(pwm >= 0)//pwm>=0 (BIN1, BIN2)=(0, 1) 正转 顺时针
  { 
	  PBout(14) = 1; //BIN1=1
	  PBout(15) = 0; //BIN2=0
	  TIM_SetCompare2(TIM3, pwm);
  }
  else if(pwm < 0)//pwm<0 (BIN1, BIN2)=(1, 0) 反转 逆时针
  {
	  PBout(14) = 0; //BIN1=0
	  PBout(15) = 1; //BIN2=1
      TIM_SetCompare2(TIM3, -pwm);
  }
}


/**************************************************************************
函数功能：设置TIM3通道1的PWM值
入口参数：PWM值
返回  值：无
**************************************************************************/
//void SetPWM(int pwm)
//{
//  if(pwm >= 0)//pwm>=0 (BIN1, BIN2)=(0, 1) 正转 顺时针
//  { 
//	  PBout(13) = 1; //AIN1=1
//	  PBout(12) = 0; //AIN2=0
//	  TIM_SetCompare1(TIM3, pwm);
//  }
//  else if(pwm < 0)//pwm<0 (BIN1, BIN2)=(1, 0) 反转 逆时针
//  {
//	  PBout(13) = 0; //AIN1=0
//	  PBout(12) = 1; //AIN2=1
//      TIM_SetCompare1(TIM3, -pwm);
//  }
//}

