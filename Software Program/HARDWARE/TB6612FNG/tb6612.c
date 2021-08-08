#include "TB6612.h"  

/**************************************************************************
�������ܣ�TB6612����������ų�ʼ������
��ڲ�������
����  ֵ����
**************************************************************************/
void Tb6612_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; //����һ�����ų�ʼ���Ľṹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ��GPIOBʱ�ӣ�GPIOB������APB2ʱ���£���STM32��ʹ��IO��ǰ��Ҫʹ�ܶ�Ӧʱ��
	
	//��ʼ�����1�͵��2�ķ����������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure); //��ʼ������GPIOB4
	
	GPIO_ResetBits(GPIOB, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);//��ʼ����ƽ
}




/**************************************************************************
�������ܣ�TB6612��PWM��ʼ������
��ڲ�������ʱ��3��������,��ʱ��3Ԥ��Ƶϵ��
����  ֵ����
**************************************************************************/
void TIM3_PWM_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure; //����һ�����ų�ʼ���Ľṹ��
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue; //����һ����ʱ�жϵĽṹ��	
	TIM_OCInitTypeDef TIM_OCInitTypeStrue; //����һ��PWM����ȽϵĽṹ��
	
	//��һ����ʹ�ܶ�ʱ��3ʱ�Ӻ�GPIOB��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʹ��ͨ�ö�ʱ��3ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ��GPIOBʱ�ӣ�GPIOB������APB2ʱ���£���STM32��ʹ��IO��ǰ��Ҫʹ�ܶ�Ӧʱ��
	
	//�ڶ�������GPIOB��ΪPWM���ʹ�ã�����ҪҪʹ�ö˿���ӳ�俪��AFIOʱ�ӣ�ͬʱ������ӳ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);//ʹ��AFIOʱ��,��ӳ�����ʹ��AFIOʱ��
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);//������ӳ��

	//����������ʼ��IO��Ϊ���ù������,�������������TB6612PWM�������_PWMA��PWMB
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;//PB4��PB5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //��ʼ��IO��Ϊ���ù������,�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //�������������ٶ�Ϊ50MHZ
    GPIO_Init(GPIOB, &GPIO_InitStructure); //��ʼ������GPIOB4
	
	//���Ĳ�����ʼ����ʱ����arr��psc
	TIM_TimeBaseInitStrue.TIM_Period = arr; //����ģʽΪ���ϼ���ʱ����ʱ����0��ʼ����������������arrʱ������ʱ�жϷ�����
	TIM_TimeBaseInitStrue.TIM_Prescaler = psc; //Ԥ��Ƶϵ��������ÿһ��������ʱ��
	TIM_TimeBaseInitStrue.TIM_CounterMode = TIM_CounterMode_Up; //����ģʽ�����ϼ���
	TIM_TimeBaseInitStrue.TIM_ClockDivision = TIM_CKD_DIV1; //һ�㲻ʹ�ã�Ĭ��TIM_CKD_DIV1
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStrue); //����TIM_TimeBaseInitStrue�Ĳ�����ʼ����ʱ��TIM3
	
	//���岽����ʼ������Ƚϲ���������ͨ��1��2
	TIM_OCInitTypeStrue.TIM_OCMode = TIM_OCMode_PWM1; //PWMģʽ1������ʱ������С��TIM_Pulseʱ����ʱ����ӦIO�����Ч��ƽ
	TIM_OCInitTypeStrue.TIM_OCPolarity = TIM_OCNPolarity_High; //�����Ч��ƽΪ�ߵ�ƽ
	TIM_OCInitTypeStrue.TIM_OutputState = TIM_OutputState_Enable; //ʹ��PWM���
	TIM_OC1Init(TIM3, &TIM_OCInitTypeStrue); //��TIM_OCInitTypeStrue������ʼ����ʱ��3ͨ��1
	TIM_OC2Init(TIM3, &TIM_OCInitTypeStrue); //��TIM_OCInitTypeStrue������ʼ����ʱ��3ͨ��2
	
//	TIM_CtrlPWMOutputs(TIM3,ENABLE);	//MOE �����ʹ��

	//��������ʹ��Ԥװ�ؼĴ���
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable); //CH1Ԥװ��ʹ��,ʹ�ܺ�ı�TIM_Pulse(��PWM)��ֵ������Ч����ʹ�����¸�������Ч
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable); //CH2Ԥװ��ʹ��,ʹ�ܺ�ı�TIM_Pulse(��PWM)��ֵ������Ч����ʹ�����¸�������Ч
	
	TIM_ARRPreloadConfig(TIM3, ENABLE); //TIM3Ԥװ��ʹ��
	
	//ʹ�ܶ�ʱ��
	TIM_Cmd(TIM3, ENABLE); 
	Tb6612_Init();
}

/**************************************************************************
�������ܣ�����TIM3ͨ��2��PWMֵ
��ڲ�����PWMֵ
����  ֵ����
**************************************************************************/
void SetPWM(int pwm)
{
  if(pwm >= 0)//pwm>=0 (BIN1, BIN2)=(0, 1) ��ת ˳ʱ��
  { 
	  PBout(14) = 1; //BIN1=1
	  PBout(15) = 0; //BIN2=0
	  TIM_SetCompare2(TIM3, pwm);
  }
  else if(pwm < 0)//pwm<0 (BIN1, BIN2)=(1, 0) ��ת ��ʱ��
  {
	  PBout(14) = 0; //BIN1=0
	  PBout(15) = 1; //BIN2=1
      TIM_SetCompare2(TIM3, -pwm);
  }
}


/**************************************************************************
�������ܣ�����TIM3ͨ��1��PWMֵ
��ڲ�����PWMֵ
����  ֵ����
**************************************************************************/
//void SetPWM(int pwm)
//{
//  if(pwm >= 0)//pwm>=0 (BIN1, BIN2)=(0, 1) ��ת ˳ʱ��
//  { 
//	  PBout(13) = 1; //AIN1=1
//	  PBout(12) = 0; //AIN2=0
//	  TIM_SetCompare1(TIM3, pwm);
//  }
//  else if(pwm < 0)//pwm<0 (BIN1, BIN2)=(1, 0) ��ת ��ʱ��
//  {
//	  PBout(13) = 0; //AIN1=0
//	  PBout(12) = 1; //AIN2=1
//      TIM_SetCompare1(TIM3, -pwm);
//  }
//}

