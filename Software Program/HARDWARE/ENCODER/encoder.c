#include "encoder.h" 

//�ⲿ���� extern˵���ı������������ļ�����
extern double   TargetVelocity, Encoder,PWM; //Ŀ���ٶȡ�������������PWM���Ʊ���
extern float Velcity_Kp,  Velcity_Ki,  Velcity_Kd; //����ٶ�PID����
extern int   MortorRun;  //���������Ʊ�־λ

/**************************************************************************
�������ܣ���������ʼ������
��ڲ�������
����  ֵ����
**************************************************************************/
void Encoder_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; //����һ�����ų�ʼ���Ľṹ��  
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;//����һ����ʱ����ʼ���Ľṹ��
	TIM_ICInitTypeDef TIM_ICInitStructure; //����һ����ʱ��������ģʽ��ʼ���Ľṹ��
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //ʹ��TIM2ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //ʹ��GPIOAʱ��
	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
//	GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE);
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;	//TIM2_CH1��TIM2_CH2,������ģʽֻ�����ڶ�ʱ����CH1��CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);	//����GPIO_InitStructure�Ĳ�����ʼ��GPIO

	TIM_TimeBaseStructure.TIM_Period = 0xffff; //�趨�������Զ���װֵ
	TIM_TimeBaseStructure.TIM_Prescaler = 0; // Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct�Ĳ�����ʼ����ʱ��TIM2
			
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //ʹ�ñ�����ģʽ3��CH1��CH2ͬʱ������Ϊ�ķ�Ƶ
	
	TIM_ICStructInit(&TIM_ICInitStructure); //��TIM_ICInitStruct �е�ÿһ��������ȱʡֵ����
	TIM_ICInitStructure.TIM_ICFilter = 10;  //�����˲�������
	TIM_ICInit(TIM2, &TIM_ICInitStructure); //��TIM_ICInitStructure������ʼ����ʱ��TIM2������ģʽ
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);//���TIM2�ĸ��±�־λ
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //�����ж�ʹ��
	TIM_SetCounter(TIM2,0); //��ʼ����ձ�������ֵ
	
	TIM_Cmd(TIM2, ENABLE); //ʹ�ܶ�ʱ��2
}

/**************************************************************************
�������ܣ���ȡTIM2��������ֵ����λʱ���ȡ����������
��ڲ�������
����  ֵ����
**************************************************************************/
int Read_Encoder(void)
{
	int Encoder_TIM;
	Encoder_TIM = TIM2->CNT; //��ȡ����
	if(Encoder_TIM > 0xEFFF) Encoder_TIM = Encoder_TIM-0xFFFF; //ת������ֵΪ�з����ֵ������0��ת��С��0��ת��
	                                                      //TIM2->CNT��ΧΪ0-0xffff����ֵΪ0��
	TIM2->CNT = 0; //��ȡ����������
	return Encoder_TIM; //����ֵ
}

/**************************************************************************
�������ܣ�TIM2�жϷ�����
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM2->SR & 0X0001)//����ж�
	{    				   				     	    	
	}				   
	TIM2->SR &= ~(1<<0);//����жϱ�־λ 	    
}

/**************************************************************************
�������ܣ�ͨ�ö�ʱ��4��ʼ��������
��ڲ������Զ���װ��ֵ Ԥ��Ƶϵ�� Ĭ�϶�ʱʱ��Ϊ72MHZʱ�����߹�ͬ������ʱ�ж�ʱ��
����  ֵ����
**************************************************************************/
void EncoderRead_TIM4(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue; //����һ����ʱ�жϵĽṹ��
	NVIC_InitTypeDef NVIC_InitStrue; //����һ���ж����ȼ���ʼ���Ľṹ��
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //ʹ��ͨ�ö�ʱ��4ʱ��
	
	TIM_TimeBaseInitStrue.TIM_Period = arr; //����ģʽΪ���ϼ���ʱ����ʱ����0��ʼ����������������arrʱ������ʱ�жϷ�����
	TIM_TimeBaseInitStrue.TIM_Prescaler = psc; //Ԥ��Ƶϵ��������ÿһ��������ʱ��
	TIM_TimeBaseInitStrue.TIM_CounterMode = TIM_CounterMode_Up; //����ģʽ�����ϼ���
	TIM_TimeBaseInitStrue.TIM_ClockDivision = TIM_CKD_DIV1; //һ�㲻ʹ�ã�Ĭ��TIM_CKD_DIV1
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStrue); //����TIM_TimeBaseInitStrue�Ĳ�����ʼ����ʱ��TIM4
	
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); //ʹ��TIM4�жϣ��ж�ģʽΪ�����жϣ�TIM_IT_Update
	
	NVIC_InitStrue.NVIC_IRQChannel = TIM4_IRQn; //����TIM4�ж�
	NVIC_InitStrue.NVIC_IRQChannelCmd = ENABLE; //�ж�ʹ��
	NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority = 1; //��ռ���ȼ�Ϊ1����ֵԽС���ȼ�Խ�ߣ�0�����ȼ����
	NVIC_InitStrue.NVIC_IRQChannelSubPriority = 1; //��Ӧ���ȼ�Ϊ1����ֵԽС���ȼ�Խ�ߣ�0�����ȼ����
	NVIC_Init(&NVIC_InitStrue); //����NVIC_InitStrue�Ĳ�����ʼ��VIC�Ĵ���������TIM4�ж�
	
	TIM_Cmd(TIM4, ENABLE); //ʹ�ܶ�ʱ��TIM4
}

