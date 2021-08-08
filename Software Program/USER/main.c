/***********************************************
ֱ���������ҵ
author:wang xiao
date��2021-06-13
***********************************************/

#include "stm32f10x.h"
#include "delay.h"
#include "usart.h"			
#include "tb6612.h"
#include "encoder.h"
#include "control.h"
#include "oled.h"
#include "key.h"
#include "data_scope.h"//��λ��
#include "sys.h"

//PID����_����
int   TargetVelocity = 50, Encoder, PWM;  //Ŀ���ٶȡ�������������PWM���Ʊ���
float Velcity_Kp = 20,  Velcity_Ki = 5,  Velcity_Kd = 0; //����ٶ�PID����
int   MortorRun;  //���������Ʊ�־λ
extern int Bias;//ƫ��

//��λ��_����
unsigned char i;          //��������
unsigned char Send_Count; //������Ҫ���͵����ݸ���
static float a;

/**************************************************************************
�������ܣ�������
��ڲ�������
����  ֵ����
**************************************************************************/
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init();//�ӳٺ�����ʼ��
	Stm32_Clock_Init(9);            //ϵͳʱ������
	TIM3_PWM_Init(999, 71);//������������ʼ�� ʹ�ö�ʱ��3������Ƶ��PWMƵ��Ϊ72000000/72000=1kHz,����Ϊ1ms 
	Encoder_Init();//��ʱ��2������ģʽ��ȡ��ʼ��
	EncoderRead_TIM4(7199, 99);//10ms��ȡһ�α�����(��100HZ)��������ٱ�Ϊ30����������������13��AB˫����ϵõ�4��Ƶ��
							   //��ת1Ȧ����������Ϊ30*13*4=1560�����ת��=Encoder*100/1560r/s ʹ�ö�ʱ��4
	KEY_Init();
	OLED_Init();                 //OLED��ʾ����ʼ��
	uart_init(4800);           //����1��ʼ��	 
	delay_ms(5000);              //�ӳٵȴ���ʼ�����
	while(1)
	{
		//�������
		MortorRun = 1;
		
		//OLED��ʾ
		Oled_Show();
		
		//��λ����ʾ
		a += 0.1;
		if(a > 3.14) a = -3.14; 
		
		//ͨ����ʾ
		DataScope_Get_Channel_Data(TargetVelocity, 1);//CH1��ʾת������ֵ
		DataScope_Get_Channel_Data(Encoder, 2);//CH2��ʾ����������ֵ
		DataScope_Get_Channel_Data(PWM, 3);//CH3��ʾPWMֵ
		DataScope_Get_Channel_Data(Bias, 4);//CH4��ʾƫ��ֵ
		
		Send_Count = DataScope_Data_Generate(10);
		for(i = 0; i < Send_Count; i++) 
		{
			while((USART1->SR&0X40) == 0);  
			USART1->DR = DataScope_OutPut_Buffer[i]; 
		}
		delay_ms(50); //20HZ
	}
}

