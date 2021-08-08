/***********************************************
直流电机大作业
author:wang xiao
date：2021-06-13
***********************************************/

#include "stm32f10x.h"
#include "delay.h"
#include "usart.h"			
#include "tb6612.h"
#include "encoder.h"
#include "control.h"
#include "oled.h"
#include "key.h"
#include "data_scope.h"//上位机
#include "sys.h"

//PID控制_变量
int   TargetVelocity = 50, Encoder, PWM;  //目标速度、编码器读数、PWM控制变量
float Velcity_Kp = 20,  Velcity_Ki = 5,  Velcity_Kd = 0; //相关速度PID参数
int   MortorRun;  //允许电机控制标志位
extern int Bias;//偏差

//上位机_变量
unsigned char i;          //计数变量
unsigned char Send_Count; //串口需要发送的数据个数
static float a;

/**************************************************************************
函数功能：主函数
入口参数：无
返回  值：无
**************************************************************************/
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init();//延迟函数初始化
	Stm32_Clock_Init(9);            //系统时钟设置
	TIM3_PWM_Init(999, 71);//电机驱动外设初始化 使用定时器3，不分频，PWM频率为72000000/72000=1kHz,周期为1ms 
	Encoder_Init();//定时器2编码器模式读取初始化
	EncoderRead_TIM4(7199, 99);//10ms读取一次编码器(即100HZ)，电机减速比为30，霍尔编码器精度13，AB双相组合得到4倍频，
							   //则转1圈编码器读数为30*13*4=1560，电机转速=Encoder*100/1560r/s 使用定时器4
	KEY_Init();
	OLED_Init();                 //OLED显示屏初始化
	uart_init(4800);           //串口1初始化	 
	delay_ms(5000);              //延迟等待初始化完成
	while(1)
	{
		//电机运行
		MortorRun = 1;
		
		//OLED显示
		Oled_Show();
		
		//上位机显示
		a += 0.1;
		if(a > 3.14) a = -3.14; 
		
		//通道显示
		DataScope_Get_Channel_Data(TargetVelocity, 1);//CH1显示转速期望值
		DataScope_Get_Channel_Data(Encoder, 2);//CH2显示编码器测量值
		DataScope_Get_Channel_Data(PWM, 3);//CH3显示PWM值
		DataScope_Get_Channel_Data(Bias, 4);//CH4显示偏差值
		
		Send_Count = DataScope_Data_Generate(10);
		for(i = 0; i < Send_Count; i++) 
		{
			while((USART1->SR&0X40) == 0);  
			USART1->DR = DataScope_OutPut_Buffer[i]; 
		}
		delay_ms(50); //20HZ
	}
}

