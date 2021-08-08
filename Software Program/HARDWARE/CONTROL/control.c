#include "control.h"
#include "encoder.h"

//�ⲿ���� extern˵���ñ������������ļ�����
extern int   TargetVelocity, Encoder,PWM; //Ŀ���ٶȡ�������������PWM���Ʊ���
extern float Velcity_Kp,  Velcity_Ki,  Velcity_Kd; //����ٶ�PID����
extern int   MortorRun;  //���������Ʊ�־λ

/**************************************************************************
�������ܣ�TIM4�жϷ�����,��ʱ��ȡ��������ֵ�������ٶȱջ�����,ÿ10ms����һ��
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM4_IRQHandler()
{
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) == 1) //�������ж�ʱ״̬�Ĵ���(TIMx_SR)��bit0�ᱻӲ����
	{
		Encoder = Read_Encoder();   //��ȡ��ǰ��������λʱ����������ٶ�
		
//		Encoder = Get_Motor_Speed();//�����ʵ��ת��
		if(MortorRun) //����������£����е�����Ƴ���
		{
			PWM = Velocity_FeedbackControl(TargetVelocity, Encoder); //�ٶȻ��ջ�����
			Xianfu_Pwm();//PWM�޷�
			SetPWM(PWM); //����PWM
		}
		else PWM = 0, SetPWM(PWM); //��������ٴΰ��£����ֹͣ
			
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update); //״̬�Ĵ���(TIMx_SR)��bit0��0
	}
}

/**************************************************************************
�������ܣ��ٶȱջ�PID����(ʵ��ΪPI����)
��ڲ�����Ŀ���ٶ� ��ǰ�ٶ�
����  ֵ���ٶȿ���ֵ
��������ʽ��ɢPID��ʽ 
ControlVelocity += Kp[e(k)-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
ControlVelocity�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
ControlVelocity += Kp[e(k)-e(k-1)]+Ki*e(k)
**************************************************************************/
int Bias;  //������ر���
int Velocity_FeedbackControl(double TargetVelocity, double CurrentVelocity)
{
		static double ControlVelocity, Last_bias; //��̬�������������ý�������ֵ��Ȼ����
		Bias = TargetVelocity - CurrentVelocity; //���ٶ�ƫ��
		
		ControlVelocity += Velcity_Kp*(Bias-Last_bias) + Velcity_Ki*Bias;  //����ʽPI������
                                                                   //Velcity_Kp*(Bias-Last_bias) ����Ϊ���Ƽ��ٶ�
	                                                                 //Velcity_Ki*Bias             �ٶȿ���ֵ��Bias���ϻ��ֵõ� ƫ��Խ����ٶ�Խ��
		Last_bias = Bias;	
		return ControlVelocity; //�����ٶȿ���ֵ
}








/**************************************************************************
�������ܣ�TIM4�жϷ�����,��ʱ��ȡ��������ֵ�������ٶȱջ�����,ÿ10ms����һ��
��ڲ�������
����  ֵ����
**************************************************************************/
//void TIM4_IRQHandler()
//{
//	if(TIM_GetITStatus(TIM4, TIM_IT_Update) == 1) //�������ж�ʱ״̬�Ĵ���(TIMx_SR)��bit0�ᱻӲ����
//	{
////		Encoder = Read_Encoder();   //��ȡ��ǰ��������λʱ����������ٶ�
//		
//		Encoder = Get_Motor_Speed();//�����ʵ��ת��
//		if(MortorRun) //����������£����е�����Ƴ���
//		{
//			PWM = Position_PID(TargetVelocity, Encoder); //�ٶȻ��ջ�����;
//			Xianfu_Pwm();
//			SetPWM(PWM); //����PWM
//		}
//		else PWM = 0, SetPWM(PWM); //��������ٴΰ��£����ֹͣ
//			
//		TIM_ClearITPendingBit(TIM4, TIM_IT_Update); //״̬�Ĵ���(TIMx_SR)��bit0��0
//	}
//}

/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(void) //����PWM���ȵĺ���
{
	int Amplitude = 989;  //===PWM������999
	if(PWM < -Amplitude)  PWM = -Amplitude;
	if(PWM > Amplitude)   PWM =  Amplitude;
}

/**************************************************************************
�������ܣ����ƻ��ָ�ֵ 
��ڲ�������
����  ֵ����
**************************************************************************/
float Integral_bias;
void Xianfu_Integral(void) //���ƻ��ַ��ȵĺ���
{
	int Amplitude = 500;  //�������ֵ
	if(Integral_bias < -Amplitude)  Integral_bias = -Amplitude;
	if(Integral_bias > Amplitude)   Integral_bias =  Amplitude;
}

/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int abs(int a) //ȡ����ֵ
{ 		   
	 int temp;
	 if(a < 0) temp = -a;  
	 else temp = a;
	 return temp;
}

/**************************************************************************
�������ܣ�λ��ʽPID������
��ڲ���������������λ����Ϣ��Ŀ��λ��
����  ֵ�����PWM
����λ��ʽ��ɢPID��ʽ 
pwm = Kp*e(k)+Ki*��e(k)+Kd[e(k)-e(k-1)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  
��e(k)����e(k)�Լ�֮ǰ��ƫ����ۻ���;����kΪ1,2,...,k;
pwm�������
**************************************************************************/
int Bias;
int Position_PID(int Target,int Encoder)
{ 	
	 float Position_Kp = 20,Position_Ki = 5,Position_Kd = 0;
	 static float pwm,Last_Bias;
	 Bias = Target - Encoder;  
	 Integral_bias += Bias;	
     Xianfu_Integral();	//�����޷�	//���ƫ��Ļ���
	 pwm = Position_Kp*Bias + Position_Ki*Integral_bias + Position_Kd*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias = Bias;                                       //������һ��ƫ�� 
	 return pwm;                                           //�������
}

/**************************************************************************
�������ܣ������������õ�ʵ��ת��
��ڲ�������
����  ֵ����õ�ת��
һȦ����������1560��
**************************************************************************/
int Get_Motor_Speed(void)
{
	double Speed = 0;
	Speed = (Read_Encoder()*1000.0*100/1560.0) / 30.0;//ת�ٵ�λ��r/s
	return Speed;
}
