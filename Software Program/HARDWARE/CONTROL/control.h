#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

int Velocity_FeedbackControl(double TargetVelocity, double CurrentVelocity);//����ʽPID
int Position_PID (int Target,int Encoder);//λ��ʽPID
void Xianfu_Pwm(void);//PWM���Ʒ��ȵĺ���
void Xianfu_Integral(void);//�������Ʒ��ȵĺ���
int abs(int a); //ȡ����ֵ
int Get_Motor_Speed(void);//�����������õ�ʵ��ת��

#endif
