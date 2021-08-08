#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

int Velocity_FeedbackControl(double TargetVelocity, double CurrentVelocity);//增量式PID
int Position_PID (int Target,int Encoder);//位置式PID
void Xianfu_Pwm(void);//PWM限制幅度的函数
void Xianfu_Integral(void);//积分限制幅度的函数
int abs(int a); //取绝对值
int Get_Motor_Speed(void);//计算编码器测得的实际转速

#endif
