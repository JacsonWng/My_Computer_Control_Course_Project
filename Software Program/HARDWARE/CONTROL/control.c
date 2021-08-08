#include "control.h"
#include "encoder.h"

//外部变量 extern说明该变量已在其它文件定义
extern int   TargetVelocity, Encoder,PWM; //目标速度、编码器读数、PWM控制变量
extern float Velcity_Kp,  Velcity_Ki,  Velcity_Kd; //相关速度PID参数
extern int   MortorRun;  //允许电机控制标志位

/**************************************************************************
函数功能：TIM4中断服务函数,定时读取编码器数值并进行速度闭环控制,每10ms进入一次
入口参数：无
返回  值：无
**************************************************************************/
void TIM4_IRQHandler()
{
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) == 1) //当发生中断时状态寄存器(TIMx_SR)的bit0会被硬件置
	{
		Encoder = Read_Encoder();   //读取当前编码器单位时间计数，即速度
		
//		Encoder = Get_Motor_Speed();//计算的实际转速
		if(MortorRun) //如果按键按下，运行电机控制程序
		{
			PWM = Velocity_FeedbackControl(TargetVelocity, Encoder); //速度环闭环控制
			Xianfu_Pwm();//PWM限幅
			SetPWM(PWM); //设置PWM
		}
		else PWM = 0, SetPWM(PWM); //如果按键再次按下，电机停止
			
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update); //状态寄存器(TIMx_SR)的bit0置0
	}
}

/**************************************************************************
函数功能：速度闭环PID控制(实际为PI控制)
入口参数：目标速度 当前速度
返回  值：速度控制值
根据增量式离散PID公式 
ControlVelocity += Kp[e(k)-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
ControlVelocity代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
ControlVelocity += Kp[e(k)-e(k-1)]+Ki*e(k)
**************************************************************************/
int Bias;  //定义相关变量
int Velocity_FeedbackControl(double TargetVelocity, double CurrentVelocity)
{
		static double ControlVelocity, Last_bias; //静态变量，函数调用结束后其值依然存在
		Bias = TargetVelocity - CurrentVelocity; //求速度偏差
		
		ControlVelocity += Velcity_Kp*(Bias-Last_bias) + Velcity_Ki*Bias;  //增量式PI控制器
                                                                   //Velcity_Kp*(Bias-Last_bias) 作用为限制加速度
	                                                                 //Velcity_Ki*Bias             速度控制值由Bias不断积分得到 偏差越大加速度越大
		Last_bias = Bias;	
		return ControlVelocity; //返回速度控制值
}








/**************************************************************************
函数功能：TIM4中断服务函数,定时读取编码器数值并进行速度闭环控制,每10ms进入一次
入口参数：无
返回  值：无
**************************************************************************/
//void TIM4_IRQHandler()
//{
//	if(TIM_GetITStatus(TIM4, TIM_IT_Update) == 1) //当发生中断时状态寄存器(TIMx_SR)的bit0会被硬件置
//	{
////		Encoder = Read_Encoder();   //读取当前编码器单位时间计数，即速度
//		
//		Encoder = Get_Motor_Speed();//计算的实际转速
//		if(MortorRun) //如果按键按下，运行电机控制程序
//		{
//			PWM = Position_PID(TargetVelocity, Encoder); //速度环闭环控制;
//			Xianfu_Pwm();
//			SetPWM(PWM); //设置PWM
//		}
//		else PWM = 0, SetPWM(PWM); //如果按键再次按下，电机停止
//			
//		TIM_ClearITPendingBit(TIM4, TIM_IT_Update); //状态寄存器(TIMx_SR)的bit0置0
//	}
//}

/**************************************************************************
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(void) //限制PWM幅度的函数
{
	int Amplitude = 989;  //===PWM满幅是999
	if(PWM < -Amplitude)  PWM = -Amplitude;
	if(PWM > Amplitude)   PWM =  Amplitude;
}

/**************************************************************************
函数功能：限制积分赋值 
入口参数：无
返回  值：无
**************************************************************************/
float Integral_bias;
void Xianfu_Integral(void) //限制积分幅度的函数
{
	int Amplitude = 500;  //积分最大赋值
	if(Integral_bias < -Amplitude)  Integral_bias = -Amplitude;
	if(Integral_bias > Amplitude)   Integral_bias =  Amplitude;
}

/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int abs(int a) //取绝对值
{ 		   
	 int temp;
	 if(a < 0) temp = -a;  
	 else temp = a;
	 return temp;
}

/**************************************************************************
函数功能：位置式PID控制器
入口参数：编码器测量位置信息，目标位置
返回  值：电机PWM
根据位置式离散PID公式 
pwm = Kp*e(k)+Ki*∑e(k)+Kd[e(k)-e(k-1)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  
∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,...,k;
pwm代表输出
**************************************************************************/
int Bias;
int Position_PID(int Target,int Encoder)
{ 	
	 float Position_Kp = 20,Position_Ki = 5,Position_Kd = 0;
	 static float pwm,Last_Bias;
	 Bias = Target - Encoder;  
	 Integral_bias += Bias;	
     Xianfu_Integral();	//积分限幅	//求出偏差的积分
	 pwm = Position_Kp*Bias + Position_Ki*Integral_bias + Position_Kd*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias = Bias;                                       //保存上一次偏差 
	 return pwm;                                           //增量输出
}

/**************************************************************************
函数功能：计算编码器测得的实际转速
入口参数：无
返回  值：测得的转速
一圈的脉冲数；1560个
**************************************************************************/
int Get_Motor_Speed(void)
{
	double Speed = 0;
	Speed = (Read_Encoder()*1000.0*100/1560.0) / 30.0;//转速单位，r/s
	return Speed;
}
