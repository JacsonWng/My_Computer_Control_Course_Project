#ifndef __TIM3_PWM_H 
#define __TIM3_PWM_H 
#include "sys.h"

void TIM3_PWM_Init(u16 arr, u16 psc);
void Tb6612_Init(void);
void SetPWM(int pwm);

#endif //定义完毕，或者引用过头文件到达这一步
