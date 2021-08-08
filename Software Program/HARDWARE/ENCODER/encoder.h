#ifndef __ENCODER_H 
#define __ENCODER_H 
#include "sys.h"
#include "TB6612.h"

void Encoder_Init(void); 
int Read_Encoder(void);
void EncoderRead_TIM4(u16 arr, u16 psc);


#endif
