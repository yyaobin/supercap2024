#ifndef __tASK_H
#define __tASK_H
#include "main.h"


extern uint8_t sign;
extern int frequency;
extern int frequency1;
void ADC_Handle(void);
void Main_Task(void);
extern void Protect_Function(void);
void LED_Control(void);
void Transmit_Task(void);
void sent_data(int16_t data1, int16_t data2,int16_t data3,int16_t data4,int16_t data5);
void DWT_Init(uint32_t CPU_Freq_mHz);
float DWT_GetDeltaT(uint32_t *cnt_last);
#endif

