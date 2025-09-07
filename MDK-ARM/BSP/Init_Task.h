#ifndef __iNIT_tASK_H
#define __iNIT_tASK_H

#include "main.h"




void Init_Task(void);
void Init_Open_Control(void);
void pid_init(void);
void Init_PWM_Write_Open(void);
void Init_Cap_Low_Voltage_Protection(void);


extern  uint32_t ADC1_RESULT[10];
extern int tim_1;




#endif
