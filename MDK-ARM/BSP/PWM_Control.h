#ifndef __pWM_cONTROL_H
#define __pWM_cONTROL_H


#include "main.h"


#endif

typedef struct
{
		uint8_t Model;
		float TA1,TB1;
		int aim_voltage;
		int aim_electricity;
		uint8_t Pattern;
		float I_aim;
		float I_ramp;
		float I_Out;
		float CompareValue;
		float Power_Out,Loop_Out;
		float TA1_t,TB1_t;
	  float power_limit_ramp;
		uint8_t Charge_sign;
		int Super_Cap_Power;	
		int Power_Aim;
		float P_k;
   	int time;
//		uint8_t Ct_sign;
	  float TA1_ramp,TB1_ramp;
	  uint8_t Loop_Sign;
	  int Loop_cnt;
		int time_init;
		
		int fps;
	  
} hrpwm_t;

extern hrpwm_t hrpwm;


float Ramp_Function(void);
void Power_Limit_Loop(void);
void Magazine_Control(void);
void PWM_Task(void);
void Model_Switch(void);
void HRPWM_Control(void);
void hrpwm_tim(void);



