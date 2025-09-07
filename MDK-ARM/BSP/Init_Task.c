/***
 *
 *                                                    __----~~~~~~~~~~~------___
 *                                   .  .   ~~//====......          __--~ ~~
 *                   -.            \_|//     |||\\  ~~~~~~::::... /~
 *                ___-==_       _-~o~  \/    |||  \\            _/~~-
 *        __---~~~.==~||\=_    -_--~/_-~|-   |\\   \\        _/~
 *    _-~~     .=~    |  \\-_    '-~7  /-   /  ||    \      /
 *  .~       .~       |   \\ -_    /  /-   /   ||      \   /
 * /  ____  /         |     \\ ~-_/  /|- _/   .||       \ /
 * |~~    ~~|--~~~~--_ \     ~==-/   | \~--===~~        .\
 *          '         ~-|      /|    |-~\~~       __--~~
 *                      |-~~-_/ |    |   ~\_   _-~            /\
 *                           /  \     \__   \/~                \__
 *                       _--~ _/ | .-~~____--~-/                  ~~==.
 *                      ((->/~   '.|||' -_|    ~~-/ ,              . _||
 *                                 -_     ~\      ~~---l__i__i__i--~~_/
 *                                 _-~-__   ~)  \--______________--~~
 *                               //.-~~~-~_--~- |-------~~~~~~~~
 *                                      //.-~~~--\                         /������β������/
 */



#include "Init_Task.h"
#include "main.h"
#include "hrtim.h"
#include "adc.h"
#include "BSP_CAN_FD.h"
#include "tim.h"
#include "lib.h"
#include "PWM_Control.h"
#include  "oled.h"
#include  "task.h"



/**********************************************************************************
  * @brief                                       ��ʼ��
  * @author                                      Z
  * @param                                       ��ʼ������
  * @retval                                      ������
  *********************************************************************************/
uint32_t ADC1_RESULT[10] ; //adc��ֵԭʼ����
 int tim_1;
void Init_Task()
{
	  DWT_Init(240);
	  //�˲���ʼ��                    ��������
	first_order_filter_init(&ta1, 0.3f);		
	first_order_filter_init(&tb1, 0.3f);
	first_order_filter_init(&power, 0.8f);//û�ù�
	  
	  //�������˲���ʼ��
	  KalmanCreate(&Out_I_Cap,0.5f,1.5f);
	  KalmanCreate(&In_Power,0.5f,2.5f);
	  
	  //oled��ʼ��
	MX_OLED_Init();
	  //LED��ʾ
	  OLED_ShowStr(0 ,0,(unsigned char*)"Writing..."       ,2);
	
		//fdcan��ʼ��
	  FDCAN_Init();
	  
	  //������ʱ���жϣ�1msһ��
	  HAL_TIM_Base_Start_IT(&htim3);
	  
	  //pwm��ʼ��
    //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	  
	  //ADC��ʼ��
	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_RESULT, 5);//����ADC1���� DMA���ݴ��Ͳ������������ѹ����
	 //1.�ʹ������������֮����
	  //pid��ʼ��
		pid_init();
		
		//������ʼ��
	  hrpwm.I_Out=0;
		hrpwm.TA1=360;
	  hrpwm.TB1=2280;//2.ΪʲôҪ��������ʼֵ��
		hrpwm.I_aim=1000;//3.���������ֵ���ת��TA1,TB1��
		hrpwm.I_ramp=500;
	 	hrpwm.Charge_sign=1;
	  hrpwm.Power_Out=1000;
	  can_data.power_buff=60;
	  can_data.power_limit=50;//
	  can_data.super_cap_sign=1;
		can_data.Model=1;
		can_data.charge_power=1;
		can_data.charge_sign1=1;//���
		//������
		Init_Open_Control(); 
}



/******************************************************************************************************
  * @brief                                       ������
  * @author                                      Z
  * @param                                       ����������������޶ȵĿ��ƿ���ʱ�ĵ���״̬
  * @param                                       ��ѹ���ͱ������ܿ�Ӳ����ѹ����̫�߹���������������
  * @retval                                      ������
  *****************************************************************************************************/
void Init_Open_Control()
{
	
	  //�ȴ�adc�ɼ����Ƚ�׼ȷ�ĵ�ѹ����ֵ
	  tim_1=HAL_GetTick();
		while(HAL_GetTick()-tim_1<2000) { }//��2s
	  
    //hrtim��ʼ��,����pwm���
		HAL_HRTIM_WaveformOutputStart(&hhrtim, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //����PWM�����PWM��ʱ��
		HAL_HRTIM_WaveformOutputStart(&hhrtim, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //����PWM�����PWM��ʱ��
	  HAL_HRTIM_WaveformCounterStart(&hhrtim, HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_MASTER);
		
		
	  //pwm�ĵ�һ֡�������֤�������ѹ�ϵ�����������ӽ�����
    Init_PWM_Write_Open();                       //һ����������һ֡����
	  
	  //�������ѹС��7Vʱ������7V�ٽ���pid�ջ�
	  Init_Cap_Low_Voltage_Protection();//������������2�����ݣ��Ƿ��ǵ�ѹ��̫���Ǽ��δθ�ֵ���µ���̫��mos���ջ�
													//��ѹ��Сһ��
		//������ʱ���жϣ�1msһ�Σ�����pid�ջ�����
	  HAL_TIM_Base_Start_IT(&htim4);
		HAL_TIM_Base_Start_IT(&htim5);
		
		hrpwm.time_init=HAL_GetTick();
	  
}



//pid��ʼ��
void pid_init()
{
	  //�������ƻ�
		Out_Power_Loop.kp_a=1400.1f;
		Out_Power_Loop.ki_a=50.0f;//370
		Out_Power_Loop.kd_a=0;
	  
	  //����������
	  Out_Buff_Loop.kp_a=6000.1f;
		Out_Buff_Loop.ki_a=10.0f;
		Out_Buff_Loop.kd_a=0;
	  
#if	Speed_Loop ==0 //λ��ʽ
	  //��ѹ��
	  Out_U_Loop.kp_a=0.07f;
		Out_U_Loop.ki_a=0.0003f;
		Out_U_Loop.kd_a=0;
		
	  //������
		Out_I_Loop.kp_a=0.03f;//0.03f
		Out_I_Loop.ki_a=0.005f;
		Out_I_Loop.kd_a=0;
#else              //����ʽ
	  //��ѹ��
	  Out_U_Loop.kp_s=1.0f;
		Out_U_Loop.ki_s=0.001f;
		Out_U_Loop.kd_s=0;
		
	  //������
		Out_I_Loop.kp_s=0.1f;
		Out_I_Loop.ki_s=0.0003;
		Out_I_Loop.kd_s=0;	
#endif
	  

}



//pwm�ĵ�һ�θ�������֤�����ѹ�ϵ���ݳ�ŵ��ٶȶ��ӽ�����
void Init_PWM_Write_Open()
{
	  //ͨ�������ѹ�������ѹ�ı�ֵ����pwm���
		#if Speed_Loop ==1
		if(adc.U_Out_Cap*1.0f/adc.U_In_Vcc<=1.0f)
		{
			Out_I_Loop.out_speed_total=adc.U_Out_Cap*2400/adc.U_In_Vcc-1920;
			Out_U_Loop.out_speed_total=adc.U_Out_Cap*2400/adc.U_In_Vcc-1920;
		}
		else if(adc.U_Out_Cap*1.0f/adc.U_In_Vcc>1.0f)
		{
			Out_I_Loop.out_speed_total=2000-1920;
			Out_U_Loop.out_speed_total=Out_I_Loop.out_speed_total;
		}
		#else
    if(adc.U_Out_Cap*1.0f/adc.U_In_Vcc<=1.0f)//���С��24v
		{
			hrpwm.I_Out=adc.U_Out_Cap*2400/adc.U_In_Vcc;//-340
		}
		else if(adc.U_Out_Cap*1.0f/adc.U_In_Vcc>1.0f)
		{
			hrpwm.I_Out=2000;
		}
		if(hrpwm.I_Out<0)  hrpwm.I_Out=0;
		#endif
		
		//����pwm
		if(hrpwm.I_Out<1920)//buckģʽ
		{
				 hrpwm.TA1=hrpwm.I_Out+2400*0.03f;//��0.8v�����������0
				 hrpwm.TB1=2400*0.95f;
		}
		else if(hrpwm.I_Out>=1920)//boostģʽ
		{
				 hrpwm.TA1=2400*0.95f;
				 hrpwm.TB1=4200-hrpwm.I_Out;
		}
		
		//pwm����
		hrpwm_tim( );//����ĸ�ֵTA1TB1����ûɶ��
		
		//��������ʱ200ms
		tim_1=HAL_GetTick();
		while(HAL_GetTick()-tim_1<200) { oled_task();  Protect_Function(); }

		
}



//�����ѹ����
void Init_Cap_Low_Voltage_Protection()
{
	  if(adc.U_Out_Cap<6500)
		{
			   //����pwm
				 hrpwm.I_Out=-130;
         
			   //����pwm
				 if(hrpwm.I_Out<1920)//buckģʽ
				 {
						hrpwm.TA1=hrpwm.I_Out+2400*0.15f;
						hrpwm.TB1=2400*0.95f;
				 }
				 else if(hrpwm.I_Out>=1920)//boostģʽ
				 {
						hrpwm.TA1=2400*0.95f;
						hrpwm.TB1=4200-hrpwm.I_Out;
				 }			
				 //����pwm
				 hrpwm_tim( );
				 //����ѹ����5V���ҽ���
				 tim_1=HAL_GetTick();//��ʱ�˳���15s����ֹ����ʹ�����
				 while(adc.U_Out_Cap<4500&&HAL_GetTick()-tim_1<15000) { oled_task();  Protect_Function();}
				 
				 //����pwm
				 hrpwm.I_Out=180;
                
			   //����pwm
				 if(hrpwm.I_Out<1920)//buckģʽ
				 {
						hrpwm.TA1=hrpwm.I_Out+2400*0.15f;
						hrpwm.TB1=2400*0.95f;
				 }
				 else if(hrpwm.I_Out>=1920)//boostģʽ
				 {
						hrpwm.TA1=2400*0.95f;
						hrpwm.TB1=4200-hrpwm.I_Out;
				 }			
				 //����pwm
				 hrpwm_tim( );
				 //����ѹ����7V���ҽ���
				 tim_1=HAL_GetTick();//��ʱ�˳���15s����ֹ����ʹ�����
				 while(adc.U_Out_Cap<6700&&HAL_GetTick()-tim_1<15000) { oled_task();  Protect_Function();}
				 
				 
		}
	
}





