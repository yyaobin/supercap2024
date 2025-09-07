/*
*                                         $,  $,     ,
*                                         "ss.$ss. .s��
*                                 ,     .ss$$$$$$$$$$s,
*                                 $. s$$$$$$$$$$$$$$`$$Ss
*                                 "$$$$$$$$$$$$$$$$$$o$$$       ,
*                                s$$$$$$$$$$$$$$$$$$$$$$$$s,  ,s
*                               s$$$$$$$$$"$$$$$$""""$$$$$$"$$$$$,
*                               s$$$$$$$$$$s""$$$$ssssss"$$$$$$$$��
*                              s$$$$$$$$$$'         `"""ss"$"$s"��
*                              s$$$$$$$$$$,              `"""""$  .s$$s
*                              s$$$$$$$$$$$$s,...               `s$$'  `
*                          `ssss$$$$$$$$$$$$$$$$$$$$####s.     .$$"$.   , s-
*                            `""""$$$$$$$$$$$$$$$$$$$$#####$$$$$$"     $.$��
*                                  "$$$$$$$$$$$$$$$$$$$$$####s""     .$$$|
*                                   "$$$$$$$$$$$$$$$$$$$$$$$$##s    .$$" $
*                                    $$""$$$$$$$$$$$$$$$$$$$$$$$$$$$$$"   `
*                                   $$"  "$"$$$$$$$$$$$$$$$$$$$$S""""��
*                              ,   ,"     '  $$$$$$$$$$$$$$$$####s
*                              $.          .s$$$$$$$$$$$$$$$$$####��
*                  ,           "$s.   ..ssS$$$$$$$$$$$$$$$$$$$####��
*                  $           .$$$S$$$$$$$$$$$$$$$$$$$$$$$$#####��
*                  Ss     ..sS$$$$$$$$$$$$$$$$$$$$$$$$$$$######"��
*                   "$$sS$$$$$$$$$$$$$$$$$$$$$$$$$$$########��
*            ,      s$$$$$$$$$$$$$$$$$$$$$$$$#########""��
*            $    s$$$$$$$$$$$$$$$$$$$$$#######""'      s'         ,
*            $$..$$$$$$$$$$$$$$$$$$######"'       ....,$$....    ,$
*             "$$$$$$$$$$$$$$$######"' ,     .sS$$$$$$$$$$$$$$$$s$$
*               $$$$$$$$$$$$#####"     $, .s$$$$$$$$$$$$$$$$$$$$$$$$s.
*    )          $$$$$$$$$$$#####'        `$$$$$$$$$###########$$$$$$$$$$$.
*   ((          $$$$$$$$$$$#####       $$$$$$$$###"       "####$$$$$$$$$$
*   ) \         $$$$$$$$$$$$####.     $$$$$$###"             "###$$$$$$$$$   s��
*  (   )        $$$$$$$$$$$$$####.   $$$$$###"                ####$$$$$$$$s$$��
*  )  ( (       $$"$$$$$$$$$$$#####.$$$$$###'                .###$$$$$$$$$$��
*  (  )  )   _,$"   $$$$$$$$$$$$######.$$##'                .###$$$$$$$$$$
*  ) (  ( \.         "$$$$$$$$$$$$$#######,,,.          ..####$$$$$$$$$$$��
* (   )$ )  )        ,$$$$$$$$$$$$$$$$$$####################$$$$$$$$$$$��
*    ($$  ( \     _sS"  `"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$S$$,
*  )  )$$$s ) )  .      .   `$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$"'  `$$
*   (   $$$Ss/  .$,    .$,,s$$$$$$##S$$$$$$$$$$$$$$$$$$$$$$$$S""        ��
*     \)_$$$$$$$$$$$$$$$$$$$$$$$##"  $$        `$$.        `$$.
*         `"S$$$$$$$$$$$$$$$$$#"      $          `$          `$                   /�����������~~~��/
*/


 

#include "lib.h"
#include "PWM_Control.h"
#include "hrtim.h"
#include "BSP_CAN_FD.h"
#include "referee.h"
#include "Init_Task.h"
#include "usart.h"
#include "oled.h"
static uint32_t CPU_FREQ_Hz;
static uint32_t CYCCNT_RountCount;
static float DWT_Timeline;


/**********************************************************************************
  * @brief                                       ������ѭ��
  * @author                                      Zbb
  * @param                                       ��tim4��1ms����һ��
  * @retval                                      ������
  *********************************************************************************/
//int time_t=0,i_t=0;
 void Protect_Function();
 uint8_t sign=0,tap;
void Main_Task()
{
//    //��ͨ�˲�
//		adc.Power_t=first_order_filter_cali(&power,adc.Power_t);
//	  //�������˲�
//   	adc.Power_t =KalmanFilter(&In_Power,adc.Power_t);
	  
	  //��������֡��
	  i_t++;
		if(HAL_GetTick()-time_t>=1000)//1s
		{
			hrpwm.fps=i_t;//ʵʱ��ʾ������Ƶ��
			i_t=0;
			time_t=HAL_GetTick();
		}//1s��һ��
	  
		//pwm��������
	  if(sign!=1)    
		  PWM_Task();
		
	  //��������
	  Protect_Function();
    
		//can���ݷ���
		Can_Send_Data_Can();
    
	  //���ֿ���
	 // Magazine_Control();
}



/**********************************************************************************
  * @brief                                       ������ѭ��
  * @author                                      Zbb
  * @param                                       ��tim5��10ms����һ��
  * @retval                                      ������
  *********************************************************************************/
int i_1;
void sent_data();
int wer=0,sdf=0,frequency1=0;
void Transmit_Task()//���Ƶ�������oled����
{
	    wer++;
		if(HAL_GetTick()-sdf>=1000)
		{
			frequency1=wer;//ʵʱ��ʾADC����Ƶ��
			wer=0;
			sdf=HAL_GetTick();
		}//1s��һ��
	//oled��Ļ����
		i_1++;
		if(i_1>=13)
		{
			i_1=0;
			oled_task();//100ms,
		}
	
		//��λ��  ��4�ֽ�ǿ��ת��2�ֽ�
		sent_data( (int16_t)(adc.I_Out_Cap),(int16_t)(hrpwm.I_ramp),(int16_t)(adc.Power_t*100),(int16_t)(adc.U_Out_Cap),(int16_t)(Out_Power_Loop.iout/10));
    //                       �����������            ����Ŀ���������         �Ӳ���ϵͳ�������빦��     ���ݵ�ѹ                 ���ʻ�iout
	  //����ϵͳ�������ݷ���,�¹���
//		if(can_data.robot_id!=0)
//			refree_task();
		
}



/**********************************************************************************
  * @brief                                       adc����
  * @author                                      Z
  * @param                                       �˲�������,100us����һ��
  * @retval                                      ������
  *********************************************************************************/
adc_t adc;
 int qwe=0,asd=0,frequency=0;
void ADC_Handle()
{
	//cnt��0����20��Ȼ��ȥ��ƽ��ֵ���ۼӵĲ������������и��õķ���������output compare no output�о�ûɶ�ã�����cubemx�Ķ�����һ��
	  
		//adc�˲����򵥼Ӻ���ƽ��ֵ
		adc.adc_data[0] = Filter_ADC_Function(&lb[0],ADC1_RESULT[0],20);//out_u
		adc.adc_data[1] = Filter_ADC_Function(&lb[1],ADC1_RESULT[1],20);//cap_U
		adc.adc_data[2] = Filter_ADC_Function(&lb[2],ADC1_RESULT[2],20);//24v_I
		adc.adc_data[3] = Filter_ADC_Function(&lb[3],ADC1_RESULT[3],20);//cap_I
		adc.adc_data[4] = Filter_ADC_Function(&lb[4],ADC1_RESULT[4],20);//24v_U
#if Robot ==1
		adc.U_In_Vcc=adc.adc_data[4]*1.07f-1130+200;
		adc.U_Out_Cap=adc.adc_data[1]*0.99f+874+100;//874
		adc.I_Out_Cap=adc.adc_data[3]*(-0.563f)+9418;
	  if(adc.I_Out_Cap<50&&adc.I_Out_Cap>-50)
			adc.I_Out_Cap=0;
		adc.I_In_Vcc=adc.adc_data[2]*(0.561f)-9000;
#endif
#if Robot ==2
		adc.U_In_Vcc=adc.adc_data[4]*1.07f-1130+200;
		adc.U_Out_Cap=adc.adc_data[1]*0.99f+874+100;//874
		adc.I_Out_Cap=adc.adc_data[3]*(-0.563f)+9418;
	  if(adc.I_Out_Cap<50&&adc.I_Out_Cap>-50)
			adc.I_Out_Cap=0;
		adc.I_In_Vcc=adc.adc_data[2]*(0.561f)-9300;
#endif
#if Robot ==3 
		//ADCת��
		adc.U_In_Vcc=adc.adc_data[4]*1.07f-930;
		adc.U_Out_Cap=adc.adc_data[1]*1.04f-242;
		adc.I_Out_Cap=adc.adc_data[3]*(-0.488f)+8325;
	  if(adc.I_Out_Cap<50&&adc.I_Out_Cap>-50)
			adc.I_Out_Cap=0;
		adc.I_In_Vcc=adc.adc_data[2]*(0.498f)-8285;
#endif
#if Robot ==4
		//ADCת��
		adc.U_In_Vcc=adc.adc_data[4]*1.07f-1130;
		adc.U_Out_Cap=adc.adc_data[1]*1.04f-642;
		adc.I_Out_Cap=adc.adc_data[3]*(-0.488f)+8025;
	  if(adc.I_Out_Cap<50&&adc.I_Out_Cap>-50)
			adc.I_Out_Cap=0;
		adc.I_In_Vcc=adc.adc_data[2]*(0.449f)-9514;
#endif
#if Robot ==5
		//ADCת��
		adc.U_In_Vcc=adc.adc_data[4]*1.07f-1330+2150;
		adc.U_Out_Cap=adc.adc_data[1]*0.99f+1974;//874
		adc.I_Out_Cap=adc.adc_data[3]*(-0.504f)+5116;
	  if(adc.I_Out_Cap<50&&adc.I_Out_Cap>-50)
			adc.I_Out_Cap=0;
		adc.I_In_Vcc=adc.adc_data[2]*(0.561f)+90;
#endif
#if Robot ==6
		//ADCת��
		adc.U_In_Vcc=adc.adc_data[4]*1.037f-1130+1000;
		adc.U_Out_Cap=adc.adc_data[1]*0.99f+900;//874
		adc.I_Out_Cap=adc.adc_data[3]*(-0.563f)+9418;
	  if(adc.I_Out_Cap<50&&adc.I_Out_Cap>-50)
			adc.I_Out_Cap=0;
		adc.I_In_Vcc=adc.adc_data[2]*(0.561f)-9336;
#endif
#if Robot ==7//��ͨ�õ�һ��ADC
			adc.U_In_Vcc=adc.adc_data[4]*1.07f-1130-50;
		adc.U_Out_Cap=adc.adc_data[1]*0.99f+874-1400+1200;//874
		adc.I_Out_Cap=adc.adc_data[3]*(-0.083f)-399+1760;
	  if(adc.I_Out_Cap<50&&adc.I_Out_Cap>-50)
			adc.I_Out_Cap=0;
		adc.I_In_Vcc=adc.adc_data[2]*(0.561f)-9131-680;
#endif
#if Robot ==8
//ADCת��
		adc.U_In_Vcc=adc.adc_data[4]*1.07f-1130+100;
		adc.U_Out_Cap=adc.adc_data[1]*0.99f+874-200;//874
		adc.I_Out_Cap=adc.adc_data[3]*(-0.563f)+9419-500;
	  if(adc.I_Out_Cap<50&&adc.I_Out_Cap>-50)
			adc.I_Out_Cap=0;
		adc.I_In_Vcc=adc.adc_data[2]*(0.552f)-8781;//����8
#endif
#if Robot ==9
//ADCת��
			adc.U_In_Vcc=adc.adc_data[4]*1.07f-1130+3200-4500;
		adc.U_Out_Cap=adc.adc_data[1]*0.99f+874+600+700;//874
		adc.I_Out_Cap=adc.adc_data[3]*(-0.563f)+10418-400;
	  if(adc.I_Out_Cap<50&&adc.I_Out_Cap>-50)
			adc.I_Out_Cap=0;
		adc.I_In_Vcc=adc.adc_data[2]*(0.57f)-9100-800;
#endif
#if Robot ==10
			adc.U_In_Vcc=adc.adc_data[4]*1.07f-1130+500;
		adc.U_Out_Cap=adc.adc_data[1]*0.99f+874+100;//874
		adc.I_Out_Cap=adc.adc_data[3]*(-0.563f)+9418-8800;
	  if(adc.I_Out_Cap<50&&adc.I_Out_Cap>-50)
			adc.I_Out_Cap=0;
		adc.I_In_Vcc=adc.adc_data[2]*(0.624f)-11977+600;
#endif
  #if Robot ==11
			adc.U_In_Vcc=adc.adc_data[4]*1.07f-1130+200;
		adc.U_Out_Cap=adc.adc_data[1]*0.99f+874+100;//874
		adc.I_Out_Cap=adc.adc_data[3]*(-0.563f)+9418-400;
	  if(adc.I_Out_Cap<50&&adc.I_Out_Cap>-50)
			adc.I_Out_Cap=0;
		adc.I_In_Vcc=adc.adc_data[2]*(0.62f)-9859;
#endif
  #if Robot ==12
			adc.U_In_Vcc=adc.adc_data[4]*1.07f-1130+200;
		adc.U_Out_Cap=adc.adc_data[1]*0.99f+874+500;//874
		adc.I_Out_Cap=adc.adc_data[3]*(-0.563f)+9418;
	  if(adc.I_Out_Cap<50&&adc.I_Out_Cap>-50)
			adc.I_Out_Cap=0;
		adc.I_In_Vcc=adc.adc_data[2]*(0.62f)-9859;
#endif
    #if Robot ==13
			adc.U_In_Vcc=adc.adc_data[4]*1.07f-1130+200;
		adc.U_Out_Cap=adc.adc_data[1]*0.99f+874+1000;//874
		adc.I_Out_Cap=adc.adc_data[3]*(-0.563f)+9418-500;
	  if(adc.I_Out_Cap<50&&adc.I_Out_Cap>-50)
			adc.I_Out_Cap=0;
		adc.I_In_Vcc=adc.adc_data[2]*(0.535f)-7420;
#endif
    #if Robot ==14
			adc.U_In_Vcc=adc.adc_data[4]*1.07f-1130+100;
		adc.U_Out_Cap=adc.adc_data[1]*0.99f+874-150;//874
		adc.I_Out_Cap=adc.adc_data[3]*(-0.563f)+8418+1100;
	  if(adc.I_Out_Cap<50&&adc.I_Out_Cap>-50)
			adc.I_Out_Cap=0;
		adc.I_In_Vcc=adc.adc_data[2]*(0.57f)-9884-300;
#endif
#if Robot ==15
			adc.U_In_Vcc=adc.adc_data[4]*1.07f-1130;
		adc.U_Out_Cap=adc.adc_data[1]*0.99f+874+100;//874
		adc.I_Out_Cap=adc.adc_data[3]*(-0.5333f)+8267;
	  if(adc.I_Out_Cap<50&&adc.I_Out_Cap>-50)
			adc.I_Out_Cap=0;
		adc.I_In_Vcc=adc.adc_data[2]*(0.84507f)-13446;
#endif
#if Robot ==16
			adc.U_In_Vcc=adc.adc_data[4]*1.07f-1130+200;
		adc.U_Out_Cap=adc.adc_data[1]*0.99f+874+250;//874
		adc.I_Out_Cap=adc.adc_data[3]*(-0.563f)+1481+8100;
	  if(adc.I_Out_Cap<50&&adc.I_Out_Cap>-50)
			adc.I_Out_Cap=0;
		adc.I_In_Vcc=adc.adc_data[2]*(0.5797f)-12604+2700+160+60+660;
#endif
#if Robot ==17
			adc.U_In_Vcc=adc.adc_data[4]*1.07f-1130+200;
		adc.U_Out_Cap=adc.adc_data[1]*0.99f+874+100;//874
		adc.I_Out_Cap=adc.adc_data[3]*(-0.563f)+10000;
	  if(adc.I_Out_Cap<50&&adc.I_Out_Cap>-50)
			adc.I_Out_Cap=0;
		adc.I_In_Vcc=adc.adc_data[2]*(0.500f)-10350;
#endif
    
	  adc.Power_t=adc.U_In_Vcc*adc.I_In_Vcc/1000000;//w
		adc.Power=adc.U_Out_Cap*adc.I_Out_Cap/1000000;
		
		 qwe++;
		if(HAL_GetTick()-asd>=1000)
		{
			frequency=qwe;//ʵʱ��ʾADC����Ƶ��
			qwe=0;
			asd=HAL_GetTick();
		}//1s��һ��
}



/**********************************************************************************
  * @brief                                       ��������
  * @author                                      Z
  * @param                                       ����������ѹ����
  * @retval                                      ������
  *********************************************************************************/
//uint8_t sign=0,tap;
int pt=0,time_i;
void LED_Control();
void Protect_Function()//��Ҫ�ֶ��ǹرտ���PWM������ʱ�䳤��5s֮��
{
	  
	  
	  //����ѹ�ر����,�������ر����,�����ʹر����                                                              ����
		if(adc.U_Out_Cap>30000||adc.U_In_Vcc>30000||adc.I_Out_Cap>35000||adc.I_Out_Cap<-35000||adc.Power>400||adc.Power<-400)
        pt++;
		else
			  pt=0;
		
		if(pt>=20)
			sign=1;//20ms
		
		//�����ʳ��������ƹ����(20ms��ִ��)
		if(sign==1)
		{
			  pt=0;
	      LED_Control();//��˸
			  HAL_HRTIM_WaveformOutputStop(&hhrtim, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //�ر�PWM,TA1,TB1
				HAL_HRTIM_WaveformOutputStop(&hhrtim, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //�ر�PWM,TB1,TB2
			  
			  //���¿���pwm
			  if(HAL_GetTick()-time_i>5000&&pt==0&&tap<5)
				{
					tap++;
					sign=0;
					HAL_HRTIM_WaveformOutputStart(&hhrtim, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //����
					HAL_HRTIM_WaveformOutputStart(&hhrtim, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //����
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
			    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
				}
		}
		else
			time_i=HAL_GetTick();
		
}



/**********************************************************************************
  * @brief                                       led����
  * @author                                      Z
  * @param                                       ����led��˸
  * @retval                                      ������
  *********************************************************************************/
int tim_2;
void LED_Control()
{
	  
		if(HAL_GetTick()-tim_2>100)
		{
			tim_2=HAL_GetTick();
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
		}
		else if(HAL_GetTick()-tim_2>50)
		{
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
		}
}



//��λ���궨��
#define BYTE0(dwTemp) ( * ( ( char*)(&dwTemp)    ) )
#define BYTE1(dwTemp) ( * ( ( char*)(&dwTemp) +1 ) )
#define BYTE2(dwTemp) ( * ( ( char*)(&dwTemp) +2 ) )
#define BYTE3(dwTemp) ( * ( ( char*)(&dwTemp) +3 ) )


//��λ��ͨ��Э��
uint8_t BUFF[100];
void sent_data(int16_t data1, int16_t data2,int16_t data3,int16_t data4,int16_t data5)
{
int i;
uint8_t sumcheck = 0;
uint8_t addcheck = 0;
uint8_t _cnt=0;
BUFF[_cnt++]=0xAA;//֡ͷ
BUFF[_cnt++]=0xFF;//��ַ
BUFF[_cnt++]=0XF1;//������
BUFF[_cnt++]=0x0A;//���ݳ���
BUFF[_cnt++]=BYTE0 ( data1 ) ;//uint8_t 1���ֽڣ�uint16_t 2���ֽ�
BUFF[_cnt++]=BYTE1 ( data1 ) ;
BUFF[_cnt++]=BYTE0 ( data2 ) ;
BUFF[_cnt++]=BYTE1 ( data2 ) ;
BUFF[_cnt++]=BYTE0 ( data3 ) ;
BUFF[_cnt++]=BYTE1 ( data3 ) ;
BUFF[_cnt++]=BYTE0 ( data4 ) ;
BUFF[_cnt++]=BYTE1 ( data4 ) ;
BUFF[_cnt++]=BYTE0 ( data5 ) ;
BUFF[_cnt++]=BYTE1 ( data5 ) ;//��8λ����8λ
// BUFE[_cnt++]=MchassisM_ _RX[2] . speed;
for (i=0;i<BUFF[3]+4;i++)
{
	sumcheck+=BUFF [i] ;
	addcheck+=sumcheck;
}
BUFF[_cnt++]=sumcheck;//�Ӻ�
BUFF[_cnt++]=addcheck;//�Ӻ͵ĺͣ����ڴ�����           ��255ms��ʱ
for(i=0;i<_cnt;i++) HAL_UART_Transmit(&huart3, &BUFF[i],_cnt,0xff) ;
}//���5��2�ֽ����ݴ��ڷ��͵����Դ�ӡ����
void DWT_Init(uint32_t CPU_Freq_mHz)
{
    /* ʹ��DWT���� */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* DWT CYCCNT�Ĵ���������0 */
    DWT->CYCCNT = (uint32_t)0u;

    /* ʹ��Cortex-M DWT CYCCNT�Ĵ��� */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
    CYCCNT_RountCount = 0;
    DWT_Timeline = 0;
}
 float DWT_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz))*1e6;
    *cnt_last = cnt_now;
    return dt;
}





