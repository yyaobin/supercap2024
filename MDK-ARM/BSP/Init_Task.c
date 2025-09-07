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
 *                                      //.-~~~--\                         /神龙摆尾！！！/
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
  * @brief                                       初始化
  * @author                                      Z
  * @param                                       初始化函数
  * @retval                                      返回无
  *********************************************************************************/
uint32_t ADC1_RESULT[10] ; //adc采值原始数据
 int tim_1;
void Init_Task()
{
	  DWT_Init(240);
	  //滤波初始化                    采样周期
	first_order_filter_init(&ta1, 0.3f);		
	first_order_filter_init(&tb1, 0.3f);
	first_order_filter_init(&power, 0.8f);//没用过
	  
	  //卡尔曼滤波初始化
	  KalmanCreate(&Out_I_Cap,0.5f,1.5f);
	  KalmanCreate(&In_Power,0.5f,2.5f);
	  
	  //oled初始化
	MX_OLED_Init();
	  //LED显示
	  OLED_ShowStr(0 ,0,(unsigned char*)"Writing..."       ,2);
	
		//fdcan初始化
	  FDCAN_Init();
	  
	  //开启定时器中断，1ms一次
	  HAL_TIM_Base_Start_IT(&htim3);
	  
	  //pwm初始化
    //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	  
	  //ADC初始化
	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_RESULT, 5);//启动ADC1采样 DMA数据传送采样输入输出电压电流
	 //1.和大连理工相比优劣之处？
	  //pid初始化
		pid_init();
		
		//参数初始化
	  hrpwm.I_Out=0;
		hrpwm.TA1=360;
	  hrpwm.TB1=2280;//2.为什么要这样赋初始值？
		hrpwm.I_aim=1000;//3.电流环输出值如何转化TA1,TB1？
		hrpwm.I_ramp=500;
	 	hrpwm.Charge_sign=1;
	  hrpwm.Power_Out=1000;
	  can_data.power_buff=60;
	  can_data.power_limit=50;//
	  can_data.super_cap_sign=1;
		can_data.Model=1;
		can_data.charge_power=1;
		can_data.charge_sign1=1;//充电
		//缓启动
		Init_Open_Control(); 
}



/******************************************************************************************************
  * @brief                                       缓启动
  * @author                                      Z
  * @param                                       二级缓启动，最大限度的控制开电时的电容状态
  * @param                                       电压过低保护，避开硬件低压电流太高工作不正常的问题
  * @retval                                      返回无
  *****************************************************************************************************/
void Init_Open_Control()
{
	
	  //等待adc采集到比较准确的电压电流值
	  tim_1=HAL_GetTick();
		while(HAL_GetTick()-tim_1<2000) { }//等2s
	  
    //hrtim初始化,开启pwm输出
		HAL_HRTIM_WaveformOutputStart(&hhrtim, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //开启PWM输出和PWM计时器
		HAL_HRTIM_WaveformOutputStart(&hhrtim, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); //开启PWM输出和PWM计时器
	  HAL_HRTIM_WaveformCounterStart(&hhrtim, HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_MASTER);
		
		
	  //pwm的第一帧输出，保证在任意电压上电输出电流都接近于零
    Init_PWM_Write_Open();                       //一级缓启，有一帧数据
	  
	  //当输入电压小于7V时先升到7V再进行pid闭环
	  Init_Cap_Low_Voltage_Protection();//二级缓启，有2个数据，是否是电压差太大那几次次赋值导致电流太大mos管烧毁
													//让压差小一点
		//开启定时器中断，1ms一次，开启pid闭环任务
	  HAL_TIM_Base_Start_IT(&htim4);
		HAL_TIM_Base_Start_IT(&htim5);
		
		hrpwm.time_init=HAL_GetTick();
	  
}



//pid初始化
void pid_init()
{
	  //功率限制环
		Out_Power_Loop.kp_a=1400.1f;
		Out_Power_Loop.ki_a=50.0f;//370
		Out_Power_Loop.kd_a=0;
	  
	  //缓冲能量环
	  Out_Buff_Loop.kp_a=6000.1f;
		Out_Buff_Loop.ki_a=10.0f;
		Out_Buff_Loop.kd_a=0;
	  
#if	Speed_Loop ==0 //位置式
	  //电压环
	  Out_U_Loop.kp_a=0.07f;
		Out_U_Loop.ki_a=0.0003f;
		Out_U_Loop.kd_a=0;
		
	  //电流环
		Out_I_Loop.kp_a=0.03f;//0.03f
		Out_I_Loop.ki_a=0.005f;
		Out_I_Loop.kd_a=0;
#else              //增量式
	  //电压环
	  Out_U_Loop.kp_s=1.0f;
		Out_U_Loop.ki_s=0.001f;
		Out_U_Loop.kd_s=0;
		
	  //电流环
		Out_I_Loop.kp_s=0.1f;
		Out_I_Loop.ki_s=0.0003;
		Out_I_Loop.kd_s=0;	
#endif
	  

}



//pwm的第一次给定，保证任意电压上电电容充放电速度都接近于零
void Init_PWM_Write_Open()
{
	  //通过输入电压与输出电压的比值进行pwm输出
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
    if(adc.U_Out_Cap*1.0f/adc.U_In_Vcc<=1.0f)//输出小于24v
		{
			hrpwm.I_Out=adc.U_Out_Cap*2400/adc.U_In_Vcc;//-340
		}
		else if(adc.U_Out_Cap*1.0f/adc.U_In_Vcc>1.0f)
		{
			hrpwm.I_Out=2000;
		}
		if(hrpwm.I_Out<0)  hrpwm.I_Out=0;
		#endif
		
		//计算pwm
		if(hrpwm.I_Out<1920)//buck模式
		{
				 hrpwm.TA1=hrpwm.I_Out+2400*0.03f;//在0.8v以下输出都是0
				 hrpwm.TB1=2400*0.95f;
		}
		else if(hrpwm.I_Out>=1920)//boost模式
		{
				 hrpwm.TA1=2400*0.95f;
				 hrpwm.TB1=4200-hrpwm.I_Out;
		}
		
		//pwm更新
		hrpwm_tim( );//上面的赋值TA1TB1基本没啥用
		
		//非阻塞延时200ms
		tim_1=HAL_GetTick();
		while(HAL_GetTick()-tim_1<200) { oled_task();  Protect_Function(); }

		
}



//开电低压保护
void Init_Cap_Low_Voltage_Protection()
{
	  if(adc.U_Out_Cap<6500)
		{
			   //给定pwm
				 hrpwm.I_Out=-130;
         
			   //计算pwm
				 if(hrpwm.I_Out<1920)//buck模式
				 {
						hrpwm.TA1=hrpwm.I_Out+2400*0.15f;
						hrpwm.TB1=2400*0.95f;
				 }
				 else if(hrpwm.I_Out>=1920)//boost模式
				 {
						hrpwm.TA1=2400*0.95f;
						hrpwm.TB1=4200-hrpwm.I_Out;
				 }			
				 //更新pwm
				 hrpwm_tim( );
				 //当电压升到5V左右结束
				 tim_1=HAL_GetTick();//超时退出，15s，防止程序就此陨落
				 while(adc.U_Out_Cap<4500&&HAL_GetTick()-tim_1<15000) { oled_task();  Protect_Function();}
				 
				 //给定pwm
				 hrpwm.I_Out=180;
                
			   //计算pwm
				 if(hrpwm.I_Out<1920)//buck模式
				 {
						hrpwm.TA1=hrpwm.I_Out+2400*0.15f;
						hrpwm.TB1=2400*0.95f;
				 }
				 else if(hrpwm.I_Out>=1920)//boost模式
				 {
						hrpwm.TA1=2400*0.95f;
						hrpwm.TB1=4200-hrpwm.I_Out;
				 }			
				 //更新pwm
				 hrpwm_tim( );
				 //当电压升到7V左右结束
				 tim_1=HAL_GetTick();//超时退出，15s，防止程序就此陨落
				 while(adc.U_Out_Cap<6700&&HAL_GetTick()-tim_1<15000) { oled_task();  Protect_Function();}
				 
				 
		}
	
}





