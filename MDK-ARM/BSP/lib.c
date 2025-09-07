/*                     .                    ________            . 
       /\            / \                   \        \         / \
     /    \        /  .  \       _____      \  ____   \     /  .  \
   / | \    \    /   / \___\    /    /      |  |   \  |   /   / \___\
  |  |   \   |   |  |          /    /       |  |   |  |   |  |
  |  |    \  |   |  |         /    /_____   |  |   |  |   |  |
  |  |____|  |   |  |        /          /   |  |   |  |   |  |
  |  ______  |   |  |       /_____     /    |  |   |  |   |  |
  |  |    |  |   |  |            /    /     |  |   |  |   |  |
  |  |    |  |   |  |   ___      /   /      |  |   /  |   |  |   ___
  |  |    |  |   \   \./  /      /  /       |  |__/   |   \   \./  /
 /   \   /   \     \     /       / /        /        /      \     /
  \./     \./        \./          /        /________/         \./     /数字电源DC-DC/
																																				_____   _____     _____   _____ 
																																			 |  __ \ / ____|   |  __ \ / ____|
																																			 | |  | | |        | |  | | |     
																																			 | |  | | |        | |  | | |     
																																			 | |__| | |____    | |__| | |____ 
																																			 |_____/ \_____|   |_____/ \_____|
                                  
	
  */
	
#include "lib.h"



/**********************************************************************************
  * @brief                                       平滑数据
  * @author                                      文七电源
  * @param                                       计算
  * @retval                                      返回输出
  *********************************************************************************/
lib_t lb[6];
int Filter_ADC_Function(lib_t*lib,int input,int cnt)
{
		if(lib->cnt>cnt-1)
		{
				lib->sum=lib->sum-lib->sum/cnt;
				lib->sum=lib->sum+input;
				return lib->sum/cnt;
		}
		else
		{
				lib->sum=lib->sum+input;
				lib->cnt++;
				return lib->sum/lib->cnt;//简单求平均
		}
		
}



/**********************************************************************************
  * @brief                                       速度环pid
  * @author                                      CSDN
  * @param                                       计算
  * @retval                                      返回无
  *********************************************************************************/
pid  Out_Power_Loop;//功率环
pid  Out_Buff_Loop;//缓冲能量环
pid  Out_U_Loop;
pid  Out_I_Loop;
float velocity_control_speed(pid*p1, float speed,float speed_actual,int big,int max)
{
	    //误差计算
			p1->speed_error_new=speed - speed_actual;
		  
			//积分分离计算
			p1->pout= p1->kp_s*(p1->speed_error_new-p1->speed_error_last);
			p1->iout= p1->ki_s *(p1->speed_error_new);
			p1->dout= p1->kd_s*((p1->speed_error_new)-2.0f*(p1->speed_error_last) + p1->speed_error_llast);
	    
			//积分限幅
			if(p1->iout>max)
					p1->iout=max;
			if(p1->iout<-max)
					p1->iout=-max;
			
			//计算总和
			p1->out_speed_total=p1->out_speed_total+ p1->pout + p1->iout + p1->dout;
			
			//保存上次和上上次误差
			p1->speed_error_llast=p1->speed_error_last;
			p1->speed_error_last=p1->speed_error_new;
			
			//总限幅
			if(p1->out_speed_total>big)
					p1->out_speed_total=big;
			if(p1->out_speed_total<-big)
					p1->out_speed_total=-big;
			
			return 0;
}



/**********************************************************************************
  * @brief                                       位置环pid
  * @author                                      CSDN
  * @param                                       计算
  * @retval                                      返回无
  *********************************************************************************/

float velocity_control_angle(pid*p1,float angle,float angle_actual,int big,float max)
{
	      //误差计算
	      p1->angle_error_new=angle - angle_actual;
	
				//积分分离计算
				p1->pout=  p1->kp_a*p1->angle_error_new;
				p1->iout=  p1->ki_a *(p1->angle_error_new)+p1->iout_angle;
				p1->dout=  p1->kd_a*(p1->angle_error_new-p1->angle_error_last);
		    
					if(p1->iout>max)
					p1->iout=max;
					else if(p1->iout<-max)
					p1->iout=-max;
				
				//计算总和
				p1->out_angle_total = p1->pout + p1->iout + p1->dout;
		    
				//保存上次的i输出和误差
				p1->iout_angle=p1->iout;
				p1->angle_error_llast=p1->angle_error_last;
				p1->angle_error_last=p1->angle_error_new;
		    
				//总限幅
				if(p1->out_angle_total>big)
						p1->out_angle_total=big;
				if(p1->out_angle_total<-big)
						p1->out_angle_total=-big;
			  return 0;
}



/***************************************************************************
  * @brief          一阶低通滤波初始化
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
  **************************************************************************/
first_order_filter_type_t  ta1;
first_order_filter_type_t  tb1;
first_order_filter_type_t  power;
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period)
{
    first_order_filter_type->frame_period = frame_period;//采样周期
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
	first_order_filter_type->cnt=0;//计数
}//一个输入输出



/***************************************************************************
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回滤波后的值
 ************************************************************************* */
float first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
    (1- first_order_filter_type->frame_period) * first_order_filter_type->last_out +  first_order_filter_type->frame_period* first_order_filter_type->input;
	  first_order_filter_type->last_out = first_order_filter_type->out;
	
	  	first_order_filter_type->cnt++;
	  if(	first_order_filter_type->cnt<50)
			return input;
		else
	    return first_order_filter_type->out;
}




/**
  * @brief    			斜坡函数初始化
  * @param    			斜坡函数结构体
  * @param    			间隔时间
  * @param    			斜坡目标值
  * @param    			斜坡源
  */
ramp_function_source_t ramp_function_source;
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

void ramp_calc(ramp_function_source_t *chassis_ramp, float input)
{
    chassis_ramp->input = input;
	if(input != 0)
	{
    chassis_ramp->out += chassis_ramp->input * chassis_ramp->frame_period;
    if (chassis_ramp->out > chassis_ramp->max_value)
        chassis_ramp->out = chassis_ramp->max_value;
    else if (chassis_ramp->out < chassis_ramp->min_value)
        chassis_ramp->out = chassis_ramp->min_value;
	}
	else
	{
		if(chassis_ramp->out > 5)
		  chassis_ramp->out -= 10.0f;
		else if(chassis_ramp->out <-5)
			chassis_ramp->out += 10.0f;
		else
			chassis_ramp->out = 0;
	}
}




/**
  * @name   kalmanCreate
  * @brief  创建一个卡尔曼滤波器
  * @param  p:  滤波器
  *         T_Q:系统噪声协方差
  *         T_R:测量噪声协方差
  *         
  * @retval none
  * @attention R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
  *		       	反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
  */
	
KalmanInfo  Out_I_Cap;
KalmanInfo  In_Power;
KalmanInfo  pitch_auto;
KalmanInfo  pitch_t;
KalmanInfo  pitch_k;
KalmanInfo  yaw_k;

void KalmanCreate(KalmanInfo *p,float T_Q,float T_R)
{
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
  	p->B = 0;
    p->H = 1;
    p->X_mid = p->X_last;
}//R不变，Q越大，越信任实际值，Q越小，越信任预测值

/**
  * @name   KalmanFilter
  * @brief  卡尔曼滤波器
  * @param  p:  滤波器
  *         dat:待滤波数据
  * @retval 滤波后的数据
  * @attention Z(k)是系统输入,即测量值   X(k|k)是卡尔曼滤波后的值,即最终输出
  *            A=1 B=0 H=1 I=1  W(K)  V(k)是高斯白噪声,叠加在测量值上了,可以不用管
  *            以下是卡尔曼的5个核心公式
  *            一阶H'即为它本身,否则为转置矩阵
  */

float KalmanFilter(KalmanInfo* p,float dat)
{
    p->X_mid =p->A*p->X_last;                     //百度对应公式(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    p->P_mid = p->A*p->P_last+p->Q;               //百度对应公式(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
    p->kg = p->P_mid/(p->P_mid+p->R);             //百度对应公式(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    p->X_now = p->X_mid+p->kg*(dat-p->X_mid);     //百度对应公式(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    p->P_now = (1-p->kg)*p->P_mid;                //百度对应公式(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
    p->P_last = p->P_now;                         //状态更新
    p->X_last = p->X_now;
	
	  p->cnt++;
	  if( p->cnt<10)  return dat;
		else            return p->X_now;							  //输出预测结果x(k|k)
}



