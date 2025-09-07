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
  \./     \./        \./          /        /________/         \./     /���ֵ�ԴDC-DC/
																																				_____   _____     _____   _____ 
																																			 |  __ \ / ____|   |  __ \ / ____|
																																			 | |  | | |        | |  | | |     
																																			 | |  | | |        | |  | | |     
																																			 | |__| | |____    | |__| | |____ 
																																			 |_____/ \_____|   |_____/ \_____|
                                  
	
  */
	
#include "lib.h"



/**********************************************************************************
  * @brief                                       ƽ������
  * @author                                      ���ߵ�Դ
  * @param                                       ����
  * @retval                                      �������
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
				return lib->sum/lib->cnt;//����ƽ��
		}
		
}



/**********************************************************************************
  * @brief                                       �ٶȻ�pid
  * @author                                      CSDN
  * @param                                       ����
  * @retval                                      ������
  *********************************************************************************/
pid  Out_Power_Loop;//���ʻ�
pid  Out_Buff_Loop;//����������
pid  Out_U_Loop;
pid  Out_I_Loop;
float velocity_control_speed(pid*p1, float speed,float speed_actual,int big,int max)
{
	    //������
			p1->speed_error_new=speed - speed_actual;
		  
			//���ַ������
			p1->pout= p1->kp_s*(p1->speed_error_new-p1->speed_error_last);
			p1->iout= p1->ki_s *(p1->speed_error_new);
			p1->dout= p1->kd_s*((p1->speed_error_new)-2.0f*(p1->speed_error_last) + p1->speed_error_llast);
	    
			//�����޷�
			if(p1->iout>max)
					p1->iout=max;
			if(p1->iout<-max)
					p1->iout=-max;
			
			//�����ܺ�
			p1->out_speed_total=p1->out_speed_total+ p1->pout + p1->iout + p1->dout;
			
			//�����ϴκ����ϴ����
			p1->speed_error_llast=p1->speed_error_last;
			p1->speed_error_last=p1->speed_error_new;
			
			//���޷�
			if(p1->out_speed_total>big)
					p1->out_speed_total=big;
			if(p1->out_speed_total<-big)
					p1->out_speed_total=-big;
			
			return 0;
}



/**********************************************************************************
  * @brief                                       λ�û�pid
  * @author                                      CSDN
  * @param                                       ����
  * @retval                                      ������
  *********************************************************************************/

float velocity_control_angle(pid*p1,float angle,float angle_actual,int big,float max)
{
	      //������
	      p1->angle_error_new=angle - angle_actual;
	
				//���ַ������
				p1->pout=  p1->kp_a*p1->angle_error_new;
				p1->iout=  p1->ki_a *(p1->angle_error_new)+p1->iout_angle;
				p1->dout=  p1->kd_a*(p1->angle_error_new-p1->angle_error_last);
		    
					if(p1->iout>max)
					p1->iout=max;
					else if(p1->iout<-max)
					p1->iout=-max;
				
				//�����ܺ�
				p1->out_angle_total = p1->pout + p1->iout + p1->dout;
		    
				//�����ϴε�i��������
				p1->iout_angle=p1->iout;
				p1->angle_error_llast=p1->angle_error_last;
				p1->angle_error_last=p1->angle_error_new;
		    
				//���޷�
				if(p1->out_angle_total>big)
						p1->out_angle_total=big;
				if(p1->out_angle_total<-big)
						p1->out_angle_total=-big;
			  return 0;
}



/***************************************************************************
  * @brief          һ�׵�ͨ�˲���ʼ��
  * @author         RM
  * @param[in]      һ�׵�ͨ�˲��ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @param[in]      �˲�����
  * @retval         ���ؿ�
  **************************************************************************/
first_order_filter_type_t  ta1;
first_order_filter_type_t  tb1;
first_order_filter_type_t  power;
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period)
{
    first_order_filter_type->frame_period = frame_period;//��������
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
	first_order_filter_type->cnt=0;//����
}//һ���������



/***************************************************************************
  * @brief          һ�׵�ͨ�˲�����
  * @author         RM
  * @param[in]      һ�׵�ͨ�˲��ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @retval         �����˲����ֵ
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
  * @brief    			б�º�����ʼ��
  * @param    			б�º����ṹ��
  * @param    			���ʱ��
  * @param    			б��Ŀ��ֵ
  * @param    			б��Դ
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
  * @brief  ����һ���������˲���
  * @param  p:  �˲���
  *         T_Q:ϵͳ����Э����
  *         T_R:��������Э����
  *         
  * @retval none
  * @attention R�̶���QԽ�󣬴���Խ���β���ֵ��Q�������ֻ�ò���ֵ
  *		       	��֮��QԽС����Խ����ģ��Ԥ��ֵ��QΪ������ֻ��ģ��Ԥ��
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
}//R���䣬QԽ��Խ����ʵ��ֵ��QԽС��Խ����Ԥ��ֵ

/**
  * @name   KalmanFilter
  * @brief  �������˲���
  * @param  p:  �˲���
  *         dat:���˲�����
  * @retval �˲��������
  * @attention Z(k)��ϵͳ����,������ֵ   X(k|k)�ǿ������˲����ֵ,���������
  *            A=1 B=0 H=1 I=1  W(K)  V(k)�Ǹ�˹������,�����ڲ���ֵ����,���Բ��ù�
  *            �����ǿ�������5�����Ĺ�ʽ
  *            һ��H'��Ϊ������,����Ϊת�þ���
  */

float KalmanFilter(KalmanInfo* p,float dat)
{
    p->X_mid =p->A*p->X_last;                     //�ٶȶ�Ӧ��ʽ(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    p->P_mid = p->A*p->P_last+p->Q;               //�ٶȶ�Ӧ��ʽ(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
    p->kg = p->P_mid/(p->P_mid+p->R);             //�ٶȶ�Ӧ��ʽ(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    p->X_now = p->X_mid+p->kg*(dat-p->X_mid);     //�ٶȶ�Ӧ��ʽ(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    p->P_now = (1-p->kg)*p->P_mid;                //�ٶȶ�Ӧ��ʽ(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
    p->P_last = p->P_now;                         //״̬����
    p->X_last = p->X_now;
	
	  p->cnt++;
	  if( p->cnt<10)  return dat;
		else            return p->X_now;							  //���Ԥ����x(k|k)
}



