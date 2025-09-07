 /*
        quu..__
         $$$b  `---.__
          "$$b        `--.                          ___.---uuudP
           `$$b           `.__.------.__     __.---'      $$$$"              .
             "$b          -'            `-.-'            $$$"              .'|
               ".                                       d$"             _.'  |
                 `.   /                              ..."             .'     |
                   `./                           ..::-'            _.'       |
                    /                         .:::-'            .-'         .'
                   :                          ::''\          _.'            |
                  .' .-.             .-.           `.      .'               |
                  : /'$$|           .@"$\           `.   .'              _.-'
                 .'|$u$$|          |$$,$$|           |  <            _.-'
                 | `:$$:'          :$$$$$:           `.  `.       .-'
                 :                  `"--'             |    `-.     \
                :##.       ==             .###.       `.      `.    `\
                |##:                      :###:        |        >     >
                |#'     `..'`..'          `###'        x:      /     /
                 \                                   xXX|     /    ./
                  \                                xXXX'|    /   ./
                  /`-.                                  `.  /   /
                 :    `-  ...........,                   | /  .'
                 |         ``:::::::'       .            |<    `.
                 |             ```          |           x| \ `.:``.
                 |                         .'    /'   xXX|  `:`M`M':.
                 |    |                    ;    /:' xXXX'|  -'MMMMM:'
                 `.  .'                   :    /:'       |-'MMMM.-'
                  |  |                   .'   /'        .'MMM.-'
                  `'`'                   :  ,'          |MMM<
                    |                     `'            |tbap\
                     \                                  :MM.-'
                      \                 |              .''
                       \.               `.            /
                        /     .:::::::.. :           /
                       |     .:::::::::::`.         /
                       |   .:::------------\       /
                      /   .''               >::'  /
                      `',:                 :    .'
                                           `:.:'        /Ƥ�����ʮ�����,Ƥ��������~~~/
*/

#ifndef __LIB_H
#define __LIB_H



/*---------------------------------�궨����--------------------------------*/

//��ͬ���ݰ�ADC
#define Robot (9)  //��������ֽ���Ӧ��Ŀǰ�������������15��Ӣ������16

//�Ƿ�ȫ�Զ���ŵ磬0��1��
#define Power_Loop (1)

//����ʽ��λ��ʽ��0λ��ʽ��1����ʽ
#define Speed_Loop (0)//����ʽ��pid��Ҫ�ص�������ֱ����

//��������ŵ繦��/��λmW
#define Max_Power       300000
                        
///�������������ƹ���/��λW
#define Max_Power_Limit 150
                        
//��������ŵ����/��λmA
#define Max_I           27000

/*-----------------------------------------------------------------------*/



typedef struct
{
    int sum;
		int output;
		int cnt;
		double k_1;
} lib_t;

extern lib_t lb[6];



typedef struct
{
    float adc_data[5];
    float U_In_Vcc;//����ϵͳ�����ѹ
    float U_Out_Cap;//���������ѹ
    float I_In_Vcc;//����ϵͳ�������
    float I_Out_Cap;//�����������
    float Cap;
	  float Power;
	  float Power_t,power_t_k;
	  float Referee_Power;
} adc_t;

extern adc_t adc;

typedef struct 
{
	float speed_error_new;
	float speed_error_last;
	float speed_error_llast;
	float angle_error_new;
	float angle_error_last;
	float angle_error_llast;
	float pout;
	float iout;
	float iout_angle;
	float dout;
  float out_speed_total;
	float out_angle_total;
	float kp_a;
	float ki_a;
	float kd_a;
	float kp_s;
	float ki_s;
	float kd_s;
	int speed_limit;
	int angle_limit;
	
	
	
}pid;

extern pid Out_U_Loop;
extern pid  Out_I_Loop;
extern pid  Out_Power_Loop;
extern pid  Out_Buff_Loop;



extern int super_sign;



typedef __packed struct
{
    float input;        //��������
    float out;          //�˲����������
    float num;       //�˲�����
    float frame_period; //�˲���ʱ���� ��λ s
		float last_out;
	int cnt;
} first_order_filter_type_t;

extern first_order_filter_type_t  ta1;
extern first_order_filter_type_t  tb1;
extern first_order_filter_type_t  power;




typedef __packed struct
{
    float input;        //��������
    float out;          //�������
    float min_value;    //�޷���Сֵ
    float max_value;    //�޷����ֵ
    float frame_period; //ʱ����
} ramp_function_source_t;

extern ramp_function_source_t ramp_function_source;



typedef struct {
    float X_last; //��һʱ�̵����Ž��  X(k-|k-1)
    float X_mid;  //��ǰʱ�̵�Ԥ����  X(k|k-1)
    float X_now;  //��ǰʱ�̵����Ž��  X(k|k)
    float P_mid;  //��ǰʱ��Ԥ������Э����  P(k|k-1)
    float P_now;  //��ǰʱ�����Ž����Э����  P(k|k)
    float P_last; //��һʱ�����Ž����Э����  P(k-1|k-1)
    float kg;     //kalman����
    float A;      //ϵͳ����
	float B;
    float Q;
    float R;
    float H;
  	int cnt;
}KalmanInfo;

extern KalmanInfo  Out_I_Cap;
extern KalmanInfo  In_Power;
extern KalmanInfo  pitch_auto;
extern KalmanInfo  pitch_t;
extern KalmanInfo  pitch_k;
extern KalmanInfo  yaw_k;



void KalmanCreate(KalmanInfo *p,float T_Q,float T_R);
float KalmanFilter(KalmanInfo* p,float dat);


void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min);
void ramp_calc(ramp_function_source_t *chassis_ramp, float input);



void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period);
float first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);


float velocity_control_angle(pid*p1,float angle,float angle_actual,int big,float max);
float velocity_control_speed(pid*p1,float speed,float speed_actual,int big,int max);
void pid_Init(void);







int Filter_ADC_Function(lib_t*lib,int input,int cnt);







#endif
