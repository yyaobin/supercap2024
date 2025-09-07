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
                                           `:.:'        /皮卡丘的十万伏特,皮，卡，丘~~~/
*/

#ifndef __LIB_H
#define __LIB_H



/*---------------------------------宏定义区--------------------------------*/

//不同电容板ADC
#define Robot (9)  //号码与贴纸相对应，目前在老麦上面的是15，英雄上是16

//是否全自动充放电，0否1是
#define Power_Loop (1)

//增量式与位置式，0位置式，1增量式
#define Speed_Loop (0)//增量式的pid需要重调，不能直接用

//限制最大充放电功率/单位mW
#define Max_Power       300000
                        
///限制最大底盘限制功率/单位W
#define Max_Power_Limit 150
                        
//限制最大充放电电流/单位mA
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
    float U_In_Vcc;//裁判系统输入电压
    float U_Out_Cap;//电容输出电压
    float I_In_Vcc;//裁判系统输入电流
    float I_Out_Cap;//电容输出电流
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
    float input;        //输入数据
    float out;          //滤波输出的数据
    float num;       //滤波参数
    float frame_period; //滤波的时间间隔 单位 s
		float last_out;
	int cnt;
} first_order_filter_type_t;

extern first_order_filter_type_t  ta1;
extern first_order_filter_type_t  tb1;
extern first_order_filter_type_t  power;




typedef __packed struct
{
    float input;        //输入数据
    float out;          //输出数据
    float min_value;    //限幅最小值
    float max_value;    //限幅最大值
    float frame_period; //时间间隔
} ramp_function_source_t;

extern ramp_function_source_t ramp_function_source;



typedef struct {
    float X_last; //上一时刻的最优结果  X(k-|k-1)
    float X_mid;  //当前时刻的预测结果  X(k|k-1)
    float X_now;  //当前时刻的最优结果  X(k|k)
    float P_mid;  //当前时刻预测结果的协方差  P(k|k-1)
    float P_now;  //当前时刻最优结果的协方差  P(k|k)
    float P_last; //上一时刻最优结果的协方差  P(k-1|k-1)
    float kg;     //kalman增益
    float A;      //系统参数
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
