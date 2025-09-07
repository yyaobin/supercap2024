#ifndef __referee_H
#define __referee_H

#include "main.h"

typedef __packed struct   //�������ݽ�����Ϣ��0x301
{
 uint16_t data_cmd_id;
 uint16_t sender_ID;
 uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

typedef __packed struct   //�����˼�ͨ��  ����ID��0x0200~0x02FF
{
   uint8_t data[112];
} robot_interactive_data_t;


typedef __packed struct   //�ͻ���ɾ��ͼ��  ����ID��0x0100
{
	uint8_t operate_tpye;
	uint8_t layer;
} ext_client_custom_graphic_delete_t;

typedef __packed struct   //ͼ��
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye:3;
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11; 
	uint32_t radius:10;
	uint32_t end_x:11;
	uint32_t end_y:11;
} graphic_data_struct_t;

typedef __packed struct        //0003���������˴������
{   
//  uint8_t Steering_engine; 
  uint8_t Super_cap;//������
  uint8_t shoot_flag;//Ħ����
  uint8_t Steer_flag;//����
  uint8_t car_mood;//С����
	uint8_t auto_shoot;//����
	uint8_t heat_control;//��������
}game_cmd;

typedef __packed struct   
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye:3;
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11; 
  int32_t  i;
} graphic_data_struct_t_i;        //������

typedef __packed struct   
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye:3;
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11; 
  float  f;                      //������
} graphic_data_struct_t_f;

typedef __packed struct    //����һ��ͼ��  ����ID��0x0101
{
	 graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;

typedef __packed struct    //��������ͼ��  ����ID��0x0102
{
	graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;

typedef __packed struct    //�������ͼ��  ����ID��0x0103
{
	graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;

typedef __packed struct    //�����߸�ͼ��  ����ID��0x0104
{
	graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;

typedef __packed struct     //�����ַ�  ����ID��0x0110
{
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];

} ext_client_custom_character_t;

typedef __packed struct   
{
  uint8_t SOF;
	uint16_t DataLength;
	uint8_t Seq;
	uint8_t CRC8;
}Frameheader_t;


typedef __packed struct
{
	Frameheader_t	Frameheader;
	uint16_t cmd_id;
	ext_student_interactive_header_data_t Client_Custom_ID;  
	graphic_data_struct_t grapic_data_struct;
	uint16_t crc16;
}ext_draw_ui_1;   


typedef __packed struct
{
	Frameheader_t	Frameheader;
	uint16_t cmd_id;
	ext_student_interactive_header_data_t Client_Custom_ID;  
	graphic_data_struct_t grapic_data_struct[5];
	uint16_t crc16;
}ext_draw_ui_5;                       //����5��ͼ��

typedef __packed struct
{
	Frameheader_t	Frameheader;
	uint16_t cmd_id;
	ext_student_interactive_header_data_t Client_Custom_ID;  
	graphic_data_struct_t grapic_data_struct[7];
	uint16_t crc16;
}ext_draw_ui_7;                       //����7��ͼ��

typedef __packed struct
{
	Frameheader_t	Frameheader;
	uint16_t cmd_id;
	ext_student_interactive_header_data_t Client_Custom_ID;  
	graphic_data_struct_t grapic_data_struct;
	char data[30];
	uint16_t crc16;
}ext_draw_p;                       //�ַ�

typedef __packed struct
{
	Frameheader_t	Frameheader;
	uint16_t cmd_id;
	ext_student_interactive_header_data_t Client_Custom_ID;  
	graphic_data_struct_t_i graphic_data_struct_t_ii;
	uint16_t crc16;
}ext_draw_i;                      //������

typedef __packed struct
{
	Frameheader_t	Frameheader;
	uint16_t cmd_id;
	ext_student_interactive_header_data_t Client_Custom_ID;  
	graphic_data_struct_t_f graphic_data_struct_t_ff;
	uint16_t crc16;
}ext_draw_f;                     //������


extern  uint8_t 	Self_ID;//��ǰ�����˵�ID
extern  uint16_t 	SelfClient_ID;//�����߻����˶�Ӧ�Ŀͻ���ID
extern  uint8_t uart8_tx_buff[200];

void refree_task(void);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);


#endif
