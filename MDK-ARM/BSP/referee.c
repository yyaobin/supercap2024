//                             _______________________________________  
//                            /  ___________________________________  \
//     _--""""--_            /  /_/_/_/_/_|_|_|_|_|_|_|_|_|_\_\_\_\_\  \        /人生如棋，棋如人生/
//    /          \          /  /_/_/_/_J__L_L_L_|_|_|_J_J_J__L_\_\_\_\  \       /代码如棋，棋如代码/
//   /\          /\        /  /_/_/_J__L_J__L_L_|_|_|_J_J__L_J__L_\_\_\  \
//   L ""-____-"" J       /  /_/_J__L_J__L_J_J__L_|_J__L_L_J__L_J__L_\_\  \
//   \            /      /  /_/__L_/__L_J__L_J__L_|_J__L_J__L_J__\_J__\_\  \
//    \_        _/      /  /_J__/_J__/__L_J__|__L_|_J__|__L_J__\__L_\__L_\  \
//  _--"""""--_"       /  /  F /  F J  J  |  F J  |  F J  |  F  F J  \ J  \  \
// /           \      /  /--/-J--/--L--|--L-J--J--|--L--L-J--|--J--\--L-\--\  \
///\           /\    /  /__/__L_J__J___L_J__J__|__|__|__L__L_J___L__L_J__\__\  \
//L ""-_____-"" J   /  /  /  /  F  F  J  J  |  |  |  |  |  F  F  J  J  \  \  \  \
//\             /  /  /--/--/--/--J---L--|--|--|--o--|--|--|--J---L--\--\--\--\  \
// \_         _/  /  /__/__J__J___L__J___L__L__L__|__J__J__J___L__J___L__L__\__\  \
//   "--___--"   /  /  /   F  F  J   F  J  J   F  |  J   F  F  J   F  J  J   \  \  \
//              /  /--/---/--J---L--J---L--|--J---|---L--|--J---L--J---L--\---\--\  \
//             /  /__J___/___L__/___L__J___L__J___|___L__J___L__J___\__J___\___L__\  \
//            /  /   F  J   /  J   J   |  J   J   |   F   F  |   F   F  \   F  J   \  \
//           /  /---/---L--J---L---L---L--|---|---|---|---|--J---J---J---L--J---\---\  \
//          /  /___/___/___L__J___J___J___|___|___|___|___|___L___L___L__J___\___\___\  \
//         /  /   /   /   /   F   F   F   F   F   |   J   J   J   J   J   \   \   \   \  \
//        /  /___/___J___J___J___J___J____L___L___|___J___J____L___L___L___L___L___\___\  \
//       /  /   /    F   F   F   |   |   J    F   |   J    F   |   |   J   J   J    \   \  \
//      /  /___J____/___/___J____L___L___|___J____|____L___|___J___J____L___\___\____L___\  \
//     /  /    F   /   J    F   J   J    |   J    |    F   |    F   F   J    F   \   J    \  \
//    /  /____/___J____L___/____L___|____L___|____|____|___J____|___J____\___J____L___\____\  \
//   /  /    /    F   /   J    J    F   J    F    |    J    F   J    F    F   \   J    \    \  \
//  /  /____/____/___J____L____|____L___J____L____|____J____L___J____|____J____L___\____\____\  \
// /                                                                                             \
///_______________________________________________________________________________________________\
//|                                                                                               |
//| zbb                                                                                           |
//|_______________________________________________________________________________________________|   



#include "string.h"
#include "usart.h"
#include "BSP_CAN_FD.h"
#include "stdio.h"
#include "PWM_Control.h"
#include "referee.h"

uint8_t uart8_tx_buff[200];
#define BLUE  0
#define RED   1
uint8_t 	Self_ID;//当前机器人的ID
uint16_t 	SelfClient_ID;//发送者机器人对应的客户端ID



//CRC8校验
const unsigned char CRC8_INIT = 0xff;

const unsigned char CRC8_TAB[256] =
{
		0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
		0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
		0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
		0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
		0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
		0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
		0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
		0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
		0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
		0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
		0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
		0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
		0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
		0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
		0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
		0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int
dwLength,unsigned char ucCRC8)
{
		unsigned char ucIndex;
		while (dwLength--)
		{
				ucIndex = ucCRC8^(*pchMessage++);
				ucCRC8 = CRC8_TAB[ucIndex];
		}
		return(ucCRC8);
}

unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
		unsigned char ucExpected = 0;
		if ((pchMessage == 0) || (dwLength <= 2)) return 0;
		ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT);
		return ( ucExpected == pchMessage[dwLength-1] );
}
/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
		unsigned char ucCRC = 0;
		if ((pchMessage == 0) || (dwLength <= 2)) return;
		ucCRC = Get_CRC8_Check_Sum ( (unsigned char *)pchMessage, dwLength-1, CRC8_INIT);
		pchMessage[dwLength-1] = ucCRC;
}



//CRC16校验
uint16_t CRC_INIT = 0xffff;

const uint16_t wCRC_Table[256] =
{
		0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
		0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
		0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
		0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
		0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
		0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
		0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
		0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
		0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
		0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
		0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
		0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
		0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
		0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
		0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
		0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
		0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
		0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
		0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
		0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
		0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
		0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
		0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
		0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
		0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
		0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
		0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
		0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
		0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
		0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
		0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
		0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
		uint8_t chData;
		if (pchMessage == NULL)
		{
				return 0xFFFF;
		}
		while(dwLength--)
		{
				chData = *pchMessage++;
				(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) &
				0x00ff];
		}
		return wCRC;
}

uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
		uint16_t wExpected = 0;
		if ((pchMessage == NULL) || (dwLength <= 2))
		{
				return 0;
		}
		wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT);
		return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) ==
		pchMessage[dwLength - 1]);
}
/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength)
{
		uint16_t wCRC = 0;
		if ((pchMessage == NULL) || (dwLength <= 2))
		{
				return;
		}
		wCRC = Get_CRC16_Check_Sum ( (uint8_t *)pchMessage, dwLength-2, CRC_INIT );
		pchMessage[dwLength-2] = (uint8_t)(wCRC & 0x00ff);
		pchMessage[dwLength-1] = (uint8_t)((wCRC >> 8)& 0x00ff);
}	




void draw_UI_char_1(uint16_t cmd_id, 						//命令字符
										uint16_t data_id,						//数据内容ID
										char px1[30],								//数据内容	
										uint32_t operate_tpye,			//图形操作（0不操作1增加2修改3删除）
										uint32_t start_x,						//起点坐标X
										uint32_t start_y						//起点坐标Y
										)
{
	/*读取机器人颜色和ID*/
	Self_ID=can_data.robot_id;
	if(Self_ID> 100)
		SelfClient_ID = 0x0164 + (Self_ID-100);
	else
		SelfClient_ID = 0x0100 + Self_ID;
	/*定义*/
	static ext_draw_p draw_p;
	static uint8_t seq=0;
	/*ID设置*/
	draw_p.cmd_id = cmd_id;  
	draw_p.Client_Custom_ID.data_cmd_id= data_id; 
	draw_p.Client_Custom_ID.sender_ID = Self_ID;
	draw_p.Client_Custom_ID.receiver_ID =SelfClient_ID;	
	/*发送的数据*/
  draw_p.grapic_data_struct.graphic_name[0]=1;				//图形名字
	draw_p.grapic_data_struct.operate_tpye=operate_tpye;	//图形操作
	draw_p.grapic_data_struct.graphic_tpye=7;	//图形类型
	draw_p.grapic_data_struct.layer=1;								//图层数
	draw_p.grapic_data_struct.color=1;								//图形颜色0红蓝主色1黄色2绿色3橙色4紫红色5粉色6青色7黑色8白色
	draw_p.grapic_data_struct.start_angle=20;		//字符大小
	draw_p.grapic_data_struct.end_angle=35;				//字符长度
	draw_p.grapic_data_struct.width=2;								//线宽
	draw_p.grapic_data_struct.start_x=start_x;						//起点坐标X
	draw_p.grapic_data_struct.start_y=start_y;						//起点坐标Y
	draw_p.grapic_data_struct.radius=0;										//无
	draw_p.grapic_data_struct.end_x=0;										//无
	draw_p.grapic_data_struct.end_y=0;										//无
	/*复制数组PX1至draw_p.data*/
  strcpy(draw_p.data,px1);
	/*头配置*/
	draw_p.Frameheader.SOF = 0xA5; 					//0xA5
  draw_p.Frameheader.DataLength=6+45;			//头结构长度+数据段长度
	draw_p.Frameheader.Seq=seq;							//包序号
	/*复制头数据至发送缓冲*/
	memcpy(uart8_tx_buff, &draw_p.Frameheader, sizeof(Frameheader_t));
	/*CRC8校验头*/
	Append_CRC8_Check_Sum(uart8_tx_buff,sizeof(Frameheader_t)); 
	/*复制数据段至发送缓冲*/
  memcpy(uart8_tx_buff + 5, 
				 (uint8_t*)&draw_p.cmd_id, 
				 (sizeof(draw_p.cmd_id)+ sizeof(draw_p.Client_Custom_ID)+ sizeof(draw_p.grapic_data_struct)+sizeof(draw_p.data)));
	/*CRC16校验整包数据*/
	Append_CRC16_Check_Sum(uart8_tx_buff, draw_p.Frameheader.DataLength+9);			
}
/*绘制字符——2*/
void draw_UI_char_2(uint16_t cmd_id, 						//命令字符
										uint16_t data_id,						//数据内容ID
										char px1[30],								//数据内容	
										uint32_t operate_tpye,			//图形操作（0不操作1增加2修改3删除）
										uint32_t start_x,						//起点坐标X
										uint32_t start_y						//起点坐标Y
										)
{
	/*读取机器人颜色和ID*/
	Self_ID=can_data.robot_id;
	if(Self_ID> 100)
		SelfClient_ID = 0x0164 + (Self_ID-100);
	else
		SelfClient_ID = 0x0100 + Self_ID;
	/*定义*/
	static ext_draw_p draw_p;
	static uint8_t seq=0;
	/*ID设置*/
	draw_p.cmd_id = cmd_id;  
	draw_p.Client_Custom_ID.data_cmd_id= data_id; 
	draw_p.Client_Custom_ID.sender_ID = Self_ID;
	draw_p.Client_Custom_ID.receiver_ID = SelfClient_ID;	
	/*发送的数据*/
  draw_p.grapic_data_struct.graphic_name[0]=2;				//图形名字
	draw_p.grapic_data_struct.operate_tpye=operate_tpye;	//图形操作
	draw_p.grapic_data_struct.graphic_tpye=7;	//图形类型
	draw_p.grapic_data_struct.layer=1;								//图层数
	draw_p.grapic_data_struct.color=0;								//图形颜色0红蓝主色1黄色2绿色3橙色4紫红色5粉色6青色7黑色8白色
	draw_p.grapic_data_struct.start_angle=20;		//字符大小
	draw_p.grapic_data_struct.end_angle=30;				//字符长度
	draw_p.grapic_data_struct.width=2;								//线宽
	draw_p.grapic_data_struct.start_x=start_x;						//起点坐标X
	draw_p.grapic_data_struct.start_y=start_y;						//起点坐标Y
	draw_p.grapic_data_struct.radius=0;										//无
	draw_p.grapic_data_struct.end_x=0;										//无
	draw_p.grapic_data_struct.end_y=0;										//无
	/*复制数组PX1至draw_p.data*/
  strcpy(draw_p.data,px1);
	/*头配置*/
	draw_p.Frameheader.SOF = 0xA5; 					//0xA5
  draw_p.Frameheader.DataLength=6+45;			//头结构长度+数据段长度
	draw_p.Frameheader.Seq=seq;							//包序号
	/*复制头数据至发送缓冲*/
	memcpy(uart8_tx_buff, &draw_p.Frameheader, sizeof(Frameheader_t));
	/*CRC8校验头*/
	Append_CRC8_Check_Sum(uart8_tx_buff,sizeof(Frameheader_t)); 
	/*复制数据段至发送缓冲*/
  memcpy(uart8_tx_buff + 5, 
				 (uint8_t*)&draw_p.cmd_id, 
				 (sizeof(draw_p.cmd_id)+ sizeof(draw_p.Client_Custom_ID)+ sizeof(draw_p.grapic_data_struct)+sizeof(draw_p.data)));
	/*CRC16校验整包数据*/
	Append_CRC16_Check_Sum(uart8_tx_buff, draw_p.Frameheader.DataLength+9);			
}
/*绘制字符——3*/
void draw_UI_char_3(uint16_t cmd_id, 						//命令字符
										uint16_t data_id,						//数据内容ID
										char px1[30],								//数据内容	
										uint32_t operate_tpye,			//图形操作（0不操作1增加2修改3删除）
										uint32_t start_x,						//起点坐标X
										uint32_t start_y						//起点坐标Y
										)
{
	/*读取机器人颜色和ID*/
	Self_ID=can_data.robot_id;
	if(Self_ID> 100)
		SelfClient_ID = 0x0164 + (Self_ID-100);
	else
		SelfClient_ID = 0x0100 + Self_ID;
	/*定义*/
	static ext_draw_p draw_p;
	static uint8_t seq=0;
	/*ID设置*/
	draw_p.cmd_id = cmd_id;  
	draw_p.Client_Custom_ID.data_cmd_id= data_id; 
	draw_p.Client_Custom_ID.sender_ID = Self_ID;
	draw_p.Client_Custom_ID.receiver_ID = SelfClient_ID;	
	/*发送的数据*/
  draw_p.grapic_data_struct.graphic_name[0]=3;				//图形名字
	draw_p.grapic_data_struct.operate_tpye=operate_tpye;	//图形操作
	draw_p.grapic_data_struct.graphic_tpye=7;	//图形类型
	draw_p.grapic_data_struct.layer=1;								//图层数
	draw_p.grapic_data_struct.color=0;								//图形颜色0红蓝主色1黄色2绿色3橙色4紫红色5粉色6青色7黑色8白色
	draw_p.grapic_data_struct.start_angle=20;		//字符大小
	draw_p.grapic_data_struct.end_angle=30;				//字符长度
	draw_p.grapic_data_struct.width=2;								//线宽
	draw_p.grapic_data_struct.start_x=start_x;						//起点坐标X
	draw_p.grapic_data_struct.start_y=start_y;						//起点坐标Y
	draw_p.grapic_data_struct.radius=0;										//无
	draw_p.grapic_data_struct.end_x=0;										//无
	draw_p.grapic_data_struct.end_y=0;										//无
	/*复制数组PX1至draw_p.data*/
  strcpy(draw_p.data,px1);
	/*头配置*/
	draw_p.Frameheader.SOF = 0xA5; 					//0xA5
  draw_p.Frameheader.DataLength=6+45;			//头结构长度+数据段长度
	draw_p.Frameheader.Seq=seq;							//包序号
	/*复制头数据至发送缓冲*/
	memcpy(uart8_tx_buff, &draw_p.Frameheader, sizeof(Frameheader_t));
	/*CRC8校验头*/
	Append_CRC8_Check_Sum(uart8_tx_buff,sizeof(Frameheader_t)); 
	/*复制数据段至发送缓冲*/
  memcpy(uart8_tx_buff + 5, 
				 (uint8_t*)&draw_p.cmd_id, 
				 (sizeof(draw_p.cmd_id)+ sizeof(draw_p.Client_Custom_ID)+ sizeof(draw_p.grapic_data_struct)+sizeof(draw_p.data)));
	/*CRC16校验整包数据*/
	Append_CRC16_Check_Sum(uart8_tx_buff, draw_p.Frameheader.DataLength+9);			
}



uint8_t i_ui=0;
void draw_UI_line_1	 
	(uint16_t cmd_id,	uint16_t data_id,uint32_t operate_tpye)
 /*命令字符，				数据内容ID，		 图形操作*/
{
	/*读取机器人颜色和ID*/
	Self_ID=can_data.robot_id;
	if(Self_ID> 100)
		SelfClient_ID = 0x0164 + (Self_ID-100);
	else
		SelfClient_ID = 0x0100 + Self_ID;
	/*定义*/
	static ext_draw_ui_7 draw_l;
	static uint8_t seq=0;
	/*ID设置*/
	draw_l.cmd_id = cmd_id;  
	draw_l.Client_Custom_ID.data_cmd_id= data_id; 
	draw_l.Client_Custom_ID.sender_ID = Self_ID;
	draw_l.Client_Custom_ID.receiver_ID =SelfClient_ID;	
	/*数据*/
	for(int i = 0 ;i < 7 ;i++)
	{
		draw_l.grapic_data_struct[i].graphic_name[0]=i+11;						//图形名字
		draw_l.grapic_data_struct[i].operate_tpye=operate_tpye;		//图形操作
		draw_l.grapic_data_struct[i].graphic_tpye=0;							//图形类型
		draw_l.grapic_data_struct[i].layer=1;											//图层数
		draw_l.grapic_data_struct[i].start_angle=0;								//空
		draw_l.grapic_data_struct[i].end_angle=0;									//空
		draw_l.grapic_data_struct[i].radius=0;										//空
	}
	/*0红蓝主色1黄色2绿色3橙色4紫红色5粉色6青色7黑色8白色*/
	
	//刷新内圆
	draw_l.grapic_data_struct[0].color=5;
	draw_l.grapic_data_struct[0].graphic_tpye=2;							//图形类型
	draw_l.grapic_data_struct[0].width=4;											//线条宽度
	draw_l.grapic_data_struct[0].start_x=1700;							//起点坐标X
	draw_l.grapic_data_struct[0].start_y=500;							//起点坐标Y
	draw_l.grapic_data_struct[0].end_x=0;								//终点坐标X
	draw_l.grapic_data_struct[0].end_y=0;								//终点坐标Y
	draw_l.grapic_data_struct[0].radius=30;										//空

  //电容框
	draw_l.grapic_data_struct[1].graphic_tpye=1;							//图形类型
	draw_l.grapic_data_struct[1].width=2;											//线条宽度
	draw_l.grapic_data_struct[1].color=8;
	draw_l.grapic_data_struct[1].start_x=676;							//起点坐标X
	draw_l.grapic_data_struct[1].start_y=68;								//起点坐标Y
	draw_l.grapic_data_struct[1].end_x=1244;								//终点坐标X
	draw_l.grapic_data_struct[1].end_y=100;								//终点坐标Y
	//
	//底盘位置框图
	draw_l.grapic_data_struct[2].graphic_tpye=0;							//图形类型
	draw_l.grapic_data_struct[2].width=3;											//线条宽度
	draw_l.grapic_data_struct[2].color=6;
	draw_l.grapic_data_struct[2].start_x=640-100;							//起点坐标X
	draw_l.grapic_data_struct[2].start_y=0;								//起点坐标Y
	draw_l.grapic_data_struct[2].end_x=740-50;								//终点坐标X
	draw_l.grapic_data_struct[2].end_y=272;								//终点坐标Y
	//
	draw_l.grapic_data_struct[3].graphic_tpye=0;							//图形类型
	draw_l.grapic_data_struct[3].width=3;											//线条宽度
	draw_l.grapic_data_struct[3].color=6;
	draw_l.grapic_data_struct[3].start_x=1280+100;							//起点坐标X
	draw_l.grapic_data_struct[3].start_y=0;							//起点坐标Y
	draw_l.grapic_data_struct[3].end_x=1180+50;								//终点坐标X
	draw_l.grapic_data_struct[3].end_y=272;								//终点坐标Y
	//
	draw_l.grapic_data_struct[4].graphic_tpye=0;							//图形类型
	draw_l.grapic_data_struct[4].width=3;											//线条宽度
	draw_l.grapic_data_struct[4].color=6;
	draw_l.grapic_data_struct[4].start_x=1180+50;							//起点坐标X
	draw_l.grapic_data_struct[4].start_y=272;							//起点坐标Y
	draw_l.grapic_data_struct[4].end_x=1080;								//终点坐标X
	draw_l.grapic_data_struct[4].end_y=272;								//终点坐标Y
	
	draw_l.grapic_data_struct[5].graphic_tpye=0;							//图形类型
	draw_l.grapic_data_struct[5].width=3;											//线条宽度
	draw_l.grapic_data_struct[5].color=6;
	draw_l.grapic_data_struct[5].start_x=740-50;							//起点坐标X
	draw_l.grapic_data_struct[5].start_y=272;							//起点坐标Y
	draw_l.grapic_data_struct[5].end_x=840;								//终点坐标X
	draw_l.grapic_data_struct[5].end_y=272;								//终点坐标Y
	
	//加速图案框图
	draw_l.grapic_data_struct[6].graphic_tpye=1;							//图形类型
	draw_l.grapic_data_struct[6].width=1;											//线条宽度
	draw_l.grapic_data_struct[6].color=6;
	draw_l.grapic_data_struct[6].start_x=200-10-140;							//起点坐标X
	draw_l.grapic_data_struct[6].start_y=580+90;							//起点坐标Y
	draw_l.grapic_data_struct[6].end_x=460+10-140-80;								//终点坐标X
	draw_l.grapic_data_struct[6].end_y=660+90;								//终点坐标Y
	
	
	/*头配置*/
	draw_l.Frameheader.SOF = 0xA5; 					//0xA5
  draw_l.Frameheader.DataLength=6+105;		  //头结构长度+数据段长度
	draw_l.Frameheader.Seq=seq;							//包序号
	/*复制头数据至发送缓冲*/
	memcpy(uart8_tx_buff, &draw_l.Frameheader, sizeof(Frameheader_t));
	/*CRC8校验头*/
	Append_CRC8_Check_Sum(uart8_tx_buff,sizeof(Frameheader_t)); 
	/*复制数据段至发送缓冲*/
  memcpy(uart8_tx_buff + 5, 
				 (uint8_t*)&draw_l.cmd_id, 
				 (sizeof(draw_l.cmd_id)+ sizeof(draw_l.Client_Custom_ID)+ sizeof(draw_l.grapic_data_struct)));
	/*CRC16校验整包数据*/
	Append_CRC16_Check_Sum(uart8_tx_buff, draw_l.Frameheader.DataLength+9);			
}



void draw_UI_line_2	 
	(uint16_t cmd_id,	uint16_t data_id,uint32_t operate_tpye)
 /*命令字符，				数据内容ID，		 图形操作*/
{
	/*读取机器人颜色和ID*/
	Self_ID=can_data.robot_id;
	if(Self_ID> 100)
		SelfClient_ID = 0x0164 + (Self_ID-100);
	else
		SelfClient_ID = 0x0100 + Self_ID;
	/*定义*/
	static ext_draw_ui_5 draw_l;
	static uint8_t seq=0;
	/*ID设置*/
	draw_l.cmd_id = cmd_id;  
	draw_l.Client_Custom_ID.data_cmd_id= data_id; 
	draw_l.Client_Custom_ID.sender_ID = Self_ID;
	draw_l.Client_Custom_ID.receiver_ID =SelfClient_ID;	
	/*数据*/
	for(int i = 0 ;i < 5 ;i++)
	{
		draw_l.grapic_data_struct[i].graphic_name[0]=i+4;						//图形名字
		draw_l.grapic_data_struct[i].operate_tpye=operate_tpye;		//图形操作
		draw_l.grapic_data_struct[i].graphic_tpye=0;							//图形类型
		draw_l.grapic_data_struct[i].layer=1;											//图层数
		draw_l.grapic_data_struct[i].start_angle=0;								//空
		draw_l.grapic_data_struct[i].end_angle=0;									//空
		draw_l.grapic_data_struct[i].radius=0;										//空
		draw_l.grapic_data_struct[i].width=2;											//线条宽度
	}
	/*0红蓝主色1黄色2绿色3橙色4紫红色5粉色6青色7黑色8白色*/	
	/*竖线*/
	draw_l.grapic_data_struct[0].color=0;
	draw_l.grapic_data_struct[0].start_x=960;							//起点坐标X
	draw_l.grapic_data_struct[0].start_y=250;								//起点坐标Y
	draw_l.grapic_data_struct[0].end_x=960;								//终点坐标X
	draw_l.grapic_data_struct[0].end_y=540;								//终点坐标Y
	/*横1*/
	#define Line_1 500
	draw_l.grapic_data_struct[1].color = 1;
	draw_l.grapic_data_struct[1].start_x=900;							//起点坐标X
	draw_l.grapic_data_struct[1].start_y=Line_1;							//起点坐标Y
	draw_l.grapic_data_struct[1].end_x=1020;							//终点坐标X
	draw_l.grapic_data_struct[1].end_y=Line_1;								//终点坐标Y
	/*横2*/
	#define Line_2 450
	draw_l.grapic_data_struct[2].color = 2;
	draw_l.grapic_data_struct[2].start_x=900;							//起点坐标X
	draw_l.grapic_data_struct[2].start_y=Line_2;							//起点坐标Y
	draw_l.grapic_data_struct[2].end_x=1020;							//终点坐标X
	draw_l.grapic_data_struct[2].end_y=Line_2;								//终点坐标Y
	/*横3*/
	#define Line_3 400
	draw_l.grapic_data_struct[3].color = 3;
	draw_l.grapic_data_struct[3].start_x=900;							//起点坐标X
	draw_l.grapic_data_struct[3].start_y=Line_3;							//起点坐标Y
	draw_l.grapic_data_struct[3].end_x=1020;							//终点坐标X
	draw_l.grapic_data_struct[3].end_y=Line_3;								//终点坐标Y
	/*横4*/
	#define Line_4 300
	draw_l.grapic_data_struct[4].color = 4;
	draw_l.grapic_data_struct[4].start_x=900;							//起点坐标X
	draw_l.grapic_data_struct[4].start_y=Line_4;							//起点坐标Y
	draw_l.grapic_data_struct[4].end_x=1020;							//终点坐标X
	draw_l.grapic_data_struct[4].end_y=Line_4;								//终点坐标Y
	/*横5*/


	/*头配置*/
	draw_l.Frameheader.SOF = 0xA5; 					//0xA5
  draw_l.Frameheader.DataLength=6+75;		//头结构长度+数据段长度
	draw_l.Frameheader.Seq=seq;							//包序号
	/*复制头数据至发送缓冲*/
	memcpy(uart8_tx_buff, &draw_l.Frameheader, sizeof(Frameheader_t));
	/*CRC8校验头*/
	Append_CRC8_Check_Sum(uart8_tx_buff,sizeof(Frameheader_t)); 
	/*复制数据段至发送缓冲*/
  memcpy(uart8_tx_buff + 5, 
				 (uint8_t*)&draw_l.cmd_id, 
				 (sizeof(draw_l.cmd_id)+ sizeof(draw_l.Client_Custom_ID)+ sizeof(draw_l.grapic_data_struct)));
	/*CRC16校验整包数据*/
	Append_CRC16_Check_Sum(uart8_tx_buff, draw_l.Frameheader.DataLength+9);			
}



void draw_UI_line_3	 
	(uint16_t cmd_id,	uint16_t data_id,uint32_t operate_tpye)
 /*命令字符，				数据内容ID，		 图形操作*/
{
	/*读取机器人颜色和ID*/
	Self_ID=can_data.robot_id;
	if(Self_ID> 100)
		SelfClient_ID = 0x0164 + (Self_ID-100);
	else
		SelfClient_ID = 0x0100 + Self_ID;
	/*定义*/
	static ext_draw_ui_7 draw_l;
	static uint8_t seq=0;
	/*ID设置*/
	draw_l.cmd_id = cmd_id;  
	draw_l.Client_Custom_ID.data_cmd_id= data_id; 
	draw_l.Client_Custom_ID.sender_ID = Self_ID;
	draw_l.Client_Custom_ID.receiver_ID =SelfClient_ID;	
	/*数据*/
	for(int i = 0 ;i < 7 ;i++)
	{
		draw_l.grapic_data_struct[i].graphic_name[0]=i+18;						//图形名字
		draw_l.grapic_data_struct[i].operate_tpye=operate_tpye;		//图形操作
		draw_l.grapic_data_struct[i].graphic_tpye=0;							//图形类型
		draw_l.grapic_data_struct[i].layer=0;											//图层数
		draw_l.grapic_data_struct[i].start_angle=0;								//空
		draw_l.grapic_data_struct[i].end_angle=0;									//空
		draw_l.grapic_data_struct[i].radius=0;										//空

	}
	
	if(can_data.super_cap_sign==0)
	{
		draw_l.grapic_data_struct[0].color=8;
		draw_l.grapic_data_struct[1].color=8;
		draw_l.grapic_data_struct[2].color=8;
		draw_l.grapic_data_struct[3].color=8;
		draw_l.grapic_data_struct[4].color=8;
		draw_l.grapic_data_struct[5].color=8;
		draw_l.grapic_data_struct[6].color=8;
	
	
	}
	else if(can_data.super_cap_sign==1)
	{
		draw_l.grapic_data_struct[0].color=3;
		draw_l.grapic_data_struct[1].color=3;
		draw_l.grapic_data_struct[2].color=3;
		draw_l.grapic_data_struct[3].color=3;
		draw_l.grapic_data_struct[4].color=8;
		draw_l.grapic_data_struct[5].color=8;
		draw_l.grapic_data_struct[6].color=8;		
	}
	else
	{
		draw_l.grapic_data_struct[0].color=2;
		draw_l.grapic_data_struct[1].color=2;
		draw_l.grapic_data_struct[2].color=2;
		draw_l.grapic_data_struct[3].color=2;
		draw_l.grapic_data_struct[4].color=2;
		draw_l.grapic_data_struct[5].color=2;
		draw_l.grapic_data_struct[6].color=2;		
	}
	
	
	/*0红蓝主色1黄色2绿色3橙色4紫红色5粉色6青色7黑色8白色*/
  
	//加速框图
	draw_l.grapic_data_struct[0].graphic_tpye=2;							//图形类型
	draw_l.grapic_data_struct[0].width=5;											//线条宽度
	draw_l.grapic_data_struct[0].start_x=430-140-80;							//起点坐标X
	draw_l.grapic_data_struct[0].start_y=620+90;							//起点坐标Y
	draw_l.grapic_data_struct[0].end_x=0;								//终点坐标X
	draw_l.grapic_data_struct[0].end_y=0;								//终点坐标Y
	draw_l.grapic_data_struct[0].radius=22;										//空

	draw_l.grapic_data_struct[1].graphic_tpye=0;							//图形类型
	draw_l.grapic_data_struct[1].width=4;											//线条宽度
	draw_l.grapic_data_struct[1].start_x=400-40-140-50;							//起点坐标X
	draw_l.grapic_data_struct[1].start_y=620+90;							//起点坐标Y
	draw_l.grapic_data_struct[1].end_x=360-40-140-50;							//终点坐标X
	draw_l.grapic_data_struct[1].end_y=620+90;								//终点坐标Y
	draw_l.grapic_data_struct[1].radius=0;										//空
	
	draw_l.grapic_data_struct[2].graphic_tpye=0;							//图形类型
	draw_l.grapic_data_struct[2].width=4;											//线条宽度
	draw_l.grapic_data_struct[2].start_x=395-40-140-50;							//起点坐标X
	draw_l.grapic_data_struct[2].start_y=640+90;							//起点坐标Y
	draw_l.grapic_data_struct[2].end_x=355-40-140-50;								//终点坐标X
	draw_l.grapic_data_struct[2].end_y=640+90;								//终点坐标Y
	//
	draw_l.grapic_data_struct[3].graphic_tpye=0;							//图形类型
	draw_l.grapic_data_struct[3].width=4;											//线条宽度
	draw_l.grapic_data_struct[3].start_x=395-40-140-50;							//起点坐标X
	draw_l.grapic_data_struct[3].start_y=600+90;							//起点坐标Y
	draw_l.grapic_data_struct[3].end_x=355-40-140-50;								//终点坐标X
	draw_l.grapic_data_struct[3].end_y=600+90;								//终点坐标Y
	//
	
	draw_l.grapic_data_struct[4].graphic_tpye=0;							//图形类型
	draw_l.grapic_data_struct[4].width=4;											//线条宽度
	draw_l.grapic_data_struct[4].start_x=350-100-140;							//起点坐标X
	draw_l.grapic_data_struct[4].start_y=620+90;							//起点坐标Y
	draw_l.grapic_data_struct[4].end_x=310-100-140;								//终点坐标X
	draw_l.grapic_data_struct[4].end_y=620+90;								//终点坐标Y
	//
	draw_l.grapic_data_struct[5].graphic_tpye=0;							//图形类型
	draw_l.grapic_data_struct[5].width=4;											//线条宽度
	draw_l.grapic_data_struct[5].start_x=345-100-140;							//起点坐标X
	draw_l.grapic_data_struct[5].start_y=640+90;							//起点坐标Y
	draw_l.grapic_data_struct[5].end_x=305-100-140;								//终点坐标X
	draw_l.grapic_data_struct[5].end_y=640+90;								//终点坐标Y
	//
	draw_l.grapic_data_struct[6].graphic_tpye=0;							//图形类型
	draw_l.grapic_data_struct[6].width=4;											//线条宽度
	draw_l.grapic_data_struct[6].start_x=345-100-140;							//起点坐标X
	draw_l.grapic_data_struct[6].start_y=600+90;							//起点坐标Y
	draw_l.grapic_data_struct[6].end_x=305-100-140;								//终点坐标X
	draw_l.grapic_data_struct[6].end_y=600+90;								//终点坐标Y
  
	//
	
	/*头配置*/
	draw_l.Frameheader.SOF = 0xA5; 					//0xA5
  draw_l.Frameheader.DataLength=6+105;		  //头结构长度+数据段长度
	draw_l.Frameheader.Seq=seq;							//包序号
	/*复制头数据至发送缓冲*/
	memcpy(uart8_tx_buff, &draw_l.Frameheader, sizeof(Frameheader_t));
	/*CRC8校验头*/
	Append_CRC8_Check_Sum(uart8_tx_buff,sizeof(Frameheader_t)); 
	/*复制数据段至发送缓冲*/
  memcpy(uart8_tx_buff + 5, 
				 (uint8_t*)&draw_l.cmd_id, 
				 (sizeof(draw_l.cmd_id)+ sizeof(draw_l.Client_Custom_ID)+ sizeof(draw_l.grapic_data_struct)));
	/*CRC16校验整包数据*/
	Append_CRC16_Check_Sum(uart8_tx_buff, draw_l.Frameheader.DataLength+9);			
	
	
//	/*读取机器人颜色_l.Frameheader.DataLength+9);	
}

int charge_t=0;
void draw_UI_line_4	 
	(uint16_t cmd_id,	uint16_t data_id,uint32_t operate_tpye)
 /*命令字符，				数据内容ID，		 图形操作*/
{
	/*读取机器人颜色和ID*/
	Self_ID=can_data.robot_id;
	if(Self_ID> 100)
		SelfClient_ID = 0x0164 + (Self_ID-100);
	else
		SelfClient_ID = 0x0100 + Self_ID;
	/*定义*/
	static ext_draw_ui_7 draw_l;
	static uint8_t seq=0;
	/*ID设置*/
	draw_l.cmd_id = cmd_id;  
	draw_l.Client_Custom_ID.data_cmd_id= data_id; 
	draw_l.Client_Custom_ID.sender_ID = Self_ID;
	draw_l.Client_Custom_ID.receiver_ID =SelfClient_ID;	
	/*数据*/
	for(int i = 0 ;i < 7 ;i++)
	{
		draw_l.grapic_data_struct[i].graphic_name[0]=i+25;						//图形名字
		draw_l.grapic_data_struct[i].operate_tpye=operate_tpye;		//图形操作
		draw_l.grapic_data_struct[i].graphic_tpye=0;							//图形类型
		draw_l.grapic_data_struct[i].layer=0;											//图层数
		draw_l.grapic_data_struct[i].start_angle=0;								//空
		draw_l.grapic_data_struct[i].end_angle=0;									//空
		draw_l.grapic_data_struct[i].radius=0;										//空

	}
	
//	if(cd.pinwheel_model==0)
//	{
		//装甲板框图
	  draw_l.grapic_data_struct[0].graphic_tpye=1;							//图形类型
		draw_l.grapic_data_struct[0].width=2;											//线条宽度
		draw_l.grapic_data_struct[0].start_x=1600;							//起点坐标X
		draw_l.grapic_data_struct[0].start_y=750;							//起点坐标Y
		draw_l.grapic_data_struct[0].end_x=1800;								//终点坐标X
		draw_l.grapic_data_struct[0].end_y=850;								//终点坐标Y
		draw_l.grapic_data_struct[0].radius=0;										//空

		draw_l.grapic_data_struct[1].graphic_tpye=0;							//图形类型
		draw_l.grapic_data_struct[1].width=8;											//线条宽度
		draw_l.grapic_data_struct[1].start_x=1640;							//起点坐标X
		draw_l.grapic_data_struct[1].start_y=780;							//起点坐标Y
		draw_l.grapic_data_struct[1].end_x=1640;							//终点坐标X
		draw_l.grapic_data_struct[1].end_y=820;								//终点坐标Y
		draw_l.grapic_data_struct[1].radius=0;										//空
		
		draw_l.grapic_data_struct[2].graphic_tpye=0;							//图形类型
		draw_l.grapic_data_struct[2].width=8;											//线条宽度
		draw_l.grapic_data_struct[2].start_x=1760;							//起点坐标X
		draw_l.grapic_data_struct[2].start_y=780;							//起点坐标Y
		draw_l.grapic_data_struct[2].end_x=1760;								//终点坐标X
		draw_l.grapic_data_struct[2].end_y=820;								//终点坐标Y
	
	if(can_data.vision_sign == 1)
	{
			draw_l.grapic_data_struct[3].color=0;
    	draw_l.grapic_data_struct[4].color=0;
	}
	else
	{
		 	draw_l.grapic_data_struct[3].color=8;
    	draw_l.grapic_data_struct[4].color=8;
	}
		
//		if(nuc.vision_sign==1)
//		{
//			
//			draw_l.grapic_data_struct[0].color=0;
//			draw_l.grapic_data_struct[1].color=0;
//			draw_l.grapic_data_struct[2].color=0;
//		}
//		else
//		{
			draw_l.grapic_data_struct[0].color=6;
			draw_l.grapic_data_struct[1].color=6;
			draw_l.grapic_data_struct[2].color=6;
//		}
	
	
	//自瞄装甲板框图
//	if(cd.sentry_model==1)
//		{
//			//自瞄装甲板框图
//			draw_l.grapic_data_struct[3].graphic_tpye=0;							//图形类型
//			draw_l.grapic_data_struct[3].width=7;											//线条宽度
//			draw_l.grapic_data_struct[3].start_x=1680;							//起点坐标X
//			draw_l.grapic_data_struct[3].start_y=830;							//起点坐标Y
//			draw_l.grapic_data_struct[3].end_x=1720;								//终点坐标X
//			draw_l.grapic_data_struct[3].end_y=830;								//终点坐标Y
//			//
//			draw_l.grapic_data_struct[4].graphic_tpye=0;							//图形类型
//			draw_l.grapic_data_struct[4].width=4;											//线条宽度
//			draw_l.grapic_data_struct[4].start_x=1720;							//起点坐标X
//			draw_l.grapic_data_struct[4].start_y=830;							//起点坐标Y
//			draw_l.grapic_data_struct[4].end_x=1700;								//终点坐标X
//			draw_l.grapic_data_struct[4].end_y=770;								//终点坐标Y
//		}
//		else
//    {
//			//自瞄装甲板框图
//			draw_l.grapic_data_struct[3].graphic_tpye=0;							//图形类型
//			draw_l.grapic_data_struct[3].width=4;											//线条宽度
//			draw_l.grapic_data_struct[3].start_x=1670+5;							//起点坐标X
//			draw_l.grapic_data_struct[3].start_y=760+5;							//起点坐标Y
//			draw_l.grapic_data_struct[3].end_x=1730-10;								//终点坐标X
//			draw_l.grapic_data_struct[3].end_y=840-10;								//终点坐标Y
//			//
//			draw_l.grapic_data_struct[4].graphic_tpye=0;							//图形类型
//			draw_l.grapic_data_struct[4].width=4;											//线条宽度
//			draw_l.grapic_data_struct[4].start_x=1730-15;							//起点坐标X
//			draw_l.grapic_data_struct[4].start_y=760+10;							//起点坐标Y
//			draw_l.grapic_data_struct[4].end_x=1670;								//终点坐标X
//			draw_l.grapic_data_struct[4].end_y=800+5;								//终点坐标Y
//		}

	
//	}
//	else
//	{
//		
//		if(nuc.vision_sign==1)
//		{
//			draw_l.grapic_data_struct[3].color=0;
//    	draw_l.grapic_data_struct[4].color=0;
//			draw_l.grapic_data_struct[0].color=0;
//			draw_l.grapic_data_struct[1].color=0;
//			draw_l.grapic_data_struct[2].color=0;
//		}
//		else
//		{
//			if(cd.pinwheel_model==2)
//			{
//				draw_l.grapic_data_struct[3].color=3;
//				draw_l.grapic_data_struct[4].color=3;
//				draw_l.grapic_data_struct[0].color=3;
//				draw_l.grapic_data_struct[1].color=3;
//				draw_l.grapic_data_struct[2].color=3;
//			}
//			else
//			{
//				draw_l.grapic_data_struct[3].color=1;
//				draw_l.grapic_data_struct[4].color=1;
//				draw_l.grapic_data_struct[0].color=1;
//				draw_l.grapic_data_struct[1].color=1;
//				draw_l.grapic_data_struct[2].color=1;
//				
//			}
//		}
//		
//		//风车模式
//		draw_l.grapic_data_struct[0].graphic_tpye=0;							//图形类型
//		draw_l.grapic_data_struct[0].width=10;											//线条宽度
//		draw_l.grapic_data_struct[0].start_x=1700;							//起点坐标X
//		draw_l.grapic_data_struct[0].start_y=800;							//起点坐标Y
//		draw_l.grapic_data_struct[0].end_x=1700;								//终点坐标X
//		draw_l.grapic_data_struct[0].end_y=850;								//终点坐标Y
//		draw_l.grapic_data_struct[0].radius=0;										//空

//		draw_l.grapic_data_struct[1].graphic_tpye=0;							//图形类型
//		draw_l.grapic_data_struct[1].width=10;											//线条宽度
//		draw_l.grapic_data_struct[1].start_x=1700;							//起点坐标X
//		draw_l.grapic_data_struct[1].start_y=800;							//起点坐标Y
//		draw_l.grapic_data_struct[1].end_x=1700+47;							//终点坐标X
//		draw_l.grapic_data_struct[1].end_y=800+15;								//终点坐标Y
//		draw_l.grapic_data_struct[1].radius=0;										//空
//		
//		draw_l.grapic_data_struct[2].graphic_tpye=0;							//图形类型
//		draw_l.grapic_data_struct[2].width=10;											//线条宽度
//		draw_l.grapic_data_struct[2].start_x=1700;							//起点坐标X
//		draw_l.grapic_data_struct[2].start_y=800;							//起点坐标Y
//		draw_l.grapic_data_struct[2].end_x=1700+30;								//终点坐标X
//		draw_l.grapic_data_struct[2].end_y=800-40;								//终点坐标Y
//		//
//		draw_l.grapic_data_struct[3].graphic_tpye=0;							//图形类型
//		draw_l.grapic_data_struct[3].width=10;											//线条宽度
//		draw_l.grapic_data_struct[3].start_x=1700;							//起点坐标X
//		draw_l.grapic_data_struct[3].start_y=800;							//起点坐标Y
//		draw_l.grapic_data_struct[3].end_x=1700-47;								//终点坐标X
//		draw_l.grapic_data_struct[3].end_y=800+15;								//终点坐标Y
//		//
//		draw_l.grapic_data_struct[4].graphic_tpye=0;							//图形类型
//		draw_l.grapic_data_struct[4].width=10;											//线条宽度
//		draw_l.grapic_data_struct[4].start_x=1700;							//起点坐标X
//		draw_l.grapic_data_struct[4].start_y=800;							//起点坐标Y
//		draw_l.grapic_data_struct[4].end_x=1700-30;								//终点坐标X
//		draw_l.grapic_data_struct[4].end_y=800-40;								//终点坐标Y
//			
//	}
	
	/*0红蓝主色1黄色2绿色3橙色4紫红色5粉色6青色7黑色8白色*/
	//
	
	
	//刷新UI
	charge_t+=5;
	charge_t=charge_t%360;
	
	//离线提醒
//	if(led.model==0)	 draw_l.grapic_data_struct[5].color=6;
//	else	          	 
  draw_l.grapic_data_struct[5].color=8;
	
	
	
	draw_l.grapic_data_struct[5].graphic_tpye=4;							//图形类型
	draw_l.grapic_data_struct[5].width=10;											//线条宽度
	draw_l.grapic_data_struct[5].start_x=1700;							//起点坐标X
	draw_l.grapic_data_struct[5].start_y=500;							//起点坐标Y
	draw_l.grapic_data_struct[5].end_x=50;								//终点坐标X
	draw_l.grapic_data_struct[5].end_y=50;								//终点坐标Y
	draw_l.grapic_data_struct[5].start_angle=0;								//空
	draw_l.grapic_data_struct[5].end_angle=charge_t;									//空
	
												//线条宽度
	if(hrpwm.Super_Cap_Power<35)
			draw_l.grapic_data_struct[6].color=3;
  else
	    draw_l.grapic_data_struct[6].color=2;
	draw_l.grapic_data_struct[6].width=20;
	draw_l.grapic_data_struct[6].start_x=680;							//起点坐标X
	draw_l.grapic_data_struct[6].start_y=84;								//起点坐标Y
	draw_l.grapic_data_struct[6].end_x=680+(hrpwm.Super_Cap_Power*560/100);								//终点坐标X
	draw_l.grapic_data_struct[6].end_y=84;								//终点坐标Y

	
	/*头配置*/
	draw_l.Frameheader.SOF = 0xA5; 					//0xA5
  draw_l.Frameheader.DataLength=6+105;		  //头结构长度+数据段长度
	draw_l.Frameheader.Seq=seq;							//包序号
	/*复制头数据至发送缓冲*/
	memcpy(uart8_tx_buff, &draw_l.Frameheader, sizeof(Frameheader_t));
	/*CRC8校验头*/
	Append_CRC8_Check_Sum(uart8_tx_buff,sizeof(Frameheader_t)); 
	/*复制数据段至发送缓冲*/
  memcpy(uart8_tx_buff + 5, 
				 (uint8_t*)&draw_l.cmd_id, 
				 (sizeof(draw_l.cmd_id)+ sizeof(draw_l.Client_Custom_ID)+ sizeof(draw_l.grapic_data_struct)));
	/*CRC16校验整包数据*/
	Append_CRC16_Check_Sum(uart8_tx_buff, draw_l.Frameheader.DataLength+9);			
	
	
//	/*读取机器人颜色_l.Frameheader.DataLength+9);	
}



void draw_UI_line_5	 
	(uint16_t cmd_id,	uint16_t data_id,uint32_t operate_tpye)
 /*命令字符，				数据内容ID，		 图形操作*/
{
	/*读取机器人颜色和ID*/
	Self_ID=can_data.robot_id;
	if(Self_ID> 100)
		SelfClient_ID = 0x0164 + (Self_ID-100);
	else
		SelfClient_ID = 0x0100 + Self_ID;
	/*定义*/
	static ext_draw_ui_5 draw_l;
	static uint8_t seq=0;
	/*ID设置*/
	draw_l.cmd_id = cmd_id;  
	draw_l.Client_Custom_ID.data_cmd_id= data_id; 
	draw_l.Client_Custom_ID.sender_ID = Self_ID;
	draw_l.Client_Custom_ID.receiver_ID =SelfClient_ID;	
	/*数据*/
	for(int i = 0 ;i < 5 ;i++)
	{
		draw_l.grapic_data_struct[i].graphic_name[0]=i+32;						//图形名字
		draw_l.grapic_data_struct[i].operate_tpye=operate_tpye;		//图形操作
		draw_l.grapic_data_struct[i].graphic_tpye=0;							//图形类型
		draw_l.grapic_data_struct[i].layer=0;											//图层数
		draw_l.grapic_data_struct[i].start_angle=0;								//空
		draw_l.grapic_data_struct[i].end_angle=0;									//空
		draw_l.grapic_data_struct[i].radius=0;										//空
	}
	
	//摩擦轮开关
	if(can_data.frictiongear_sign == 1) 		draw_l.grapic_data_struct[0].color=5;
	else                                    draw_l.grapic_data_struct[0].color=8;
	//陀螺开关
	if(can_data.gyro_sign == 1) 		 { draw_l.grapic_data_struct[1].color=5;
		                                 draw_l.grapic_data_struct[3].color=3;
	                                 	 draw_l.grapic_data_struct[4].color=3; }
	else                             { draw_l.grapic_data_struct[1].color=8;
		                                 draw_l.grapic_data_struct[3].color=8;
	                                 	 draw_l.grapic_data_struct[4].color=3; }
	//弹仓开关
	if(can_data.magazine_sign== 1) 	 	draw_l.grapic_data_struct[2].color=8;
	else                              draw_l.grapic_data_struct[2].color=5;
	
	/*0红蓝主色1黄色2绿色3橙色4紫红色5粉色6青色7黑色8白色*/
  //开关标志
	draw_l.grapic_data_struct[0].graphic_tpye=2;							//图形类型
	draw_l.grapic_data_struct[0].width=5;											//线条宽度
	draw_l.grapic_data_struct[0].start_x=170+20;							//起点坐标X
	draw_l.grapic_data_struct[0].start_y=845;							//起点坐标Y
	draw_l.grapic_data_struct[0].end_x=0;								//终点坐标X
	draw_l.grapic_data_struct[0].end_y=0;								//终点坐标Y
	draw_l.grapic_data_struct[0].radius=12;										//空

	draw_l.grapic_data_struct[1].graphic_tpye=2;							//图形类型
	draw_l.grapic_data_struct[1].width=5;											//线条宽度
	draw_l.grapic_data_struct[1].start_x=170+20;							//起点坐标X
	draw_l.grapic_data_struct[1].start_y=810;							//起点坐标Y
	draw_l.grapic_data_struct[1].end_x=0;							//终点坐标X
	draw_l.grapic_data_struct[1].end_y=0;								//终点坐标Y
	draw_l.grapic_data_struct[1].radius=12;										//空
	
	draw_l.grapic_data_struct[2].graphic_tpye=2;							//图形类型
	draw_l.grapic_data_struct[2].width=5;											//线条宽度
	draw_l.grapic_data_struct[2].start_x=170+20;							//起点坐标X
	draw_l.grapic_data_struct[2].start_y=775;							//起点坐标Y
	draw_l.grapic_data_struct[2].end_x=0;								//终点坐标X
	draw_l.grapic_data_struct[2].end_y=0;								//终点坐标Y
	draw_l.grapic_data_struct[2].radius=12;										//空

  //底盘位置圆
  draw_l.grapic_data_struct[3].graphic_tpye=2;							//图形类型
	draw_l.grapic_data_struct[3].width=4;											//线条宽度
	draw_l.grapic_data_struct[3].start_x=960;							//起点坐标X
	draw_l.grapic_data_struct[3].start_y=200;							//起点坐标Y
	draw_l.grapic_data_struct[3].end_x=0;								//终点坐标X
	draw_l.grapic_data_struct[3].end_y=0;								//终点坐标Y
	draw_l.grapic_data_struct[3].radius=50;										//空
  //底盘位置线
	draw_l.grapic_data_struct[4].graphic_tpye=0;							//图形类型
	draw_l.grapic_data_struct[4].width=4;											//线条宽度
	draw_l.grapic_data_struct[4].start_x=960;							//起点坐标X
	draw_l.grapic_data_struct[4].start_y=200;							//起点坐标Y
	draw_l.grapic_data_struct[4].end_x=(uint32_t)(960+50*sin(can_data.angle_gap));								//终点坐标X
	draw_l.grapic_data_struct[4].end_y=(uint32_t)(200+50*cos(can_data.angle_gap));								//终点坐标Y
	
	
	/*头配置*/
	draw_l.Frameheader.SOF = 0xA5; 					//0xA5
  draw_l.Frameheader.DataLength=6+105;		  //头结构长度+数据段长度
	draw_l.Frameheader.Seq=seq;							//包序号
	/*复制头数据至发送缓冲*/
	memcpy(uart8_tx_buff, &draw_l.Frameheader, sizeof(Frameheader_t));
	/*CRC8校验头*/
	Append_CRC8_Check_Sum(uart8_tx_buff,sizeof(Frameheader_t)); 
	/*复制数据段至发送缓冲*/
  memcpy(uart8_tx_buff + 5, 
				 (uint8_t*)&draw_l.cmd_id, 
				 (sizeof(draw_l.cmd_id)+ sizeof(draw_l.Client_Custom_ID)+ sizeof(draw_l.grapic_data_struct)));
	/*CRC16校验整包数据*/
	Append_CRC16_Check_Sum(uart8_tx_buff, draw_l.Frameheader.DataLength+9);			
	
	
//	/*读取机器人颜色_l.Frameheader.DataLength+9);	
}



void draw_UI_line_6	 
	(uint16_t cmd_id,	uint16_t data_id,uint32_t operate_tpye)
 /*命令字符，				数据内容ID，		 图形操作*/
{
	/*读取机器人颜色和ID*/
	Self_ID=can_data.robot_id;
	if(Self_ID> 100)
		SelfClient_ID = 0x0164 + (Self_ID-100);
	else
		SelfClient_ID = 0x0100 + Self_ID;
	/*定义*/
	static ext_draw_ui_1 draw_l;
	static uint8_t seq=0;
	/*ID设置*/
	draw_l.cmd_id = cmd_id;  
	draw_l.Client_Custom_ID.data_cmd_id= data_id; 
	draw_l.Client_Custom_ID.sender_ID = Self_ID;
	draw_l.Client_Custom_ID.receiver_ID =SelfClient_ID;	
	/*数据*/
	/*0红蓝主色1黄色2绿色3橙色4紫红色5粉色6青色7黑色8白色*/
		draw_l.grapic_data_struct.graphic_name[0]=37;						//图形名字
		draw_l.grapic_data_struct.operate_tpye=operate_tpye;		//图形操作
		draw_l.grapic_data_struct.layer=2;											//图层数
		draw_l.grapic_data_struct.start_angle=0;								//空
		draw_l.grapic_data_struct.end_angle=0;									//空
		draw_l.grapic_data_struct.graphic_tpye=2;							//图形类型
		draw_l.grapic_data_struct.width=15;											//线条宽度
		draw_l.grapic_data_struct.start_x=960;							//起点坐标X
		draw_l.grapic_data_struct.start_y=540;							//起点坐标Y
		draw_l.grapic_data_struct.end_x=0;								//终点坐标X
		draw_l.grapic_data_struct.end_y=0;								//终点坐标Y
		draw_l.grapic_data_struct.color=1;
		draw_l.grapic_data_struct.radius=10;										//空
			

 
	/*头配置*/
	draw_l.Frameheader.SOF = 0xA5; 					//0xA5
  draw_l.Frameheader.DataLength=6+15;		  //头结构长度+数据段长度
	draw_l.Frameheader.Seq=seq;							//包序号
	/*复制头数据至发送缓冲*/
	memcpy(uart8_tx_buff, &draw_l.Frameheader, sizeof(Frameheader_t));
	/*CRC8校验头*/
	Append_CRC8_Check_Sum(uart8_tx_buff,sizeof(Frameheader_t)); 
	/*复制数据段至发送缓冲*/
  memcpy(uart8_tx_buff + 5, 
				 (uint8_t*)&draw_l.cmd_id, 
				 (sizeof(draw_l.cmd_id)+ sizeof(draw_l.Client_Custom_ID)+ sizeof(draw_l.grapic_data_struct)));
	/*CRC16校验整包数据*/
	Append_CRC16_Check_Sum(uart8_tx_buff, draw_l.Frameheader.DataLength+9);			
	
	
//	/*读取机器人颜色_l.Frameheader.DataLength+9);	
}




void draw_UI_int_1(uint16_t cmd_id, 						//命令字符
										uint16_t data_id,						//数据内容ID
										int32_t data,								//数据内容	
										uint32_t operate_tpye,			//图形操作（0不操作1增加2修改3删除）
                    uint8_t  name,
										uint32_t start_x,						//起点坐标X
										uint32_t start_y						//起点坐标Y
										)
{
	/*读取机器人颜色和ID*/
	Self_ID=can_data.robot_id;
	if(Self_ID> 100)
		SelfClient_ID = 0x0164 + (Self_ID-100);
	else
		SelfClient_ID = 0x0100 + Self_ID;
	/*定义*/
	static ext_draw_i draw_p;
	static uint8_t seq=0;
	/*ID设置*/
	draw_p.cmd_id = cmd_id;  
	draw_p.Client_Custom_ID.data_cmd_id= data_id; 
	draw_p.Client_Custom_ID.sender_ID = Self_ID;
	draw_p.Client_Custom_ID.receiver_ID = SelfClient_ID;	
	/*发送的数据*/
  draw_p.graphic_data_struct_t_ii.graphic_name[0]=38+name;				//图形名字
	draw_p.graphic_data_struct_t_ii.operate_tpye=operate_tpye;	//图形操作
	draw_p.graphic_data_struct_t_ii.graphic_tpye=6;	//图形类型
	draw_p.graphic_data_struct_t_ii.layer=0;								//图层数
	draw_p.graphic_data_struct_t_ii.color=5;								//图形颜色0红蓝主色1黄色2绿色3橙色4紫红色5粉色6青色7黑色8白色
	draw_p.graphic_data_struct_t_ii.start_angle=20;		//字符大小
	draw_p.graphic_data_struct_t_ii.end_angle=0;				//字符长度
	draw_p.graphic_data_struct_t_ii.width=2;								//线宽
	draw_p.graphic_data_struct_t_ii.start_x=start_x;						//起点坐标X
	draw_p.graphic_data_struct_t_ii.start_y=start_y;						//起点坐标Y
	draw_p.graphic_data_struct_t_ii.i=data;										//无
//	strcpy(draw_p.data,px1);
	/*头配置*/
	draw_p.Frameheader.SOF = 0xA5; 					//0xA5
  draw_p.Frameheader.DataLength=6+15;			//头结构长度+数据段长度
	draw_p.Frameheader.Seq=seq;							//包序号
	/*复制头数据至发送缓冲*/
	memcpy(uart8_tx_buff, &draw_p.Frameheader, sizeof(Frameheader_t));
	/*CRC8校验头*/
	Append_CRC8_Check_Sum(uart8_tx_buff,sizeof(Frameheader_t)); 
	/*复制数据段至发送缓冲*/
  memcpy(uart8_tx_buff + 5, 
				 (uint8_t*)&draw_p.cmd_id, 
				 (sizeof(draw_p.cmd_id)+ sizeof(draw_p.Client_Custom_ID)+ sizeof(draw_p.graphic_data_struct_t_ii)));
	/*CRC16校验整包数据*/
	Append_CRC16_Check_Sum(uart8_tx_buff, draw_p.Frameheader.DataLength+9);			
}



/**********************************************************************************
 * @brief        For the brave souls who get this far: You are the chosen ones,
 * @brief        the valiant knights of programming who toil away, without rest,
 * @brief        fixing our most awful code. To you, true saviors, kings of men,
 * @brief        I say this: never gonna give you up, never gonna let you down,
 * @brief        never gonna run around and desert you. Never gonna make you cry,
 * @brief        never gonna say goodbye. Never gonna tell a lie and hurt you.
  *********************************************************************************/
int16_t send_flag = 1,flag_t=1;
void refree_task()
{
	send_flag++;
	  
    if(send_flag>5)
	     send_flag=1;
	
	  char Char_a[] = "FRICT\nGYRO\nMAGA\n";			//小陀螺,摩擦轮,弹舱
				switch (send_flag)
				{
					case 1:	draw_UI_line_3(0x0301,		0x0104,     2);//加速模式x7
									HAL_UART_Transmit(&huart2,uart8_tx_buff,200,999); 	break;
					case 2:	draw_UI_line_4(0x0301,		0x0104,     2);//自瞄模式x5，UI刷新，电容容量写
									HAL_UART_Transmit(&huart2,uart8_tx_buff,200,999);  	break;
					case 3:	draw_UI_line_5(0x0301,		0x0104,     2);//开关标志x3，底盘位置x2
									HAL_UART_Transmit(&huart2,uart8_tx_buff,200,999);  	break;
//					case 4:	//自瞄瞄准UI
//						if(rc.mouse.press_right == 1&&nuc.aim_sign==1&&cd.yaw_auto<8&&cd.pitch_auto<8){//
//								draw_UI_line_6(0x0301,		0x0101,     1);
//								HAL_UART_Transmit(&huart2,uart8_tx_buff,200,999);	}
//						else{
//								draw_UI_line_6(0x0301,		0x0101,     3);
//								HAL_UART_Transmit(&huart2,uart8_tx_buff,200,999); }	break;
					case 5:
						flag_t++;
					  if(flag_t>30)  flag_t=1;
						switch (flag_t)
						{
							case 1:	draw_UI_char_1(0x0301,0x0110,Char_a,1,50,850);//字符写
												HAL_UART_Transmit(&huart2,uart8_tx_buff,200,999);  	break;
							case 2:	draw_UI_line_2(0x0301,		0x0103,     1);//瞄准线x5写
												HAL_UART_Transmit(&huart2,uart8_tx_buff,200,999);  	break;
							case 3:	draw_UI_line_1(0x0301,		0x0104,     1);//车位线x4，电容框，加速框，UI刷新内圆写
												HAL_UART_Transmit(&huart2,uart8_tx_buff,200,999);  	break;
							//线条写
							case 20:	draw_UI_line_3(0x0301,		0x0104,     1);
												HAL_UART_Transmit(&huart2,uart8_tx_buff,200,999);  	return;
							case 21:	draw_UI_line_4(0x0301,		0x0104,     1);
												HAL_UART_Transmit(&huart2,uart8_tx_buff,200,999);  	return;
							case 22:	draw_UI_line_5(0x0301,		0x0104,     1);
												HAL_UART_Transmit(&huart2,uart8_tx_buff,200,999);  	return;
						}
					break;
				}
		
}

