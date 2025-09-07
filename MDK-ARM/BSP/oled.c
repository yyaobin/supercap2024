/*
 *           _____                   _______                   _____                   _______                   _____                    _____                    _____                _____                    _____                    _____          
 *          /\    \                 /::\    \                 /\    \                 /::\    \                 /\    \                  /\    \                  /\    \              /\    \                  /\    \                  /\    \
 *         /::\    \               /::::\    \               /::\    \               /::::\    \               /::\____\                /::\    \                /::\    \            /::\    \                /::\    \                /::\    \
 *        /::::\    \             /::::::\    \             /::::\    \             /::::::\    \             /::::|   |               /::::\    \              /::::\    \           \:::\    \              /::::\    \              /::::\    \
 *       /::::::\    \           /::::::::\    \           /::::::\    \           /::::::::\    \           /:::::|   |              /::::::\    \            /::::::\    \           \:::\    \            /::::::\    \            /::::::\    \
 *      /:::/\:::\    \         /:::/~~\:::\    \         /:::/\:::\    \         /:::/~~\:::\    \         /::::::|   |             /:::/\:::\    \          /:::/\:::\    \           \:::\    \          /:::/\:::\    \          /:::/\:::\    \
 *     /:::/__\:::\    \       /:::/    \:::\    \       /:::/__\:::\    \       /:::/    \:::\    \       /:::/|::|   |            /:::/__\:::\    \        /:::/__\:::\    \           \:::\    \        /:::/__\:::\    \        /:::/__\:::\    \
 *    /::::\   \:::\    \     /:::/    / \:::\    \     /::::\   \:::\    \     /:::/    / \:::\    \     /:::/ |::|   |           /::::\   \:::\    \       \:::\   \:::\    \          /::::\    \      /::::\   \:::\    \      /::::\   \:::\    \
 *   /::::::\   \:::\    \   /:::/____/   \:::\____\   /::::::\   \:::\    \   /:::/____/   \:::\____\   /:::/  |::|___|______    /::::::\   \:::\    \    ___\:::\   \:::\    \        /::::::\    \    /::::::\   \:::\    \    /::::::\   \:::\    \
 *  /:::/\:::\   \:::\____\ |:::|    |     |:::|    | /:::/\:::\   \:::\ ___\ |:::|    |     |:::|    | /:::/   |::::::::\    \  /:::/\:::\   \:::\    \  /\   \:::\   \:::\    \      /:::/\:::\    \  /:::/\:::\   \:::\    \  /:::/\:::\   \:::\____\
 * /:::/  \:::\   \:::|    ||:::|____|     |:::|    |/:::/__\:::\   \:::|    ||:::|____|     |:::|    |/:::/    |:::::::::\____\/:::/  \:::\   \:::\____\/::\   \:::\   \:::\____\    /:::/  \:::\____\/:::/__\:::\   \:::\____\/:::/  \:::\   \:::|    |
 * \::/   |::::\  /:::|____| \:::\    \   /:::/    / \:::\   \:::\  /:::|____| \:::\    \   /:::/    / \::/    / ~~~~~/:::/    /\::/    \:::\  /:::/    /\:::\   \:::\   \::/    /   /:::/    \::/    /\:::\   \:::\   \::/    /\::/   |::::\  /:::|____|
 *  \/____|:::::\/:::/    /   \:::\    \ /:::/    /   \:::\   \:::\/:::/    /   \:::\    \ /:::/    /   \/____/      /:::/    /  \/____/ \:::\/:::/    /  \:::\   \:::\   \/____/   /:::/    / \/____/  \:::\   \:::\   \/____/  \/____|:::::\/:::/    / 
 *        |:::::::::/    /     \:::\    /:::/    /     \:::\   \::::::/    /     \:::\    /:::/    /                /:::/    /            \::::::/    /    \:::\   \:::\    \      /:::/    /            \:::\   \:::\    \            |:::::::::/    /  
 *        |::|\::::/    /       \:::\__/:::/    /       \:::\   \::::/    /       \:::\__/:::/    /                /:::/    /              \::::/    /      \:::\   \:::\____\    /:::/    /              \:::\   \:::\____\           |::|\::::/    /   
 *        |::| \::/____/         \::::::::/    /         \:::\  /:::/    /         \::::::::/    /                /:::/    /               /:::/    /        \:::\  /:::/    /    \::/    /                \:::\   \::/    /           |::| \::/____/    
 *        |::|  ~|                \::::::/    /           \:::\/:::/    /           \::::::/    /                /:::/    /               /:::/    /          \:::\/:::/    /      \/____/                  \:::\   \/____/            |::|  ~|          
 *        |::|   |                 \::::/    /             \::::::/    /             \::::/    /                /:::/    /               /:::/    /            \::::::/    /                                 \:::\    \                |::|   |          
 *        \::|   |                  \::/____/               \::::/    /               \::/____/                /:::/    /               /:::/    /              \::::/    /                                   \:::\____\               \::|   |          
 *         \:|   |                   ~~                      \::/____/                 ~~                      \::/    /                \::/    /                \::/    /                                     \::/    /                \:|   |          
 *          \|___|                                            ~~                                                \/____/                  \/____/                  \/____/                                       \/____/                  \|___|          
 * 
 *                                                                                         /RoboMaster2022��ȥ����/
 */



#include "oled.h"
#include  "i2c.h"
#include "oledfont.h"
#include  "PWM_Control.h"
#include  "stdio.h"
#include "lib.h"
#include "BSP_CAN_FD.h"



void Page_1(void);
void Page_2(void);
//void Page_3();
//void Page_4();



uint8_t out_t[10];
int16_t  page_last=4;



void oled_task()
{
	
	if(page_last!=can_data.page)
		   OLED_CLS();//����
	page_last=can_data.page;
	
	if(can_data.page>4) can_data.page=4;
	else if(can_data.page<0) can_data.page=0;
	
	if(can_data.page==0)//��ʾ������ѹ
	    Page_1();
	else if(can_data.page==1)//��ʾ�����Ƿ�����
	    Page_2();
//	else if(can_data.page==2)   
//		  Page_3();
//	else if(can_data.page==3) 
//  		Page_4();
	
}




void Page_2()
{
//		if(adc.U_Out_Cap>6000)
//		OLED_ShowStr( 0,0,(unsigned char*)"Cap-----OK"       ,2);
//		else
//		OLED_ShowStr( 0,0,(unsigned char*)"Cap-----NO"       ,2);
	  
	  //FPS
	  int cap_v=hrpwm.fps;
		out_t[0]=(int)(cap_v/1000);
		out_t[1]=(int)((cap_v-out_t[0]*1000)/100);
		out_t[2]=(int)((cap_v-out_t[0]*1000-out_t[1]*100)/10);
		out_t[3]=(int)(cap_v-out_t[0]*1000-out_t[1]*100-out_t[2]*10);
	
	  OLED_ShowStr( 0,0,(unsigned char*)"FPS:"      ,2);
		OLEDShowData(35,0,(uint8_t)out_t[0]);
		OLEDShowData(45,0,(uint8_t)out_t[1]);
		OLEDShowData(55,0,(uint8_t)out_t[2]);
		OLEDShowData(65,0,(uint8_t)out_t[3]);
	  OLED_ShowStr(75,0,(unsigned char*)"Hz"       ,2);
		
	  //��������
		if(adc.U_In_Vcc>10000)
		OLED_ShowStr(0 ,2,(unsigned char*)"VCC-----OK"       ,2);
		else
		OLED_ShowStr(0 ,2,(unsigned char*)"VCC-----NO"       ,2);
		//CAN����
		if(can_data.cnt>10)
		OLED_ShowStr(0 ,4,(unsigned char*)"CAN-----OK"       ,2);
		else
		OLED_ShowStr(0 ,4,(unsigned char*)"CAN-----NO"       ,2);
		//����ģʽ
		if(can_data.Model==2)
		OLED_ShowStr(0 ,6,(unsigned char*)"Model--Charge"       ,2);
		else if(can_data.Model==1)
		OLED_ShowStr(0 ,6,(unsigned char*)"Model--Auto"       ,2);
		else if(can_data.Model==3)
		OLED_ShowStr(0 ,6,(unsigned char*)"Model--Turnoff"       ,2);
}



void Page_1()
{
		float cap_v;
    
		//������ʾ�̶����ַ�
		OLED_ShowStr( 0,0,(unsigned char*)"Iin :"       ,2);
		OLED_ShowStr(68,0,(unsigned char*)"."           ,2);
		OLED_ShowStr(95,0,(unsigned char*)"A"           ,2);
		OLED_ShowStr(0 ,2,(unsigned char*)"Vin :"       ,2);
		OLED_ShowStr(68,2,(unsigned char*)"."           ,2);
		OLED_ShowStr(95,2,(unsigned char*)"V"           ,2);
		OLED_ShowStr(0 ,4,(unsigned char*)"Vout:"       ,2);
		OLED_ShowStr(68,4,(unsigned char*)"."           ,2);
		OLED_ShowStr(95,4,(unsigned char*)"V"           ,2);
		OLED_ShowStr(0 ,6,(unsigned char*)"Iout:"       ,2);
		OLED_ShowStr(68,6,(unsigned char*)"."           ,2);
		OLED_ShowStr(95,6,(unsigned char*)"A"           ,2);

	  //��ʾ�����ѹ
		cap_v=adc.U_Out_Cap*0.001f;
		out_t[0]=(int)(cap_v/10);
		out_t[1]=(int)(cap_v-out_t[0]*10);
		out_t[2]=(int)((cap_v-(int)cap_v)*10);
		out_t[3]=(int)((cap_v-(int)cap_v-out_t[2]*0.1f)*100);
		
		OLEDShowData(50,4,(uint8_t)out_t[0]);
		OLEDShowData(60,4,(uint8_t)out_t[1]);
		OLEDShowData(75+2,4,(uint8_t)out_t[2]);
		OLEDShowData(85+2,4,(uint8_t)out_t[3]);
		
	  //��ʾ�������
		if(adc.I_Out_Cap<0){
				cap_v=-adc.I_Out_Cap*0.001f;
				 OLED_ShowStr(40,6,(unsigned char*)"-",2);
		}else{
				OLED_ShowStr(40,6,(unsigned char*)"+",2);
				cap_v=adc.I_Out_Cap*0.001f;   }
		out_t[0]=(int)(cap_v/10);
		out_t[1]=(int)(cap_v-out_t[0]*10);
		out_t[2]=(int)((cap_v-(int)cap_v)*10);
		out_t[3]=(int)((cap_v-(int)cap_v-out_t[2]*0.1f)*100);

		OLEDShowData(50,6,(uint8_t)out_t[0]);
		OLEDShowData(60,6,(uint8_t)out_t[1]);
		OLEDShowData(75+2,6,(uint8_t)out_t[2]);
		OLEDShowData(85+2,6,(uint8_t)out_t[3]);
		
		//��ʾ�����ѹ
		cap_v=adc.U_In_Vcc*0.001f;
		out_t[0]=(int)(cap_v/10);
		out_t[1]=(int)(cap_v-out_t[0]*10);
		out_t[2]=(int)((cap_v-(int)cap_v)*10);
		out_t[3]=(int)((cap_v-(int)cap_v-out_t[2]*0.1f)*100);
		
		OLEDShowData(50,2,(uint8_t)out_t[0]);
		OLEDShowData(60,2,(uint8_t)out_t[1]);
		OLEDShowData(75+2,2,(uint8_t)out_t[2]);
		OLEDShowData(85+2,2,(uint8_t)out_t[3]);
		
	  //��ʾ�������
		if(adc.I_In_Vcc<0){
				cap_v=-adc.I_In_Vcc*0.001f;
				OLED_ShowStr(40,0,(unsigned char*)"-",2);
		}else{
				OLED_ShowStr(40,0,(unsigned char*)"+",2);
				cap_v=adc.I_In_Vcc*0.001f;   }  
		out_t[0]=(int)(cap_v/10);
		out_t[1]=(int)(cap_v-out_t[0]*10);
		out_t[2]=(int)((cap_v-(int)cap_v)*10);
		out_t[3]=(int)((cap_v-(int)cap_v-out_t[2]*0.1f)*100);
		
		OLEDShowData(50,0,(uint8_t)out_t[0]);
		OLEDShowData(60,0,(uint8_t)out_t[1]);
		OLEDShowData(75+2,0,(uint8_t)out_t[2]);
		OLEDShowData(85+2,0,(uint8_t)out_t[3]);
}


void MX_OLED_Init(void)
{
	//��ʼ��OLED
	OLED_Init();
	OLED_CLS();
	OLED_ON();
}



void WriteCmd(unsigned char I2C_Command)//д����
{
	HAL_I2C_Mem_Write(&hi2c1,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&I2C_Command,1,100);	
}

void WriteDat(unsigned char I2C_Data)//д���� 
{
	HAL_I2C_Mem_Write(&hi2c1,OLED0561_ADD,DAT,I2C_MEMADD_SIZE_8BIT,&I2C_Data,1,100);	
}

void OLED_Init(void)
{
	HAL_Delay(200); //�������ʱ����Ҫ

	WriteCmd(0xAE); //display off
	WriteCmd(0x20);	//Set Memory Addressing Mode	
	WriteCmd(0x10);	//00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	WriteCmd(0xb0);	//Set Page Start Address for Page Addressing Mode,0-7
	WriteCmd(0xc8);	//Set COM Output Scan Direction
	WriteCmd(0x00); //---set low column address
	WriteCmd(0x10); //---set high column address
	WriteCmd(0x40); //--set start line address
	WriteCmd(0x81); //--set contrast control register
	WriteCmd(0xff); //���ȵ��� 0x00~0xff
	WriteCmd(0xa1); //--set segment re-map 0 to 127
	WriteCmd(0xa6); //--set normal display
	WriteCmd(0xa8); //--set multiplex ratio(1 to 64)
	WriteCmd(0x3F); //
	WriteCmd(0xa4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	WriteCmd(0xd3); //-set display offset
	WriteCmd(0x00); //-not offset
	WriteCmd(0xd5); //--set display clock divide ratio/oscillator frequency
	WriteCmd(0xf0); //--set divide ratio
	WriteCmd(0xd9); //--set pre-charge period
	WriteCmd(0x22); //
	WriteCmd(0xda); //--set com pins hardware configuration
	WriteCmd(0x12);
	WriteCmd(0xdb); //--set vcomh
	WriteCmd(0x20); //0x20,0.77xVcc
	WriteCmd(0x8d); //--set DC-DC enable
	WriteCmd(0x14); //
	WriteCmd(0xaf); //--turn on oled panel
	
	HAL_Delay(100);
}

void OLED_SetPos(unsigned char x, unsigned char y) //������ʼ������
{ 
	WriteCmd(0xb0+y);
	WriteCmd(((x&0xf0)>>4)|0x10);
	WriteCmd((x&0x0f)|0x01);
}

void OLED_Fill(unsigned char fill_Data)//ȫ�����
{
	unsigned char m,n;
	for(m=0;m<8;m++)
	{
		WriteCmd(0xb0+m);		//page0-page1
		WriteCmd(0x00);		//low column start address
		WriteCmd(0x10);		//high column start address
		for(n=0;n<128;n++)
		{
			WriteDat(fill_Data);
		}
	}
}

void OLED_CLS(void)//����
{
	OLED_Fill(0x00);
//	HAL_Delay(100);
}

void OLED_ON(void)
{
	WriteCmd(0X8D);  //���õ�ɱ�
	WriteCmd(0X14);  //������ɱ�
	WriteCmd(0XAF);  //OLED����
}

void OLED_OFF(void)
{
	WriteCmd(0X8D);  //���õ�ɱ�
	WriteCmd(0X10);  //�رյ�ɱ�
	WriteCmd(0XAE);  //OLED����
}


// Parameters     : x,y -- ��ʼ������(x:0~127, y:0~7); ch[] -- Ҫ��ʾ���ַ���; TextSize -- �ַ���С(1:6*8 ; 2:8*16)
// Description    : ��ʾcodetab.h�е�ASCII�ַ�,��6*8��8*16��ѡ��
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize)
{
	unsigned char c = 0,i = 0,j = 0;
	switch(TextSize)
	{
		case 1:
		{
			while(ch[j] != '\0')
			{
				c = ch[j] - 32;
				if(x > 126)
				{
					x = 0;
					y++;
				}
				OLED_SetPos(x,y);
				for(i=0;i<6;i++)
				WriteDat(F6x8[c][i]);
				x += 6;
				j++;
			}
		}break;
		case 2:
		{
			while(ch[j] != '\0')
			{
				c = ch[j] - 32;
				if(x > 120)
				{
					x = 0;
					y++;
				}
				OLED_SetPos(x,y);
				for(i=0;i<8;i++)
				WriteDat(F8X16[c*16+i]);
				OLED_SetPos(x,y+1);
				for(i=0;i<8;i++)
				WriteDat(F8X16[c*16+i+8]);
				x += 8;
				j++;
			}
		}break;
	}
}


// Parameters     : x,y -- ��ʼ������(x:0~127, y:0~7); N:������.h�е�����
// Description    : ��ʾASCII_8x16.h�еĺ���,16*16����
void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N)
{
	unsigned char wm=0;
	unsigned int  adder=8*N;
	OLED_SetPos(x , y);
	for(wm = 0;wm < 8;wm++)
	{
		WriteDat(F8x8_t[adder]);
		adder += 1;
	}
//	OLED_SetPos(x,y + 1);
//	for(wm = 0;wm < 8;wm++)
//	{
//		WriteDat(F8x8_t[adder]);
//		adder += 1;
//	}
}



// Parameters     : x0,y0 -- ��ʼ������(x0:0~127, y0:0~7); x1,y1 -- ���Խ���(������)������(x1:1~128,y1:1~8)
// Description    : ��ʾBMPλͼ
void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[])
{
	unsigned int j=0;
	unsigned char x,y;

	if(y1%8==0)
	y = y1/8;
	else
	y = y1/8 + 1;
	for(y=y0;y<y1;y++)
	{
		OLED_SetPos(x0,y);
		for(x=x0;x<x1;x++)
		{
			WriteDat(BMP[j++]);
		}
	}
}

void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size)
{      	
	unsigned char c=0,i=0;	
	c=chr-' ';//�õ�ƫ�ƺ��ֵ			
	if(x>128-1){x=0;y=y+2;}
	if(Char_Size ==16)
	{
		OLED_SetPos(x,y);	
		for(i=0;i<8;i++)
		WriteDat(F8X16[c*16+i]);
		OLED_SetPos(x,y+1);
		for(i=0;i<8;i++)
		WriteDat(F8X16[c*16+i+8]);
	}
	else 
	{	
		OLED_SetPos(x,y);
		for(i=0;i<6;i++)
		WriteDat(F6x8[c][i]);
	}
}

uint32_t oled_pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}	


//��ʾ2������
//x,y :�������	 
//len :���ֵ�λ��
//size:�����С
//mode:ģʽ	0,���ģʽ;1,����ģʽ
//num:��ֵ(0~4294967295);	 		  
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size2)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size2/2)*t,y,' ',size2);
				continue;
			}
			else 
				enshow=1; 
		}
		OLED_ShowChar(x+(size2/2)*t,y,temp+'0',size2); 
	}
} 

//���ַ�������ʽ��ʾ����
//x,y :�������	
//temp:����
void OLEDShowData(uint8_t x,uint8_t y,uint8_t temp)
{
	switch(temp)
	{
		case  0: OLED_ShowStr(x,y,"0",2);
		break;
		case  1: OLED_ShowStr(x,y,"1",2);
		break;
		case  2: OLED_ShowStr(x,y,"2",2);
		break;
		case  3: OLED_ShowStr(x,y,"3",2);
		break;
		case  4: OLED_ShowStr(x,y,"4",2);
		break;
		case  5: OLED_ShowStr(x,y,"5",2);
		break;
		case  6: OLED_ShowStr(x,y,"6",2);
		break;
		case  7: OLED_ShowStr(x,y,"7",2);
		break;
		case  8: OLED_ShowStr(x,y,"8",2);
		break;
		case  9: OLED_ShowStr(x,y,"9",2);
		break;		
	}
}




