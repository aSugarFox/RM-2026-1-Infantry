#include "vofa.h"
#include "usart.h"

/****************************************************
将浮点数f转化为4个字节数据存放在byte[4]中
*****************************************************/
void Float_to_Byte(float f,unsigned char byte[])   //参考https://blog.csdn.net/ls667/article/details/50811519博客
{
	FloatLongType fl;
	fl.fdata=f;
	byte[0]=(unsigned char)fl.ldata;
	byte[1]=(unsigned char)(fl.ldata>>8);
	byte[2]=(unsigned char)(fl.ldata>>16);
	byte[3]=(unsigned char)(fl.ldata>>24);
}

uint8_t byte[4]={0};    //定义数组
uint8_t send_date[4]={0};

void vofa_send_dat(float num[],int n)
{
	uint8_t t_test=0;
	int i;
	//printf("%d\t\n",num);
		//发送数据，绘制图像
	
	for(i = 0;i < n;i++)
	{
		Float_to_Byte(num[i],byte);
		for(t_test=0;t_test<4;t_test++)
		{
			HAL_UART_Transmit(&huart6, &byte[t_test], 1, 1000);         //向串口6发送数据
			while(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) != SET);
		}
	
	}
	
	send_date[0]=0X00;send_date[1]=0X00;
	send_date[2]=0X80;send_date[3]=0X7f;
	for(t_test=0;t_test<4;t_test++)
	{
		HAL_UART_Transmit(&huart6, &send_date[t_test], 1, 1000);         //向串口6发送数据
		while(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) != SET);
	}
}

void vofa_send_dat_usb(float num[],int n)
{
	uint8_t t_test=0;
	int i;
	//printf("%d\t\n",num);
		//发送数据，绘制图像
	
	for(i = 0;i < n;i++)
	{
		Float_to_Byte(num[i],byte);
		for(t_test=0;t_test<4;t_test++)
		{
			CDC_Transmit_FS(&byte[t_test],1);// 通过USB CDC发送调试信息到PC
			// HAL_UART_Transmit(&huart6, &byte[t_test], 1, 1000);         //向串口6发送数据
			// while(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) != SET);
		}
	
	}
	
	send_date[0]=0X00;send_date[1]=0X00;
	send_date[2]=0X80;send_date[3]=0X7f;
	for(t_test=0;t_test<4;t_test++)
	{
		CDC_Transmit_FS(&send_date[t_test],1);
		// HAL_UART_Transmit(&huart6, &send_date[t_test], 1, 1000);         //向串口6发送数据
		// while(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) != SET);
	}
}
