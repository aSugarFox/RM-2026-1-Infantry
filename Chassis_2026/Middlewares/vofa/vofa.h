#ifndef __VOFA_H
#define __VOFA_H
#include "main.h"
typedef union     //定义结构体，参考https://blog.csdn.net/ls667/article/details/50811519博客
{
	float fdata;
	unsigned long ldata;
}FloatLongType;

void vofa_send_dat(float num[],int n);
void vofa_send_dat_usb(float num[],int n);
#endif
