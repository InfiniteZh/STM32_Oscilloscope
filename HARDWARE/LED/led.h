#ifndef _LED_H
#define _LED_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//LED�˿ڶ���
#define LED8 PFout(9)
#define LED9 PFout(10)
#define LED0 PBout(6)
#define LED1 PBout(7)
#define LED2 PBout(8)
#define LED3 PBout(9)
#define LED4 PBout(10)
#define LED5 PBout(11)
#define LED6 PBout(12)
#define LED7 PBout(13)

void LED_Init(void);  //��ʼ��
#endif
