#ifndef _LED_H
#define _LED_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//LED驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//LED端口定义
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

void LED_Init(void);  //初始化
#endif
