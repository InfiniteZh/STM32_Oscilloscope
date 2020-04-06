#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "sram.h"
#include "malloc.h"
#include "ILI93xx.h"
#include "led.h"
#include "timer.h"
#include "touch.h"
#include "sdio_sdcard.h"
#include "GUI.h"
#include "pwm.h"
#include "ff.h"
#include "exfuns.h"
#include "w25qxx.h"
#include "includes.h"
#include "WM.h"
#include "DIALOG.h"
#include "adc.h"
#include "exti.h"
#include "key.h"
#include "EmWinHZFont.h"
#include "fontupd.h"
#include "MainDLG.h"
/************************************************
 ALIENTEK ̽����STM32F407������ 
 STemWin GUIBuliderʹ��
 
 UCOSIII���������ȼ��û�������ʹ�ã�ALIENTEK
 ����Щ���ȼ��������UCOSIII��5��ϵͳ�ڲ�����
 ���ȼ�0���жϷ������������� OS_IntQTask()
 ���ȼ�1��ʱ�ӽ������� OS_TickTask()
 ���ȼ�2����ʱ���� OS_TmrTask()
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com  
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/

//�������ȼ�
#define START_TASK_PRIO				3
//�����ջ��С	
#define START_STK_SIZE 				1024
//������ƿ�
OS_TCB StartTaskTCB;
//�����ջ	
CPU_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *p_arg);

//TOUCH����
//�����������ȼ�
#define TOUCH_TASK_PRIO				4
//�����ջ��С
#define TOUCH_STK_SIZE				128
//������ƿ�
OS_TCB TouchTaskTCB;
//�����ջ
CPU_STK TOUCH_TASK_STK[TOUCH_STK_SIZE];
//touch����
void touch_task(void *p_arg);

////LED0����
////�����������ȼ�
//#define LED0_TASK_PRIO 				5
////�����ջ��С
//#define LED0_STK_SIZE				128
////������ƿ�
//OS_TCB Led0TaskTCB;
////�����ջ
//CPU_STK LED0_TASK_STK[LED0_STK_SIZE];
////led0����
//void led0_task(void *p_arg);

//Main ����
//�����������ȼ�
#define MAIN_TASK_PRIO			6
//�����ջ��С
#define MAIN_STK_SIZE			1024
//������ƿ�
OS_TCB MainTaskTCB;
//�����ջ
CPU_STK MAIN_TASK_STK[MAIN_STK_SIZE];
//emwindemo_task����
void main_task(void *p_arg);


int main(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	
	delay_init(168);       	//��ʱ��ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 	//�жϷ�������
	uart_init(115200);    	//���ڲ���������
	TFTLCD_Init();			//��ʼ��LCD
	W25QXX_Init();			//��ʼ��W25Q128
	KEY_Init();
	FSMC_SRAM_Init(); 		//SRAM��ʼ��	
	mem_init(SRAMIN); 		//�ڲ�RAM��ʼ��
	mem_init(SRAMEX); 		//�ⲿRAM��ʼ��
	mem_init(SRAMCCM);		//CCM��ʼ��
	exfuns_init();			//Ϊfatfs�ļ�ϵͳ�����ڴ�
	f_mount(fs[0],"0:",1);	//����SD��
	f_mount(fs[1],"1:",1);	//����FLASH
	font_init();

	TP_Init();				//��ʼ��������
	OSInit(&err);			//��ʼ��UCOSIII
	OS_CRITICAL_ENTER();	//�����ٽ���
	//������ʼ����
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//������ƿ�
				 (CPU_CHAR	* )"start task", 		//��������
                 (OS_TASK_PTR )start_task, 			//������
                 (void		* )0,					//���ݸ��������Ĳ���
                 (OS_PRIO	  )START_TASK_PRIO,     //�������ȼ�
                 (CPU_STK   * )&START_TASK_STK[0],	//�����ջ����ַ
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//�����ջ�����λ
                 (CPU_STK_SIZE)START_STK_SIZE,		//�����ջ��С
                 (OS_MSG_QTY  )0,					//�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
                 (OS_TICK	  )0,					//��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
                 (void   	* )0,					//�û�����Ĵ洢��
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //����ѡ��
                 (OS_ERR 	* )&err);				//��Ÿú�������ʱ�ķ���ֵ
	OS_CRITICAL_EXIT();	//�˳��ٽ���	 
	OSStart(&err);  //����UCOSIII
	while(1);
}

//��ʼ������
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//ͳ������                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//���ʹ���˲����жϹر�ʱ��
    CPU_IntDisMeasMaxCurReset();	
#endif

#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //��ʹ��ʱ��Ƭ��ת��ʱ��
	 //ʹ��ʱ��Ƭ��ת���ȹ���,ʱ��Ƭ����Ϊ1��ϵͳʱ�ӽ��ģ���1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC,ENABLE);//����CRCʱ��
	WM_SetCreateFlags(WM_CF_MEMDEV); //�������д��ڵĴ洢�豸
	GUI_Init();  			//STemWin��ʼ��
	
	OS_CRITICAL_ENTER();	//�����ٽ���
	//Main ����
	OSTaskCreate((OS_TCB*     )&MainTaskTCB,		
				 (CPU_CHAR*   )"MAIN task", 		
                 (OS_TASK_PTR )main_task, 			
                 (void*       )0,					
                 (OS_PRIO	  )MAIN_TASK_PRIO,     
                 (CPU_STK*    )&MAIN_TASK_STK[0],	
                 (CPU_STK_SIZE)MAIN_STK_SIZE/10,	
                 (CPU_STK_SIZE)MAIN_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,  					
                 (void*       )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR*     )&err);

	//����������
	OSTaskCreate((OS_TCB*     )&TouchTaskTCB,		
				 (CPU_CHAR*   )"Touch task", 		
                 (OS_TASK_PTR )touch_task, 			
                 (void*       )0,					
                 (OS_PRIO	  )TOUCH_TASK_PRIO,     
                 (CPU_STK*    )&TOUCH_TASK_STK[0],	
                 (CPU_STK_SIZE)TOUCH_STK_SIZE/10,	
                 (CPU_STK_SIZE)TOUCH_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,  					
                 (void*       )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR*     )&err);			 
	//LED0����
//	OSTaskCreate((OS_TCB*     )&Led0TaskTCB,		
//				 (CPU_CHAR*   )"Led0 task", 		
//                 (OS_TASK_PTR )led0_task, 			
//                 (void*       )0,					
//                 (OS_PRIO	  )LED0_TASK_PRIO,     
//                 (CPU_STK*    )&LED0_TASK_STK[0],	
//                 (CPU_STK_SIZE)LED0_STK_SIZE/10,	
//                 (CPU_STK_SIZE)LED0_STK_SIZE,		
//                 (OS_MSG_QTY  )0,					
//                 (OS_TICK	  )0,  					
//                 (void*       )0,					
//                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
//                 (OS_ERR*     )&err);				 
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);		//����ʼ����			 
	OS_CRITICAL_EXIT();	//�˳��ٽ���
}

void main_task(void *p_arg)
{
		//����Ƥ��
	BUTTON_SetDefaultSkin(BUTTON_SKIN_FLEX); 
	CHECKBOX_SetDefaultSkin(CHECKBOX_SKIN_FLEX);
	DROPDOWN_SetDefaultSkin(DROPDOWN_SKIN_FLEX);
	FRAMEWIN_SetDefaultSkin(FRAMEWIN_SKIN_FLEX);
	HEADER_SetDefaultSkin(HEADER_SKIN_FLEX);
	MENU_SetDefaultSkin(MENU_SKIN_FLEX);
	MULTIPAGE_SetDefaultSkin(MULTIPAGE_SKIN_FLEX);
	PROGBAR_SetDefaultSkin(PROGBAR_SKIN_FLEX);
	RADIO_SetDefaultSkin(RADIO_SKIN_FLEX);
	SCROLLBAR_SetDefaultSkin(SCROLLBAR_SKIN_FLEX);
	SLIDER_SetDefaultSkin(SLIDER_SKIN_FLEX);
	SPINBOX_SetDefaultSkin(SPINBOX_SKIN_FLEX);
	while(1)
	{
		Main_Demo();
	}

}


//extern u8 UDLR;
//extern u8 speed;
//void welcome_task(void *p_arg)
//{
//		//����Ƥ��
//	BUTTON_SetDefaultSkin(BUTTON_SKIN_FLEX); 
//	CHECKBOX_SetDefaultSkin(CHECKBOX_SKIN_FLEX);
//	DROPDOWN_SetDefaultSkin(DROPDOWN_SKIN_FLEX);
//	FRAMEWIN_SetDefaultSkin(FRAMEWIN_SKIN_FLEX);
//	HEADER_SetDefaultSkin(HEADER_SKIN_FLEX);
//	MENU_SetDefaultSkin(MENU_SKIN_FLEX);
//	MULTIPAGE_SetDefaultSkin(MULTIPAGE_SKIN_FLEX);
//	PROGBAR_SetDefaultSkin(PROGBAR_SKIN_FLEX);
//	RADIO_SetDefaultSkin(RADIO_SKIN_FLEX);
//	SCROLLBAR_SetDefaultSkin(SCROLLBAR_SKIN_FLEX);
//	SLIDER_SetDefaultSkin(SLIDER_SKIN_FLEX);
//	SPINBOX_SetDefaultSkin(SPINBOX_SKIN_FLEX);
//	
//	while(1)
//	{
//		Welcome_Demo();
//	}

//}




//extern GRAPH_DATA_Handle  _ahDataYT;
//extern WM_HWIN hItem;
//extern u8 plainFlag;
//extern u16 ADC_Data_Buf0[ADC_DataLen];


//#define Ratio 2		//������չ >1
////OSCILLOSCOPE����
//extern 	EXTI_InitTypeDef   EXTI_InitStructure;

//void oscilloscope_task(void *p_arg)
//{
//	u16 i;
//	u8 j;
//	//����Ƥ��
//	BUTTON_SetDefaultSkin(BUTTON_SKIN_FLEX); 
//	CHECKBOX_SetDefaultSkin(CHECKBOX_SKIN_FLEX);
//	DROPDOWN_SetDefaultSkin(DROPDOWN_SKIN_FLEX);
//	FRAMEWIN_SetDefaultSkin(FRAMEWIN_SKIN_FLEX);
//	HEADER_SetDefaultSkin(HEADER_SKIN_FLEX);
//	MENU_SetDefaultSkin(MENU_SKIN_FLEX);
//	MULTIPAGE_SetDefaultSkin(MULTIPAGE_SKIN_FLEX);
//	PROGBAR_SetDefaultSkin(PROGBAR_SKIN_FLEX);
//	RADIO_SetDefaultSkin(RADIO_SKIN_FLEX);
//	SCROLLBAR_SetDefaultSkin(SCROLLBAR_SKIN_FLEX);
//	SLIDER_SetDefaultSkin(SLIDER_SKIN_FLEX);
//	SPINBOX_SetDefaultSkin(SPINBOX_SKIN_FLEX);
//	TIM3_Int_Init(50-1,84-1);
//	EXTIX_Init();
//	CreateFramewin_Oscilloscope();
//	Adc_Init();         //��ʼ��ADC
//	DMA2_Config();
//	ADC_RegularChannelConfig(ADC1, 2, 1, ADC_SampleTime_15Cycles );	//ADC1,ADCͨ��,480������,��߲���ʱ�������߾�ȷ��		
//	while(1)
//	{
//		if(plainFlag)
//		{
//			for(i=0;i<ADC_DataLen;i++)
//			{
//				for(j=0;j<Ratio;j++)
//					GRAPH_DATA_YT_AddValue(_ahDataYT,(I16)ADC_Data_Buf0[i]*1.0/4096*300);
//			}
//			GUI_Delay(100);
//			plainFlag=0;
//			EXTI_InitStructure.EXTI_LineCmd = ENABLE;//�ж���ʹ��
//			EXTI_Init(&EXTI_InitStructure);//����
//		}
//	}
//}

//TOUCH����
void touch_task(void *p_arg)
{
	OS_ERR err;
	while(1)
	{
		GUI_TOUCH_Exec();	
		OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_PERIODIC,&err);//��ʱ5ms
	}
}

////LED0����
//void led0_task(void *p_arg)
//{
//	OS_ERR err;
//	while(1)
//	{
//		LED0 = !LED0;
//		OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_PERIODIC,&err);//��ʱ500ms
//	}
//}

