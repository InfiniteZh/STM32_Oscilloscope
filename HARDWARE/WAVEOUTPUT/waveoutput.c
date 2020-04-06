#include "waveoutput.h"

/********�������Ҳ��������***********/
void SineWave_Data( u16 cycle ,u16 *D)
{
	u16 i;
	for( i=0;i<cycle;i++)
	{
		D[i]=(u16)((Um/2*sin(( 1.0*i/(cycle-1))*2*PI)+Um/2)*4095/3.3);
	}
}

/********���ɾ�ݲ��������***********/
void SawTooth_Data( u16 cycle ,u16 *D)
{
	u16 i;
	for( i=0;i<cycle;i++)
	{
		D[i]= (u16)(1.0*1.0*i/255*4095/3.3);
	}
}
/********���ɷ����������***********/
void Square_Data( u16 cycle ,u16 *D)
{
	u16 i;
	for( i=0;i<cycle;i++)
		{
			if(i<(cycle/2))
				D[i]=(u16)1.0*4095.0/3.3;
			else
				D[i]=(u16)1.0;
				
		}
//	for( i=0;i<cycle;i++)
//		{
//			if(i<(cycle/2))
//				D[i]=(u16)1241.0;
//			else
//				D[i]=(u16)1.0;
//				
//		}

}
/********�������ǲ��������***********/
void Triangle_Data( u16 cycle ,u16 *D)
{
	u16 i;
	for( i=0;i<cycle;i++)
		{
			 if(i<cycle/2)
				D[i]= (u16)(2.0*i/255*4095.0/3.3);
			 else
				D[i]= (u16)(2.0*(255-i)/255*4095.0/3.3);
		}


}
/******************���Ҳ��α�***********************/
#ifdef  Sine_WaveOutput_Enable 	
     u16 SineWave_Value[256];		//���ú�������
#endif
/******************��ݲ��α�***********************/
#ifdef  SawTooth_WaveOutput_Enable
     u16 SawToothWave_Value[256];  //���ú�������
#endif	
/******************�������α�***********************/
#ifdef  Square_WaveOutput_Enable 	
     u16 SquareWave_Value[256];		//���ú�������
#endif	
/******************���ǲ����α�***********************/
#ifdef  Triangle_WaveOutput_Enable 	
     u16 TriangleWave_Value[256];		//���ú�������
#endif	

/******DAC�Ĵ�����ַ����*******/	
#define DAC_DHR12R1    (u32)&(DAC->DHR12R1)   //DACͨ��1����Ĵ�����ַ



/****************���ų�ʼ��******************/
void SineWave_GPIO_Config()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);   //��ʱ��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;          //��ͨ���ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			//�������	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;       //�������
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 ; //ѡ������
	GPIO_SetBits(GPIOA,GPIO_Pin_4)	;	//�������
	GPIO_Init(GPIOA, &GPIO_InitStructure);		//��ʼ��
}

/******************DAC��ʼ��*************************/
void SineWave_DAC_Config()
{
	DAC_InitTypeDef            DAC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);//��DACʱ��
	
  /**************DAC�ṹ��ʼ��������Ҫ�������޲���*******************/
	DAC_StructInit(&DAC_InitStructure);		
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;//����������
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable; //��ʹ���������
	
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_T4_TRGO;//ѡ��DAC����ԴΪTIM2
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);//��ʼ��
	DAC_Cmd(DAC_Channel_1, ENABLE);	   //ʹ��DACͨ��1
	DAC_SetChannel1Data(DAC_Align_12b_R, 0);  //12λ�Ҷ������ݸ�ʽ����DACֵ
	DAC_DMACmd(DAC_Channel_1, ENABLE); //ʹ��DACͨ��1��DMA
}
		 
/*********��ʱ������************/
void SineWave_TIM_Config( u32 Wave1_Fre )
{
	TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//��ʱ��
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
 	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;     //��Ԥ��Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0; //����Ƶ	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period = Wave1_Fre;    
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_SelectOutputTrigger(TIM4, TIM_TRGOSource_Update);//����TIM2�������Ϊ����ģʽ
}

/*********DMA����***********/
void SineWave_DMA_Config( u16 *Wave1_Mem )
{					
	DMA_InitTypeDef            DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);//��DMA1��ʱ��
	
	DMA_StructInit( &DMA_InitStructure);		//DMA�ṹ��ʼ��
	DMA_InitStructure.DMA_Channel=DMA_Channel_7;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//�Ӵ洢��������
	DMA_InitStructure.DMA_BufferSize = 256;//�����С��һ��Ϊ256��
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//�ڴ��ַ����
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//���Ϊ����
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//���Ϊ����
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//���ȼ����ǳ���
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//ѭ������ģʽ 
	DMA_InitStructure.DMA_PeripheralBaseAddr = DAC_DHR12R1;//�����ַΪDACͨ��1���ݼĴ���
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Wave1_Mem;//�ڴ��ַΪ���������������
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);//��ʼ��
	DMA_Cmd(DMA1_Stream5, ENABLE);  //ʹ��DMAͨ��3       
}

/***********���Ҳ���ʼ��***************/
u16 *add1;
u16 f1;
void SineWave_Init(u8 Wave1)
{
	SineWave_Data( N ,SineWave_Value);		//���ɲ��α�1
	SawTooth_Data( N ,SawToothWave_Value);//���ɲ��α�2
	Square_Data( N ,SquareWave_Value);		//���ɲ��α�3
	Triangle_Data( N ,TriangleWave_Value);//���ɲ��α�4
	if( Wave1==0x00)   add1=SineWave_Value;
	if( Wave1==0x01)	 add1=SawToothWave_Value;
	if( Wave1==0x02)	 add1=SquareWave_Value;
	if( Wave1==0x03)	 add1=TriangleWave_Value;
	f1=(u16)(84000000/512*2/1000);
	SineWave_GPIO_Config();			  //��ʼ������ 
	SineWave_TIM_Config( f1 );			  //��ʼ����ʱ�� 
	SineWave_DAC_Config();			  //��ʼ��DAC
	SineWave_DMA_Config( add1 );			  //��ʼ��DMA
  TIM_Cmd(TIM4, ENABLE);			 //ʹ��TIM2,��ʼ��������  
}

void SineWave(u8 Wave1)
{
	if( Wave1==0x00)   add1=SineWave_Value;
	if( Wave1==0x01)	 add1=SawToothWave_Value;
	if( Wave1==0x02)	 add1=SquareWave_Value;
	if( Wave1==0x03)	 add1=TriangleWave_Value;
	DMA_Cmd(DMA1_Stream5,DISABLE);
	SineWave_DMA_Config( add1 );			  //��ʼ��DMA
	DMA_Cmd(DMA1_Stream5, ENABLE);  //ʹ��DMAͨ��3       

}


void Set_WaveFre(u16 fre)
{
	u16 reload;	
	reload=(u16)(84000000/512*2/fre);
	TIM_SetAutoreload( TIM4 ,reload);
}


