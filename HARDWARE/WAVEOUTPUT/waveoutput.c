#include "waveoutput.h"

/********生成正弦波形输出表***********/
void SineWave_Data( u16 cycle ,u16 *D)
{
	u16 i;
	for( i=0;i<cycle;i++)
	{
		D[i]=(u16)((Um/2*sin(( 1.0*i/(cycle-1))*2*PI)+Um/2)*4095/3.3);
	}
}

/********生成锯齿波形输出表***********/
void SawTooth_Data( u16 cycle ,u16 *D)
{
	u16 i;
	for( i=0;i<cycle;i++)
	{
		D[i]= (u16)(1.0*1.0*i/255*4095/3.3);
	}
}
/********生成方波形输出表***********/
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
/********生成三角波形输出表***********/
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
/******************正弦波形表***********************/
#ifdef  Sine_WaveOutput_Enable 	
     u16 SineWave_Value[256];		//已用函数代替
#endif
/******************锯齿波形表***********************/
#ifdef  SawTooth_WaveOutput_Enable
     u16 SawToothWave_Value[256];  //已用函数代替
#endif	
/******************方波波形表***********************/
#ifdef  Square_WaveOutput_Enable 	
     u16 SquareWave_Value[256];		//已用函数代替
#endif	
/******************三角波波形表***********************/
#ifdef  Triangle_WaveOutput_Enable 	
     u16 TriangleWave_Value[256];		//已用函数代替
#endif	

/******DAC寄存器地址声明*******/	
#define DAC_DHR12R1    (u32)&(DAC->DHR12R1)   //DAC通道1输出寄存器地址



/****************引脚初始化******************/
void SineWave_GPIO_Config()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);   //开时钟
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;          //普通输出模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			//输出速率	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;       //推挽输出
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 ; //选择引脚
	GPIO_SetBits(GPIOA,GPIO_Pin_4)	;	//拉高输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);		//初始化
}

/******************DAC初始化*************************/
void SineWave_DAC_Config()
{
	DAC_InitTypeDef            DAC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);//开DAC时钟
	
  /**************DAC结构初始化，很重要，否则无波形*******************/
	DAC_StructInit(&DAC_InitStructure);		
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;//不产生波形
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable; //不使能输出缓存
	
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_T4_TRGO;//选择DAC触发源为TIM2
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);//初始化
	DAC_Cmd(DAC_Channel_1, ENABLE);	   //使能DAC通道1
	DAC_SetChannel1Data(DAC_Align_12b_R, 0);  //12位右对齐数据格式设置DAC值
	DAC_DMACmd(DAC_Channel_1, ENABLE); //使能DAC通道1的DMA
}
		 
/*********定时器配置************/
void SineWave_TIM_Config( u32 Wave1_Fre )
{
	TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//开时钟
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
 	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;     //不预分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0; //不分频	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数模式
	TIM_TimeBaseStructure.TIM_Period = Wave1_Fre;    
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_SelectOutputTrigger(TIM4, TIM_TRGOSource_Update);//设置TIM2输出触发为更新模式
}

/*********DMA配置***********/
void SineWave_DMA_Config( u16 *Wave1_Mem )
{					
	DMA_InitTypeDef            DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);//开DMA1的时钟
	
	DMA_StructInit( &DMA_InitStructure);		//DMA结构初始化
	DMA_InitStructure.DMA_Channel=DMA_Channel_7;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//从存储器读数据
	DMA_InitStructure.DMA_BufferSize = 256;//缓存大小，一般为256点
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不递增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//宽度为半字
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//宽度为半字
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//优先级：非常高
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//循环发送模式 
	DMA_InitStructure.DMA_PeripheralBaseAddr = DAC_DHR12R1;//外设地址为DAC通道1数据寄存器
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Wave1_Mem;//内存地址为输出波形数据数组
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);//初始化
	DMA_Cmd(DMA1_Stream5, ENABLE);  //使能DMA通道3       
}

/***********正弦波初始化***************/
u16 *add1;
u16 f1;
void SineWave_Init(u8 Wave1)
{
	SineWave_Data( N ,SineWave_Value);		//生成波形表1
	SawTooth_Data( N ,SawToothWave_Value);//生成波形表2
	Square_Data( N ,SquareWave_Value);		//生成波形表3
	Triangle_Data( N ,TriangleWave_Value);//生成波形表4
	if( Wave1==0x00)   add1=SineWave_Value;
	if( Wave1==0x01)	 add1=SawToothWave_Value;
	if( Wave1==0x02)	 add1=SquareWave_Value;
	if( Wave1==0x03)	 add1=TriangleWave_Value;
	f1=(u16)(84000000/512*2/1000);
	SineWave_GPIO_Config();			  //初始化引脚 
	SineWave_TIM_Config( f1 );			  //初始化定时器 
	SineWave_DAC_Config();			  //初始化DAC
	SineWave_DMA_Config( add1 );			  //初始化DMA
  TIM_Cmd(TIM4, ENABLE);			 //使能TIM2,开始产生波形  
}

void SineWave(u8 Wave1)
{
	if( Wave1==0x00)   add1=SineWave_Value;
	if( Wave1==0x01)	 add1=SawToothWave_Value;
	if( Wave1==0x02)	 add1=SquareWave_Value;
	if( Wave1==0x03)	 add1=TriangleWave_Value;
	DMA_Cmd(DMA1_Stream5,DISABLE);
	SineWave_DMA_Config( add1 );			  //初始化DMA
	DMA_Cmd(DMA1_Stream5, ENABLE);  //使能DMA通道3       

}


void Set_WaveFre(u16 fre)
{
	u16 reload;	
	reload=(u16)(84000000/512*2/fre);
	TIM_SetAutoreload( TIM4 ,reload);
}


