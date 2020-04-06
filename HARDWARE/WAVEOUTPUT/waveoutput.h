#ifndef _WAVEOUTPUT_H
#define _WAVEOUTPUT_H
#include "sys.h"
#include "math.h"

#define PI  3.1415926
#define Vref 3.3		//0.1~3.3V可调
#define Um  1.0
#define N 256

/********不需要的波形注释掉即可**********/
#define  Sine_WaveOutput_Enable
#define  SawTooth_WaveOutput_Enable
#define  Square_WaveOutput_Enable
#define  Triangle_WaveOutput_Enable

#define  SinWave   			0x00	//正弦波
#define  SawToothWave   0x01	//锯齿波
#define  SquareWave   			0x02	//方波
#define  TriangleWave   			0x03	//三角波
#define  Wave_Channel_1   0x00	//通道1
#define  Wave_Channel_2   0x01	//通道2
#define  Wave_Channel_3   0x03	//通道3
#define  Wave_Channel_4   0x04	//通道4

/***********正弦波初始化***************/
//Wave1：波形1选择
//Wave1_Fre：波形1频率
//NewState1：波形1使能
//Wave2：波形2选择
//Wave2_Fre：波形2频率
//NewState2：波形2使能
void SineWave_Init(u8 Wave1);
/************动态设置波形频率*************/
//Wave_Channel：通道选择
//fre：频率
void Set_WaveFre( u16 fre);
void SineWave(u8 Wave1);

void SineWave_DMA_Config( u16 *Wave1_Mem );


#endif


