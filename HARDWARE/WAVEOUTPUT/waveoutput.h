#ifndef _WAVEOUTPUT_H
#define _WAVEOUTPUT_H
#include "sys.h"
#include "math.h"

#define PI  3.1415926
#define Vref 3.3		//0.1~3.3V�ɵ�
#define Um  1.0
#define N 256

/********����Ҫ�Ĳ���ע�͵�����**********/
#define  Sine_WaveOutput_Enable
#define  SawTooth_WaveOutput_Enable
#define  Square_WaveOutput_Enable
#define  Triangle_WaveOutput_Enable

#define  SinWave   			0x00	//���Ҳ�
#define  SawToothWave   0x01	//��ݲ�
#define  SquareWave   			0x02	//����
#define  TriangleWave   			0x03	//���ǲ�
#define  Wave_Channel_1   0x00	//ͨ��1
#define  Wave_Channel_2   0x01	//ͨ��2
#define  Wave_Channel_3   0x03	//ͨ��3
#define  Wave_Channel_4   0x04	//ͨ��4

/***********���Ҳ���ʼ��***************/
//Wave1������1ѡ��
//Wave1_Fre������1Ƶ��
//NewState1������1ʹ��
//Wave2������2ѡ��
//Wave2_Fre������2Ƶ��
//NewState2������2ʹ��
void SineWave_Init(u8 Wave1);
/************��̬���ò���Ƶ��*************/
//Wave_Channel��ͨ��ѡ��
//fre��Ƶ��
void Set_WaveFre( u16 fre);
void SineWave(u8 Wave1);

void SineWave_DMA_Config( u16 *Wave1_Mem );


#endif


