#include "sdio_sdcard.h"
#include "string.h"	 
#include "sys.h"	 
#include "usart.h"	 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//SDIO ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/14
//�汾��V1.1
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved		
//V1.1 
//1,���볬ʱ�ж�,�����ѯ��������������.
////////////////////////////////////////////////////////////////////////////////// 	 


/*����sdio��ʼ���Ľṹ��*/
SDIO_InitTypeDef SDIO_InitStructure;
SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
SDIO_DataInitTypeDef SDIO_DataInitStructure;   


static u8 CardType=SDIO_STD_CAPACITY_SD_CARD_V1_1;		//SD�����ͣ�Ĭ��Ϊ1.x����
static u32 CSD_Tab[4],CID_Tab[4],RCA=0;					//SD��CSD,CID�Լ���Ե�ַ(RCA)����
static u8 DeviceMode=SD_DMA_MODE;		   				//����ģʽ,ע��,����ģʽ����ͨ��SD_SetDeviceMode,�������.����ֻ�Ƕ���һ��Ĭ�ϵ�ģʽ(SD_DMA_MODE)
static u8 StopCondition=0; 								//�Ƿ���ֹͣ�����־λ,DMA����д��ʱ���õ�  
volatile SD_Error TransferError=SD_OK;					//���ݴ�������־,DMA��дʱʹ��	    
volatile u8 TransferEnd=0;								//���������־,DMA��дʱʹ��
SD_CardInfo SDCardInfo;									//SD����Ϣ


static SD_Error CmdError(void);
static SD_Error CmdResp1Error(uint8_t cmd);
static SD_Error CmdResp7Error(void);
static SD_Error CmdResp3Error(void);
static SD_Error CmdResp2Error(void);
static SD_Error CmdResp6Error(uint8_t cmd, uint16_t *prca);
SD_Error SDEnWideBus(FunctionalState NewState);

SD_Error IsCardProgramming(u8 *pstatus); 
SD_Error FindSCR(u16 rca,u32 *pscr);

u8 convert_from_bytes_to_power_of_two(u16 NumberOfBytes); 


//SD_ReadDisk/SD_WriteDisk����ר��buf,�����������������ݻ�������ַ����4�ֽڶ����ʱ��,
//��Ҫ�õ�������,ȷ�����ݻ�������ַ��4�ֽڶ����.
__align(4) u8 SDIO_DATA_BUFFER[512];						  
 
 
void SD_Register_DeInit(void) 
{
  SDIO->POWER = 0x00000000;
  SDIO->CLKCR = 0x00000000;
  SDIO->ARG = 0x00000000;
  SDIO->CMD = 0x00000000;
  SDIO->DTIMER = 0x00000000;
  SDIO->DLEN = 0x00000000;
  SDIO->DCTRL = 0x00000000;
  SDIO->ICR = 0x00C007FF;
  SDIO->MASK = 0x00000000;
}

//��ʼ��SD��
//����ֵ:�������;(0,�޴���)
SD_Error SD_Init(void)
{
	SD_Error errorstatus=SD_OK;	   
	//SDIO IO�ڳ�ʼ��   	 
 
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_DMA2, ENABLE);//ʹ��GPIOC,GPIOD DMA2ʱ��
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDIO, ENABLE);//SDIOʱ��ʹ��
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SDIO, ENABLE);//SDIO��λ
	
	
	GPIO_InitStructure.GPIO_Pin =0X3F<<8; 	//PC8,9,10,11,12���ù������	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOC, &GPIO_InitStructure);// PC8,9,10,11,12���ù������

	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_2;
	GPIO_Init(GPIOD, &GPIO_InitStructure);//PD2���ù������
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_SDIO); //PC8,AF12
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_SDIO);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_SDIO);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_SDIO);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_SDIO);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_SDIO);		

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SDIO, DISABLE);//SDIO������λ
	
 	//SDIO����Ĵ�������ΪĬ��ֵ 			   
	SD_Register_DeInit();
	SDIO_DeInit();
 
	NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

	errorstatus=SD_PowerON();			//SD���ϵ�
 	
	if(errorstatus==SD_OK)errorstatus=SD_InitializeCards();			//��ʼ��SD��														  
	if(errorstatus==SD_OK)errorstatus=SD_GetCardInfo(&SDCardInfo);	//��ȡ����Ϣ
 	if(errorstatus==SD_OK)errorstatus=SD_SelectDeselect((u32)(SDCardInfo.RCA<<16));//ѡ��SD��   
	if(errorstatus==SD_OK)errorstatus=SD_EnableWideBusOperation(SDIO_BusWide_4b);	//4λ���,�����MMC��,������4λģʽ 
	if((errorstatus==SD_OK)||(SDIO_MULTIMEDIA_CARD==CardType))
	{  		    
		SDIO_Clock_Set(SDIO_TRANSFER_CLK_DIV);			//����ʱ��Ƶ��,SDIOʱ�Ӽ��㹫ʽ:SDIO_CKʱ��=SDIOCLK/[clkdiv+2];����,SDIOCLK�̶�Ϊ48Mhz 
		//errorstatus=SD_SetDeviceMode(SD_DMA_MODE);//����ΪDMAģʽ
		errorstatus=SD_SetDeviceMode(SD_POLLING_MODE);//����Ϊ��ѯģʽ
 	}
	return errorstatus;		 
}
//SDIOʱ�ӳ�ʼ������
//clkdiv:ʱ�ӷ�Ƶϵ��
//CKʱ��=SDIOCLK/[clkdiv+2];(SDIOCLKʱ�ӹ̶�Ϊ48Mhz)
void SDIO_Clock_Set(u8 clkdiv)
{
	u32 tmpreg=SDIO->CLKCR; 
  	tmpreg&=0XFFFFFF00; 
 	tmpreg|=clkdiv; 
	SDIO->CLKCR=tmpreg;
} 

//���ϵ�
//��ѯ����SDIO�ӿ��ϵĿ��豸,����ѯ���ѹ������ʱ��
//����ֵ:�������;(0,�޴���)
SD_Error SD_PowerON(void)
{
	
  SD_Error errorstatus = SD_OK;
  uint32_t response = 0, count = 0, validvoltage = 0;
  uint32_t SDType = SD_STD_CAPACITY;

  //��ʼ��ʱ��ʱ�Ӳ��ܴ���400KHz 
  SDIO_InitStructure.SDIO_ClockDiv = SDIO_INIT_CLK_DIV;	//SDIOCLK = 48MHz, SDIO_CK = SDIOCLK/(118 + 2) = 400 KHz 
  SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
  SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;  //��ʹ��bypassģʽ��ֱSDIOCLK���з�Ƶ�õ�SDIO_CK
  SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;	// ����ʱ���ر�ʱ�ӵ�Դ
  SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;	 				//1λ������
  SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;//�ر�Ӳ������
  SDIO_Init(&SDIO_InitStructure);

  SDIO_SetPowerState(SDIO_PowerState_ON); //��SDIO��Դ

  SDIO_ClockCmd(ENABLE);  //����SDIOʱ��
   
  //���淢��һϵ������,��ʼ��ʶ������
  SDIO_CmdInitStructure.SDIO_Argument = 0x0;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_GO_IDLE_STATE; //cmd0
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_No;  //����Ӧ
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;  //��CPSM�ڿ�ʼ��������֮ǰ�ȴ����ݴ�������� 
  SDIO_SendCommand(&SDIO_CmdInitStructure);	  		//д���������Ĵ���

  errorstatus = CmdError();//����Ƿ���ȷ���յ�cmd0

  if (errorstatus != SD_OK)	//����ͳ�������
  {
    /*!< CMD Response TimeOut (wait for CMDSENT flag) */
    return(errorstatus);
  }

  /*!< CMD8: SEND_IF_COND ----------------------------------------------------*/
  /*!< Send CMD8 to verify SD card interface operating condition */
  /*!< Argument: - [31:12]: Reserved (shall be set to '0')
               - [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
               - [7:0]: Check Pattern (recommended 0xAA) */
  /*!< CMD Response: R7 */
  SDIO_CmdInitStructure.SDIO_Argument = SD_CHECK_PATTERN;	//���յ�����sd�᷵���������
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SEND_IF_COND;	//cmd8
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;	 //r7
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;			 //�رյȴ��ж�
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);
   
  /*����Ƿ���յ�����*/
  errorstatus = CmdResp7Error(); 
	
  if (errorstatus == SD_OK)	  	//����Ӧ��card��ѭsdЭ��2.0�汾
  {
    CardType = SDIO_STD_CAPACITY_SD_CARD_V2_0; /*!< SD Card 2.0 ���Ȱ��������sdsc���͵Ŀ�*/
    SDType = SD_HIGH_CAPACITY;	//�����������acmd41�Ĳ���������ѯ����sdsc������sdhc��
  }
  else	//����Ӧ��˵����1.x�Ļ�mmc�Ŀ�
  {
    /*!< CMD55 */	  
    SDIO_CmdInitStructure.SDIO_Argument = 0x00;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);
    errorstatus = CmdResp1Error(SD_CMD_APP_CMD);
  }
  /*!< CMD55 */		//Ϊʲô��else���else���涼Ҫ����CMD55? 
  //����cmd55�����ڼ����sd������mmc�������ǲ�֧�ֵĿ�  
  SDIO_CmdInitStructure.SDIO_Argument = 0x00;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short; //r1
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);
  errorstatus = CmdResp1Error(SD_CMD_APP_CMD);	//�Ƿ���Ӧ��û��Ӧ����mmc��֧�ֵĿ�


  if (errorstatus == SD_OK)	//��Ӧ��cmd55����sd��������Ϊ1.x,����Ϊ2.0
  {
  	//���濪ʼѭ���ط���sdio֧�ֵĵ�ѹ��Χ��ѭ��һ������

	 //����ACMD42 ����Ϊ0X80100000
    while ((!validvoltage) && (count < SD_MAX_VOLT_TRIAL))
    {	 
	  //��Ϊ����Ҫ�õ�ACMD41����ACMD����ڷ���ACMD����ǰ��Ҫ���򿨷���CMD55
      SDIO_CmdInitStructure.SDIO_Argument = 0x00;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;	  //CMD55
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SD_CMD_APP_CMD); //�����Ӧ

      if (errorstatus != SD_OK)
      {
        return(errorstatus);//û��ӦCMD55������  
      }
	  //acmd41�����������֧�ֵĵ�ѹ��Χ��HCSλ��ɣ�HCSλ��һ�����ֿ���SDSc����sdhc
      SDIO_CmdInitStructure.SDIO_Argument = SD_VOLTAGE_WINDOW_SD | SDType;	  //����Ϊ�����ɹ���ѹ��Χ��hcsλ
      SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SD_APP_OP_COND;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;  //r3
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp3Error();	//����Ƿ���ȷ���յ�����
      if (errorstatus != SD_OK)
      {
        return(errorstatus);  //û��ȷ���յ�acmd41����������
      }
	   //���������ѹ��SDIO�Ĺ����ѹ��Χ�ڣ����Զ��ϵ粢��־pwr_upλ
      response = SDIO_GetResponse(SDIO_RESP1);	 //��ȡ���Ĵ�������״̬
      validvoltage = (((response >> 31) == 1) ? 1 : 0);	//��ȡ����ocr�Ĵ�����pwr_upλ�����Ƿ��ѹ�����������ѹ
      count++;			  //����ѭ������
    }
    if (count >= SD_MAX_VOLT_TRIAL)	//ѭ����ⳬ��һ��������û�ϵ�
    {
      errorstatus = SD_INVALID_VOLTRANGE;	   //SDIO��֧��card�Ĺ����ѹ
      return(errorstatus);
    }
			 //��鿨������Ϣ�е�HCSλ
    if (response &= SD_HIGH_CAPACITY)  //�ж�ocr�е�ccsλ �������sdsc����ִ����������
    {
      CardType = SDIO_HIGH_CAPACITY_SD_CARD;  //�ѿ����ʹӳ�ʼ����sdsc�͸�Ϊsdhc��
    }

  }/*!< else MMC Card */

  return(errorstatus);		
}

//SD�� Power OFF
//����ֵ:�������;(0,�޴���)
SD_Error SD_PowerOFF(void)
{
  SD_Error errorstatus = SD_OK;

  //�ر�SD����
  SDIO_SetPowerState(SDIO_PowerState_OFF);

  return(errorstatus);
}
 
//��ʼ�����еĿ�,���ÿ��������״̬
//����ֵ:�������
SD_Error SD_InitializeCards(void)
{
  SD_Error errorstatus = SD_OK;
  uint16_t rca = 0x01;

  if (SDIO_GetPowerState() == SDIO_PowerState_OFF)  
  {
    errorstatus = SD_REQUEST_NOT_APPLICABLE;
    return(errorstatus);
  }
	
  if (SDIO_SECURE_DIGITAL_IO_CARD != CardType)//�жϿ�������
  {
	 //����CMD2���� ��ȡ��CID��Ϣ
    SDIO_CmdInitStructure.SDIO_Argument = 0x0;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_ALL_SEND_CID;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Long;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp2Error();

    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }

    CID_Tab[0] = SDIO_GetResponse(SDIO_RESP1);
    CID_Tab[1] = SDIO_GetResponse(SDIO_RESP2);
    CID_Tab[2] = SDIO_GetResponse(SDIO_RESP3);
    CID_Tab[3] = SDIO_GetResponse(SDIO_RESP4);
  }

     /*���濪ʼSD����ʼ������*/
  if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) ||  (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) ||  (SDIO_SECURE_DIGITAL_IO_COMBO_CARD == CardType)
      ||  (SDIO_HIGH_CAPACITY_SD_CARD == CardType))	 //ʹ�õ���2.0�Ŀ�
  {
	 //����CMD3���� ��ȡSD����Ե�ַ
    SDIO_CmdInitStructure.SDIO_Argument = 0x00;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_REL_ADDR;	//cmd3
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short; //r6
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp6Error(SD_CMD_SET_REL_ADDR, &rca);	//�ѽ��յ��Ŀ���Ե�ַ��������

    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }
  }

  if (SDIO_SECURE_DIGITAL_IO_CARD != CardType)
  {
    RCA = rca;

    /*!< Send CMD9 SEND_CSD with argument as card's RCA */
	 //����CMD9  ��ȡSD�������Ϣ
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)(rca << 16);
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_CSD;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Long;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp2Error();

    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }

    CSD_Tab[0] = SDIO_GetResponse(SDIO_RESP1);
    CSD_Tab[1] = SDIO_GetResponse(SDIO_RESP2);
    CSD_Tab[2] = SDIO_GetResponse(SDIO_RESP3);
    CSD_Tab[3] = SDIO_GetResponse(SDIO_RESP4);
  }

  errorstatus = SD_OK; /*!< All cards get intialized */

  return(errorstatus);
} 
//�õ�����Ϣ
//cardinfo:����Ϣ�洢��
//����ֵ:����״̬
SD_Error SD_GetCardInfo(SD_CardInfo *cardinfo)
{
 	SD_Error errorstatus=SD_OK;
	u8 tmp=0;	   
	cardinfo->CardType=(u8)CardType; 				//������
	cardinfo->RCA=(u16)RCA;							//��RCAֵ
	tmp=(u8)((CSD_Tab[0]&0xFF000000)>>24);
	cardinfo->SD_csd.CSDStruct=(tmp&0xC0)>>6;		//CSD�ṹ
	cardinfo->SD_csd.SysSpecVersion=(tmp&0x3C)>>2;	//2.0Э�黹û�����ⲿ��(Ϊ����),Ӧ���Ǻ���Э�鶨���
	cardinfo->SD_csd.Reserved1=tmp&0x03;			//2������λ 
	
	tmp=(u8)((CSD_Tab[0]&0x00FF0000)>>16);			//��1���ֽ�
	cardinfo->SD_csd.TAAC=tmp;				   		//���ݶ�ʱ��1
	
	tmp=(u8)((CSD_Tab[0]&0x0000FF00)>>8);	  		//��2���ֽ�
	cardinfo->SD_csd.NSAC=tmp;		  				//���ݶ�ʱ��2
	
	tmp=(u8)(CSD_Tab[0]&0x000000FF);				//��3���ֽ�
	cardinfo->SD_csd.MaxBusClkFrec=tmp;		  		//�����ٶ�	 
  
	tmp=(u8)((CSD_Tab[1]&0xFF000000)>>24);			//��4���ֽ�
	cardinfo->SD_csd.CardComdClasses=tmp<<4;    	//��ָ�������λ
	
	tmp=(u8)((CSD_Tab[1]&0x00FF0000)>>16);	 		//��5���ֽ�
	cardinfo->SD_csd.CardComdClasses|=(tmp&0xF0)>>4;//��ָ�������λ
	cardinfo->SD_csd.RdBlockLen=tmp&0x0F;	    	//����ȡ���ݳ���
	
	tmp=(u8)((CSD_Tab[1]&0x0000FF00)>>8);			//��6���ֽ�
	cardinfo->SD_csd.PartBlockRead=(tmp&0x80)>>7;	//����ֿ��
	cardinfo->SD_csd.WrBlockMisalign=(tmp&0x40)>>6;	//д���λ
	cardinfo->SD_csd.RdBlockMisalign=(tmp&0x20)>>5;	//�����λ
	cardinfo->SD_csd.DSRImpl=(tmp&0x10)>>4;
	cardinfo->SD_csd.Reserved2=0; 					//����
	
	
 	if((CardType==SDIO_STD_CAPACITY_SD_CARD_V1_1)||(CardType==SDIO_STD_CAPACITY_SD_CARD_V2_0)||(SDIO_MULTIMEDIA_CARD==CardType))//��׼1.1/2.0��/MMC��
	{
		cardinfo->SD_csd.DeviceSize=(tmp&0x03)<<10;	//C_SIZE(12λ)
	 	tmp=(u8)(CSD_Tab[1]&0x000000FF); 			//��7���ֽ�	
		cardinfo->SD_csd.DeviceSize|=(tmp)<<2;
		
 		tmp=(u8)((CSD_Tab[2]&0xFF000000)>>24);		//��8���ֽ�	
		cardinfo->SD_csd.DeviceSize|=(tmp&0xC0)>>6;
 		cardinfo->SD_csd.MaxRdCurrentVDDMin=(tmp&0x38)>>3;
		cardinfo->SD_csd.MaxRdCurrentVDDMax=(tmp&0x07);
		
 		tmp=(u8)((CSD_Tab[2]&0x00FF0000)>>16);		//��9���ֽ�	
		cardinfo->SD_csd.MaxWrCurrentVDDMin=(tmp&0xE0)>>5;
		cardinfo->SD_csd.MaxWrCurrentVDDMax=(tmp&0x1C)>>2;
		cardinfo->SD_csd.DeviceSizeMul=(tmp&0x03)<<1;//C_SIZE_MULT
		
 		tmp=(u8)((CSD_Tab[2]&0x0000FF00)>>8);	  	//��10���ֽ�	
		cardinfo->SD_csd.DeviceSizeMul|=(tmp&0x80)>>7;
 		cardinfo->CardCapacity=(cardinfo->SD_csd.DeviceSize+1);//���㿨����------------------------------------
		cardinfo->CardCapacity*=(1<<(cardinfo->SD_csd.DeviceSizeMul+2));
		cardinfo->CardBlockSize=1<<(cardinfo->SD_csd.RdBlockLen);//���С
		cardinfo->CardCapacity*=cardinfo->CardBlockSize;
	}else if(CardType==SDIO_HIGH_CAPACITY_SD_CARD)	//��������
	{
 		tmp=(u8)(CSD_Tab[1]&0x000000FF); 		//��7���ֽ�	
		cardinfo->SD_csd.DeviceSize=(tmp&0x3F)<<16;//C_SIZE
		
 		tmp=(u8)((CSD_Tab[2]&0xFF000000)>>24); 	//��8���ֽ�	
 		cardinfo->SD_csd.DeviceSize|=(tmp<<8);
		
 		tmp=(u8)((CSD_Tab[2]&0x00FF0000)>>16);	//��9���ֽ�	
 		cardinfo->SD_csd.DeviceSize|=(tmp);
		
 		tmp=(u8)((CSD_Tab[2]&0x0000FF00)>>8); 	//��10���ֽ�	
 		cardinfo->CardCapacity=(long long)(cardinfo->SD_csd.DeviceSize+1)*512*1024;//���㿨����
		cardinfo->CardBlockSize=512; 			//���С�̶�Ϊ512�ֽ�
	}	  
	cardinfo->SD_csd.EraseGrSize=(tmp&0x40)>>6;
	cardinfo->SD_csd.EraseGrMul=(tmp&0x3F)<<1;	   
	tmp=(u8)(CSD_Tab[2]&0x000000FF);			//��11���ֽ�	
	
	
	cardinfo->SD_csd.EraseGrMul|=(tmp&0x80)>>7;
	cardinfo->SD_csd.WrProtectGrSize=(tmp&0x7F);
 	tmp=(u8)((CSD_Tab[3]&0xFF000000)>>24);		//��12���ֽ�	
	
	
	cardinfo->SD_csd.WrProtectGrEnable=(tmp&0x80)>>7;
	cardinfo->SD_csd.ManDeflECC=(tmp&0x60)>>5;
	cardinfo->SD_csd.WrSpeedFact=(tmp&0x1C)>>2;
	cardinfo->SD_csd.MaxWrBlockLen=(tmp&0x03)<<2;	 
	tmp=(u8)((CSD_Tab[3]&0x00FF0000)>>16);		//��13���ֽ�
	
	
	cardinfo->SD_csd.MaxWrBlockLen|=(tmp&0xC0)>>6;
	cardinfo->SD_csd.WriteBlockPaPartial=(tmp&0x20)>>5;
	cardinfo->SD_csd.Reserved3=0;
	cardinfo->SD_csd.ContentProtectAppli=(tmp&0x01);  
	
	
	tmp=(u8)((CSD_Tab[3]&0x0000FF00)>>8);		//��14���ֽ�
	cardinfo->SD_csd.FileFormatGrouop=(tmp&0x80)>>7;
	cardinfo->SD_csd.CopyFlag=(tmp&0x40)>>6;
	cardinfo->SD_csd.PermWrProtect=(tmp&0x20)>>5;
	cardinfo->SD_csd.TempWrProtect=(tmp&0x10)>>4;
	cardinfo->SD_csd.FileFormat=(tmp&0x0C)>>2;
	cardinfo->SD_csd.ECC=(tmp&0x03);  
	
	
	tmp=(u8)(CSD_Tab[3]&0x000000FF);			//��15���ֽ�
	cardinfo->SD_csd.CSD_CRC=(tmp&0xFE)>>1;
	cardinfo->SD_csd.Reserved4=1;		 
	tmp=(u8)((CID_Tab[0]&0xFF000000)>>24);		//��0���ֽ�
	cardinfo->SD_cid.ManufacturerID=tmp;		    
	tmp=(u8)((CID_Tab[0]&0x00FF0000)>>16);		//��1���ֽ�
	cardinfo->SD_cid.OEM_AppliID=tmp<<8;	  
	tmp=(u8)((CID_Tab[0]&0x000000FF00)>>8);		//��2���ֽ�
	cardinfo->SD_cid.OEM_AppliID|=tmp;	    
	tmp=(u8)(CID_Tab[0]&0x000000FF);			//��3���ֽ�	
	cardinfo->SD_cid.ProdName1=tmp<<24;				  
	tmp=(u8)((CID_Tab[1]&0xFF000000)>>24); 		//��4���ֽ�
	cardinfo->SD_cid.ProdName1|=tmp<<16;	  
	tmp=(u8)((CID_Tab[1]&0x00FF0000)>>16);	   	//��5���ֽ�
	cardinfo->SD_cid.ProdName1|=tmp<<8;		 
	tmp=(u8)((CID_Tab[1]&0x0000FF00)>>8);		//��6���ֽ�
	cardinfo->SD_cid.ProdName1|=tmp;		   
	tmp=(u8)(CID_Tab[1]&0x000000FF);	  		//��7���ֽ�
	cardinfo->SD_cid.ProdName2=tmp;			  
	tmp=(u8)((CID_Tab[2]&0xFF000000)>>24); 		//��8���ֽ�
	cardinfo->SD_cid.ProdRev=tmp;		 
	tmp=(u8)((CID_Tab[2]&0x00FF0000)>>16);		//��9���ֽ�
	cardinfo->SD_cid.ProdSN=tmp<<24;	   
	tmp=(u8)((CID_Tab[2]&0x0000FF00)>>8); 		//��10���ֽ�
	cardinfo->SD_cid.ProdSN|=tmp<<16;	   
	tmp=(u8)(CID_Tab[2]&0x000000FF);   			//��11���ֽ�
	cardinfo->SD_cid.ProdSN|=tmp<<8;		   
	tmp=(u8)((CID_Tab[3]&0xFF000000)>>24); 		//��12���ֽ�
	cardinfo->SD_cid.ProdSN|=tmp;			     
	tmp=(u8)((CID_Tab[3]&0x00FF0000)>>16);	 	//��13���ֽ�
	cardinfo->SD_cid.Reserved1|=(tmp&0xF0)>>4;
	cardinfo->SD_cid.ManufactDate=(tmp&0x0F)<<8;    
	tmp=(u8)((CID_Tab[3]&0x0000FF00)>>8);		//��14���ֽ�
	cardinfo->SD_cid.ManufactDate|=tmp;		 	  
	tmp=(u8)(CID_Tab[3]&0x000000FF);			//��15���ֽ�
	cardinfo->SD_cid.CID_CRC=(tmp&0xFE)>>1;
	cardinfo->SD_cid.Reserved2=1;	 
	return errorstatus;
}

//����SDIO���߿��(MMC����֧��4bitģʽ)
//   @arg SDIO_BusWide_8b: 8-bit data transfer (Only for MMC)
//   @arg SDIO_BusWide_4b: 4-bit data transfer
//   @arg SDIO_BusWide_1b: 1-bit data transfer (Ĭ��)
//����ֵ:SD������״̬
SD_Error SD_EnableWideBusOperation(uint32_t WideMode)
{
  SD_Error errorstatus = SD_OK;

	//MMC����֧��
  if (SDIO_MULTIMEDIA_CARD == CardType)
  {
    errorstatus = SD_UNSUPPORTED_FEATURE;
    return(errorstatus);
  }
  else if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) || (SDIO_HIGH_CAPACITY_SD_CARD == CardType))
  {														 
    if (SDIO_BusWide_8b == WideMode)   //2.0 sd��֧��8bits
    {
      errorstatus = SD_UNSUPPORTED_FEATURE;
      return(errorstatus);
    }
    else if (SDIO_BusWide_4b == WideMode)//4������ģʽ
    {
      errorstatus = SDEnWideBus(ENABLE);//ʹ��acmd6�������߿�ȣ����ÿ��Ĵ��䷽ʽ

      if (SD_OK == errorstatus)
      {
        /*!< Configure the SDIO peripheral */
        SDIO_InitStructure.SDIO_ClockDiv = SDIO_TRANSFER_CLK_DIV; 
        SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
        SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
        SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
        SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_4b;	//���������stm32��sdio�Ĵ��䷽ʽ ���л�ģʽ����ӿ���sdio����Ӧ��
        SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
        SDIO_Init(&SDIO_InitStructure);
      }
    }
    else//��������ģʽ
    {
      errorstatus = SDEnWideBus(DISABLE);

      if (SD_OK == errorstatus)
      {
        /*!< Configure the SDIO peripheral */
        SDIO_InitStructure.SDIO_ClockDiv = SDIO_TRANSFER_CLK_DIV; 
        SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
        SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
        SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
        SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;
        SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
        SDIO_Init(&SDIO_InitStructure);
      }
    }
  }

  return(errorstatus);
}

//����SD������ģʽ
//Mode:
//����ֵ:����״̬
SD_Error SD_SetDeviceMode(u32 Mode)
{
	SD_Error errorstatus = SD_OK;
 	if((Mode==SD_DMA_MODE)||(Mode==SD_POLLING_MODE))DeviceMode=Mode;
	else errorstatus=SD_INVALID_PARAMETER;
	return errorstatus;	    
}
//ѡ��
//����CMD7,ѡ����Ե�ַ(rca)Ϊaddr�Ŀ�,ȡ��������.���Ϊ0,�򶼲�ѡ��.
//addr:����RCA��ַ
SD_Error SD_SelectDeselect(u32 addr)
{
  SD_Error errorstatus = SD_OK;

  /*!< Send CMD7 SDIO_SEL_DESEL_CARD */
  SDIO_CmdInitStructure.SDIO_Argument =  addr;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEL_DESEL_CARD;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_SEL_DESEL_CARD);

  return(errorstatus);
}
//SD����ȡһ���� 
//buf:�����ݻ�����(����4�ֽڶ���!!)
//addr:��ȡ��ַ
//blksize:���С
SD_Error SD_ReadBlock(u8 *buf,long long addr,u16 blksize)
{	  
	SD_Error errorstatus=SD_OK;
	u8 power;
   	u32 count=0,*tempbuff=(u32*)buf;//ת��Ϊu32ָ�� 
	u32 timeout=SDIO_DATATIMEOUT;   
   	if(NULL==buf)return SD_INVALID_PARAMETER; 
   	SDIO->DCTRL=0x0;	//���ݿ��ƼĴ�������(��DMA)   
	if(CardType==SDIO_HIGH_CAPACITY_SD_CARD)//��������
	{
		blksize=512;
		addr>>=9;
	}   
 
	SDIO_DataInitStructure.SDIO_DataBlockSize= 0 ;//���DPSM״̬������
	SDIO_DataInitStructure.SDIO_DataLength= 0 ;
	SDIO_DataInitStructure.SDIO_DataTimeOut=SD_DATATIMEOUT ;
	SDIO_DataInitStructure.SDIO_DPSM=SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir=SDIO_TransferDir_ToCard;
	SDIO_DataInitStructure.SDIO_TransferMode=SDIO_TransferMode_Block;
	SDIO_DataConfig(&SDIO_DataInitStructure);

	
	if(SDIO->RESP1&SD_CARD_LOCKED)return SD_LOCK_UNLOCK_FAILED;//������
	if((blksize>0)&&(blksize<=2048)&&((blksize&(blksize-1))==0))
	{
		power=convert_from_bytes_to_power_of_two(blksize);	    	   

		SDIO_CmdInitStructure.SDIO_Argument =  blksize;//����CMD16+�������ݳ���Ϊblksize,����Ӧ 
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);
		
		errorstatus=CmdResp1Error(SD_CMD_SET_BLOCKLEN);	//�ȴ�R1��Ӧ   
		if(errorstatus!=SD_OK)return errorstatus;   	//��Ӧ����	 
	}else return SD_INVALID_PARAMETER;	  	  									    
   
	SDIO_DataInitStructure.SDIO_DataBlockSize= power<<4; ;//���DPSM״̬������
	SDIO_DataInitStructure.SDIO_DataLength= blksize ;
	SDIO_DataInitStructure.SDIO_DataTimeOut=SD_DATATIMEOUT ;
	SDIO_DataInitStructure.SDIO_DPSM=SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir=SDIO_TransferDir_ToSDIO;
	SDIO_DataInitStructure.SDIO_TransferMode=SDIO_TransferMode_Block;
	SDIO_DataConfig(&SDIO_DataInitStructure);
	
	
	SDIO_CmdInitStructure.SDIO_Argument =  addr;//����CMD17+��addr��ַ����ȡ����,����Ӧ 	
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_READ_SINGLE_BLOCK;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);
	
	errorstatus=CmdResp1Error(SD_CMD_READ_SINGLE_BLOCK);//�ȴ�R1��Ӧ   
	if(errorstatus!=SD_OK)return errorstatus;   		//��Ӧ����	 
 	if(DeviceMode==SD_POLLING_MODE)						//��ѯģʽ,��ѯ����	 
	{
 		INTX_DISABLE();//�ر����ж�(POLLINGģʽ,�Ͻ��жϴ��SDIO��д����!!!)
		while(!(SDIO->STA&((1<<5)|(1<<1)|(1<<3)|(1<<10)|(1<<9))))//������/CRC/��ʱ/���(��־)/��ʼλ����
		{
			if(SDIO->STA&(1<<15))						//����������,��ʾ���ٴ���8����
			{
				for(count=0;count<8;count++)			//ѭ����ȡ����
				{
					*(tempbuff+count)=SDIO->FIFO;
				}
				tempbuff+=8;	 
				timeout=0X7FFFF; 
			}else 	//����ʱ
			{
				if(timeout==0)return SD_DATA_TIMEOUT;
				timeout--;
			}
		} 
		if(SDIO->STA&(1<<3))		//���ݳ�ʱ����
		{										   
	 		SDIO->ICR|=1<<3; 		//������־
			return SD_DATA_TIMEOUT;
	 	}else if(SDIO->STA&(1<<1))	//���ݿ�CRC����
		{
	 		SDIO->ICR|=1<<1; 		//������־
			return SD_DATA_CRC_FAIL;		   
		}else if(SDIO->STA&(1<<5)) 	//����fifo�������
		{
	 		SDIO->ICR|=1<<5; 		//������־
			return SD_RX_OVERRUN;		 
		}else if(SDIO->STA&(1<<9)) 	//������ʼλ����
		{
	 		SDIO->ICR|=1<<9; 		//������־
			return SD_START_BIT_ERR;		 
		}   
		while(SDIO->STA&(1<<21))	//FIFO����,�����ڿ�������
		{
			*tempbuff=SDIO->FIFO;	//ѭ����ȡ����
			tempbuff++;
		}
		INTX_ENABLE();//�������ж�
		SDIO->ICR=0X5FF;	 		//������б��
	}else if(DeviceMode==SD_DMA_MODE)
	{
 		TransferError=SD_OK;
		StopCondition=0;			//�����,����Ҫ����ֹͣ����ָ��
		TransferEnd=0;				//�����������λ�����жϷ�����1
		SDIO->MASK|=(1<<1)|(1<<3)|(1<<8)|(1<<5)|(1<<9);	//������Ҫ���ж� 
	 	SDIO->DCTRL|=1<<3;		 	//SDIO DMAʹ�� 
 	    SD_DMA_Config((u32*)buf,blksize,DMA_DIR_PeripheralToMemory); 
 		while(((DMA2->LISR&(1<<27))==RESET)&&(TransferEnd==0)&&(TransferError==SD_OK)&&timeout)timeout--;//�ȴ�������� 
		if(timeout==0)return SD_DATA_TIMEOUT;//��ʱ
		if(TransferError!=SD_OK)errorstatus=TransferError;  
    }   
 	return errorstatus; 
}
//SD����ȡ����� 
//buf:�����ݻ�����
//addr:��ȡ��ַ
//blksize:���С
//nblks:Ҫ��ȡ�Ŀ���
//����ֵ:����״̬
__align(4) u32 *tempbuff;
SD_Error SD_ReadMultiBlocks(u8 *buf,long long addr,u16 blksize,u32 nblks)
{
  	SD_Error errorstatus=SD_OK;
	u8 power;
   	u32 count=0;
	u32 timeout=SDIO_DATATIMEOUT;  
	tempbuff=(u32*)buf;//ת��Ϊu32ָ��
	
    SDIO->DCTRL=0x0;		//���ݿ��ƼĴ�������(��DMA)   
	if(CardType==SDIO_HIGH_CAPACITY_SD_CARD)//��������
	{
		blksize=512;
		addr>>=9;
	}  

	SDIO_DataInitStructure.SDIO_DataBlockSize= 0; ;//���DPSM״̬������
	SDIO_DataInitStructure.SDIO_DataLength= 0 ;
	SDIO_DataInitStructure.SDIO_DataTimeOut=SD_DATATIMEOUT ;
	SDIO_DataInitStructure.SDIO_DPSM=SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir=SDIO_TransferDir_ToCard;
	SDIO_DataInitStructure.SDIO_TransferMode=SDIO_TransferMode_Block;
	SDIO_DataConfig(&SDIO_DataInitStructure);
	
	
	if(SDIO->RESP1&SD_CARD_LOCKED)return SD_LOCK_UNLOCK_FAILED;//������
	if((blksize>0)&&(blksize<=2048)&&((blksize&(blksize-1))==0))
	{
		power=convert_from_bytes_to_power_of_two(blksize);	    

		SDIO_CmdInitStructure.SDIO_Argument =  blksize;//����CMD16+�������ݳ���Ϊblksize,����Ӧ 
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);		
		
		errorstatus=CmdResp1Error(SD_CMD_SET_BLOCKLEN);	//�ȴ�R1��Ӧ   
		if(errorstatus!=SD_OK)return errorstatus;   	//��Ӧ����	 
	}else return SD_INVALID_PARAMETER;	  
	if(nblks>1)											//����  
	{									    
 	  	if(nblks*blksize>SD_MAX_DATA_LENGTH)return SD_INVALID_PARAMETER;//�ж��Ƿ񳬹������ճ���
	 
		SDIO_DataInitStructure.SDIO_DataBlockSize= power<<4; ;//nblks*blksize,512���С,����������
		SDIO_DataInitStructure.SDIO_DataLength= nblks*blksize ;
		SDIO_DataInitStructure.SDIO_DataTimeOut=SD_DATATIMEOUT ;
		SDIO_DataInitStructure.SDIO_DPSM=SDIO_DPSM_Enable;
		SDIO_DataInitStructure.SDIO_TransferDir=SDIO_TransferDir_ToSDIO;
		SDIO_DataInitStructure.SDIO_TransferMode=SDIO_TransferMode_Block;
		SDIO_DataConfig(&SDIO_DataInitStructure);
		
		

		SDIO_CmdInitStructure.SDIO_Argument =  addr;//����CMD18+��addr��ַ����ȡ����,����Ӧ 
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_READ_MULT_BLOCK;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);		
		
		errorstatus=CmdResp1Error(SD_CMD_READ_MULT_BLOCK);//�ȴ�R1��Ӧ   
		if(errorstatus!=SD_OK)return errorstatus;   	//��Ӧ����	  
 		if(DeviceMode==SD_POLLING_MODE)
		{
			INTX_DISABLE();//�ر����ж�(POLLINGģʽ,�Ͻ��жϴ��SDIO��д����!!!)
			while(!(SDIO->STA&((1<<5)|(1<<1)|(1<<3)|(1<<8)|(1<<9))))//������/CRC/��ʱ/���(��־)/��ʼλ����
			{
				if(SDIO->STA&(1<<15))						//����������,��ʾ���ٴ���8����
				{
					for(count=0;count<8;count++)			//ѭ����ȡ����
					{
						*(tempbuff+count)=SDIO->FIFO;
					}
					tempbuff+=8;	 
					timeout=0X7FFFF; 
				}else 	//����ʱ
				{
					if(timeout==0)return SD_DATA_TIMEOUT;
					timeout--;
				}
			}  
			if(SDIO->STA&(1<<3))		//���ݳ�ʱ����
			{										   
		 		SDIO->ICR|=1<<3; 		//������־
				return SD_DATA_TIMEOUT;
		 	}else if(SDIO->STA&(1<<1))	//���ݿ�CRC����
			{
		 		SDIO->ICR|=1<<1; 		//������־
				return SD_DATA_CRC_FAIL;		   
			}else if(SDIO->STA&(1<<5)) 	//����fifo�������
			{
		 		SDIO->ICR|=1<<5; 		//������־
				return SD_RX_OVERRUN;		 
			}else if(SDIO->STA&(1<<9)) 	//������ʼλ����
			{
		 		SDIO->ICR|=1<<9; 		//������־
				return SD_START_BIT_ERR;		 
			}   
			while(SDIO->STA&(1<<21))	//FIFO����,�����ڿ�������
			{
				*tempbuff=SDIO->FIFO;	//ѭ����ȡ����
				tempbuff++;
			}
	 		if(SDIO->STA&(1<<8))		//���ս���
			{
				if((SDIO_STD_CAPACITY_SD_CARD_V1_1==CardType)||(SDIO_STD_CAPACITY_SD_CARD_V2_0==CardType)||(SDIO_HIGH_CAPACITY_SD_CARD==CardType))
				{
  
				SDIO_CmdInitStructure.SDIO_Argument =  0;//����CMD12+��������
				SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_STOP_TRANSMISSION;
				SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
				SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
				SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
				SDIO_SendCommand(&SDIO_CmdInitStructure);		

					errorstatus=CmdResp1Error(SD_CMD_STOP_TRANSMISSION);//�ȴ�R1��Ӧ   
					if(errorstatus!=SD_OK)return errorstatus;	 
				}
 			}
			INTX_ENABLE();//�������ж�
	 		SDIO->ICR=0X5FF;	 		//������б�� 
 		}else if(DeviceMode==SD_DMA_MODE)
		{
	   		TransferError=SD_OK;
			StopCondition=1;			//����,��Ҫ����ֹͣ����ָ�� 
			TransferEnd=0;				//�����������λ�����жϷ�����1
			SDIO->MASK|=(1<<1)|(1<<3)|(1<<8)|(1<<5)|(1<<9);	//������Ҫ���ж� 
		 	SDIO->DCTRL|=1<<3;		 						//SDIO DMAʹ�� 
	 	    SD_DMA_Config((u32*)buf,nblks*blksize,DMA_DIR_PeripheralToMemory); 
	 		while(((DMA2->LISR&(1<<27))==RESET)&&timeout)timeout--;//�ȴ�������� 
			if(timeout==0)return SD_DATA_TIMEOUT;//��ʱ
			while((TransferEnd==0)&&(TransferError==SD_OK)); 
			if(TransferError!=SD_OK)errorstatus=TransferError;  	 
		}		 
  	}
	return errorstatus;
}			    																  
//SD��д1���� 
//buf:���ݻ�����
//addr:д��ַ
//blksize:���С	  
//����ֵ:����״̬
SD_Error SD_WriteBlock(u8 *buf,long long addr,  u16 blksize)
{
	SD_Error errorstatus = SD_OK;
	u8  power=0,cardstate=0;
	u32 timeout=0,bytestransferred=0;
	u32 cardstatus=0,count=0,restwords=0;
	u32	tlen=blksize;						//�ܳ���(�ֽ�)
	u32*tempbuff=(u32*)buf;								 
 	if(buf==NULL)return SD_INVALID_PARAMETER;//��������   
  	SDIO->DCTRL=0x0;							//���ݿ��ƼĴ�������(��DMA)   


	SDIO_DataInitStructure.SDIO_DataBlockSize= 0; ;//���DPSM״̬������
	SDIO_DataInitStructure.SDIO_DataLength= 0 ;
	SDIO_DataInitStructure.SDIO_DataTimeOut=SD_DATATIMEOUT ;
	SDIO_DataInitStructure.SDIO_DPSM=SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir=SDIO_TransferDir_ToCard;
	SDIO_DataInitStructure.SDIO_TransferMode=SDIO_TransferMode_Block;
  SDIO_DataConfig(&SDIO_DataInitStructure);
		
	
	
	if(SDIO->RESP1&SD_CARD_LOCKED)return SD_LOCK_UNLOCK_FAILED;//������
 	if(CardType==SDIO_HIGH_CAPACITY_SD_CARD)	//��������
	{
		blksize=512;
		addr>>=9;
	}    
	if((blksize>0)&&(blksize<=2048)&&((blksize&(blksize-1))==0))
	{
		power=convert_from_bytes_to_power_of_two(blksize);	    

		SDIO_CmdInitStructure.SDIO_Argument = blksize;//����CMD16+�������ݳ���Ϊblksize,����Ӧ 	
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);		

		
		errorstatus=CmdResp1Error(SD_CMD_SET_BLOCKLEN);	//�ȴ�R1��Ӧ   
		if(errorstatus!=SD_OK)return errorstatus;   	//��Ӧ����	 
	}else return SD_INVALID_PARAMETER;	 
  	

	SDIO_CmdInitStructure.SDIO_Argument = (u32)RCA<<16;//����CMD13,��ѯ����״̬,����Ӧ 	
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_STATUS;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);	

	
	errorstatus=CmdResp1Error(SD_CMD_SEND_STATUS);		//�ȴ�R1��Ӧ   		   
	if(errorstatus!=SD_OK)return errorstatus;
	cardstatus=SDIO->RESP1;													  
	timeout=SD_DATATIMEOUT;
   	while(((cardstatus&0x00000100)==0)&&(timeout>0)) 	//���READY_FOR_DATAλ�Ƿ���λ
	{
		timeout--;
 
		SDIO_CmdInitStructure.SDIO_Argument = (u32)RCA<<16;//����CMD13,��ѯ����״̬,����Ӧ
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_STATUS;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);	

		
		errorstatus=CmdResp1Error(SD_CMD_SEND_STATUS);	//�ȴ�R1��Ӧ   		   
		if(errorstatus!=SD_OK)return errorstatus;				    
		cardstatus=SDIO->RESP1;													  
	}
	if(timeout==0)return SD_ERROR;
 
	SDIO_CmdInitStructure.SDIO_Argument = addr;//����CMD13,��ѯ����״̬,����Ӧ
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_WRITE_SINGLE_BLOCK;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);	
	
	errorstatus=CmdResp1Error(SD_CMD_WRITE_SINGLE_BLOCK);//�ȴ�R1��Ӧ   		   
	if(errorstatus!=SD_OK)return errorstatus;   	  
	StopCondition=0;									//����д,����Ҫ����ֹͣ����ָ�� 
 
		SDIO_DataInitStructure.SDIO_DataBlockSize= power<<4; ;	//blksize, ����������	
	SDIO_DataInitStructure.SDIO_DataLength= blksize ;
	SDIO_DataInitStructure.SDIO_DataTimeOut=SD_DATATIMEOUT ;
	SDIO_DataInitStructure.SDIO_DPSM=SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir=SDIO_TransferDir_ToCard;
	SDIO_DataInitStructure.SDIO_TransferMode=SDIO_TransferMode_Block;
  SDIO_DataConfig(&SDIO_DataInitStructure);
	
	
	timeout=SDIO_DATATIMEOUT;
	if (DeviceMode == SD_POLLING_MODE)
	{
		INTX_DISABLE();//�ر����ж�(POLLINGģʽ,�Ͻ��жϴ��SDIO��д����!!!)
		while(!(SDIO->STA&((1<<10)|(1<<4)|(1<<1)|(1<<3)|(1<<9))))//���ݿ鷢�ͳɹ�/����/CRC/��ʱ/��ʼλ����
		{
			if(SDIO->STA&(1<<14))							//���������,��ʾ���ٴ���8����
			{
				if((tlen-bytestransferred)<SD_HALFFIFOBYTES)//����32�ֽ���
				{
					restwords=((tlen-bytestransferred)%4==0)?((tlen-bytestransferred)/4):((tlen-bytestransferred)/4+1);
					
					for(count=0;count<restwords;count++,tempbuff++,bytestransferred+=4)
					{
						SDIO->FIFO=*tempbuff;
					}
				}else
				{
					for(count=0;count<8;count++)
					{
						SDIO->FIFO=*(tempbuff+count);
					}
					tempbuff+=8;
					bytestransferred+=32;
				}
				timeout=0X7FFFF;
			}else
			{
				if(timeout==0)return SD_DATA_TIMEOUT;
				timeout--;
			}
		} 
		if(SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)		//���ݳ�ʱ����
		{										   
      SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
			return SD_DATA_TIMEOUT;
	 	}else if(SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)	//���ݿ�CRC����
		{
      SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
			return SD_DATA_CRC_FAIL;		   
		}else if(SDIO_GetFlagStatus(SDIO_FLAG_TXUNDERR) != RESET) 	//����fifo�������
		{
      SDIO_ClearFlag(SDIO_FLAG_TXUNDERR);
			return SD_TX_UNDERRUN;		 
		}else if(SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET) 	//������ʼλ����
		{
	 	  SDIO_ClearFlag(SDIO_FLAG_STBITERR);
			return SD_START_BIT_ERR;		 
		}   
		INTX_ENABLE();//�������ж�
		SDIO_ClearFlag(0X5FF);//������б��
	}else if(DeviceMode==SD_DMA_MODE)
	{
   		TransferError=SD_OK;
		StopCondition=0;			//����д,����Ҫ����ֹͣ����ָ�� 
		TransferEnd=0;				//�����������λ�����жϷ�����1
	
		SDIO_ITConfig(SDIO_IT_DCRCFAIL|SDIO_IT_DTIMEOUT|SDIO_IT_DATAEND|SDIO_IT_STBITERR|SDIO_IT_TXUNDERR,ENABLE);//���ò������ݽ�������ж�
		
		SD_DMA_Config((u32*)buf,blksize,DMA_DIR_MemoryToPeripheral);				//SDIO DMA����							.
    SDIO_DMACmd(ENABLE);	//SDIO DMAʹ��	
 		while(((DMA2->LISR&(1<<27))==RESET)&&timeout)timeout--;//�ȴ�������� 
		if(timeout==0)
		{
  			SD_Init();	 					//���³�ʼ��SD��,���Խ��д������������
			return SD_DATA_TIMEOUT;			//��ʱ	 
 		}
		timeout=SDIO_DATATIMEOUT;
		while((TransferEnd==0)&&(TransferError==SD_OK)&&timeout)timeout--;
 		if(timeout==0)return SD_DATA_TIMEOUT;			//��ʱ	 
  		if(TransferError!=SD_OK)return TransferError;
 	}  
	SDIO_ClearFlag(0X5FF);//������б��
 	errorstatus=IsCardProgramming(&cardstate);
 	while((errorstatus==SD_OK)&&((cardstate==SD_CARD_PROGRAMMING)||(cardstate==SD_CARD_RECEIVING)))
	{
		errorstatus=IsCardProgramming(&cardstate);
	}   
	return errorstatus;
}

//SD��д����� 
//buf:���ݻ�����
//addr:д��ַ
//blksize:���С
//nblks:Ҫд��Ŀ���
//����ֵ:����״̬	
SD_Error SD_WriteMultiBlocks(u8 *buf,long long addr,u16 blksize,u32 nblks)
{
	SD_Error errorstatus = SD_OK;
	u8  power = 0, cardstate = 0;
	u32 timeout=0,bytestransferred=0;
	u32 count = 0, restwords = 0;
	u32 tlen=nblks*blksize;				//�ܳ���(�ֽ�)
	u32 *tempbuff = (u32*)buf;  
  	if(buf==NULL)return SD_INVALID_PARAMETER; //��������  
  	SDIO->DCTRL=0x0;							//���ݿ��ƼĴ�������(��DMA)   

	SDIO_DataInitStructure.SDIO_DataBlockSize= 0; ;	//���DPSM״̬������	
	SDIO_DataInitStructure.SDIO_DataLength= 0 ;
	SDIO_DataInitStructure.SDIO_DataTimeOut=SD_DATATIMEOUT ;
	SDIO_DataInitStructure.SDIO_DPSM=SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir=SDIO_TransferDir_ToCard;
	SDIO_DataInitStructure.SDIO_TransferMode=SDIO_TransferMode_Block;
	SDIO_DataConfig(&SDIO_DataInitStructure);
	
	
	if(SDIO->RESP1&SD_CARD_LOCKED)return SD_LOCK_UNLOCK_FAILED;//������
 	if(CardType==SDIO_HIGH_CAPACITY_SD_CARD)//��������
	{
		blksize=512;
		addr>>=9;
	}    
	if((blksize>0)&&(blksize<=2048)&&((blksize&(blksize-1))==0))
	{
		power=convert_from_bytes_to_power_of_two(blksize);	    
 
		SDIO_CmdInitStructure.SDIO_Argument = blksize;	//����CMD16+�������ݳ���Ϊblksize,����Ӧ
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);	

		 errorstatus=CmdResp1Error(SD_CMD_SET_BLOCKLEN);	//�ȴ�R1��Ӧ   
		if(errorstatus!=SD_OK)return errorstatus;   	//��Ӧ����	 
	}else return SD_INVALID_PARAMETER;	 
	if(nblks>1)
	{					  
		if(nblks*blksize>SD_MAX_DATA_LENGTH)return SD_INVALID_PARAMETER;   
     	if((SDIO_STD_CAPACITY_SD_CARD_V1_1==CardType)||(SDIO_STD_CAPACITY_SD_CARD_V2_0==CardType)||(SDIO_HIGH_CAPACITY_SD_CARD==CardType))
    	{
 
				SDIO_CmdInitStructure.SDIO_Argument = (u32)RCA<<16;		//����ACMD55,����Ӧ 	
				SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
				SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
				SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
				SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
				SDIO_SendCommand(&SDIO_CmdInitStructure);	
				
			errorstatus=CmdResp1Error(SD_CMD_APP_CMD);		//�ȴ�R1��Ӧ   		   
			if(errorstatus!=SD_OK)return errorstatus;				    
 
				SDIO_CmdInitStructure.SDIO_Argument =nblks;		//����CMD23,���ÿ�����,����Ӧ 	 
				SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCK_COUNT;
				SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
				SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
				SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
				SDIO_SendCommand(&SDIO_CmdInitStructure);	
				
			errorstatus=CmdResp1Error(SD_CMD_SET_BLOCK_COUNT);//�ȴ�R1��Ӧ   		   
			if(errorstatus!=SD_OK)return errorstatus;				    
		} 
 
				SDIO_CmdInitStructure.SDIO_Argument =addr;	//����CMD25,���дָ��,����Ӧ 	  
				SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_WRITE_MULT_BLOCK;
				SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
				SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
				SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
				SDIO_SendCommand(&SDIO_CmdInitStructure);	
		
		errorstatus=CmdResp1Error(SD_CMD_WRITE_MULT_BLOCK);	//�ȴ�R1��Ӧ   		   
		if(errorstatus!=SD_OK)return errorstatus;

				SDIO_DataInitStructure.SDIO_DataBlockSize= power<<4; ;	//���DPSM״̬������	
				SDIO_DataInitStructure.SDIO_DataLength= nblks*blksize ;
				SDIO_DataInitStructure.SDIO_DataTimeOut=SD_DATATIMEOUT ;
				SDIO_DataInitStructure.SDIO_DPSM=SDIO_DPSM_Enable;
				SDIO_DataInitStructure.SDIO_TransferDir=SDIO_TransferDir_ToCard;
				SDIO_DataInitStructure.SDIO_TransferMode=SDIO_TransferMode_Block;
				SDIO_DataConfig(&SDIO_DataInitStructure);
		
		
		if(DeviceMode==SD_POLLING_MODE)
	    {
			timeout=SDIO_DATATIMEOUT;
			INTX_DISABLE();//�ر����ж�(POLLINGģʽ,�Ͻ��жϴ��SDIO��д����!!!)
			while(!(SDIO->STA&((1<<4)|(1<<1)|(1<<8)|(1<<3)|(1<<9))))//����/CRC/���ݽ���/��ʱ/��ʼλ����
			{
				if(SDIO->STA&(1<<14))							//���������,��ʾ���ٴ���8��(32�ֽ�)
				{	  
					if((tlen-bytestransferred)<SD_HALFFIFOBYTES)//����32�ֽ���
					{
						restwords=((tlen-bytestransferred)%4==0)?((tlen-bytestransferred)/4):((tlen-bytestransferred)/4+1);
						for(count=0;count<restwords;count++,tempbuff++,bytestransferred+=4)
						{
							SDIO->FIFO=*tempbuff;
						}
					}else 										//���������,���Է�������8��(32�ֽ�)����
					{
						for(count=0;count<SD_HALFFIFO;count++)
						{
							SDIO->FIFO=*(tempbuff+count);
						}
						tempbuff+=SD_HALFFIFO;
						bytestransferred+=SD_HALFFIFOBYTES;
					}
					timeout=0X7FFFF; 
				}else
				{
					if(timeout==0)return SD_DATA_TIMEOUT;
					timeout--;
				}
			} 
			if(SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)		//���ݳ�ʱ����
			{										   
		 		 SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);	//������־
				 return SD_DATA_TIMEOUT;
		 	}else if(SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)	//���ݿ�CRC����
			{
		 		 SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);		//������־
				return SD_DATA_CRC_FAIL;		   
			}else if(SDIO_GetFlagStatus(SDIO_FLAG_TXUNDERR) != RESET) 	//����fifo�������
			{
		 		SDIO_ClearFlag(SDIO_FLAG_TXUNDERR);		//������־
				return SD_TX_UNDERRUN;		 
			}else if(SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET) 	//������ʼλ����
			{
		 		SDIO_ClearFlag(SDIO_FLAG_STBITERR); 		//������־
				return SD_START_BIT_ERR;		 
			}   										   
			if(SDIO_GetFlagStatus(SDIO_FLAG_DATAEND) != RESET)		//���ͽ���
			{															 
				if((SDIO_STD_CAPACITY_SD_CARD_V1_1==CardType)||(SDIO_STD_CAPACITY_SD_CARD_V2_0==CardType)||(SDIO_HIGH_CAPACITY_SD_CARD==CardType))
				{
 
					SDIO_CmdInitStructure.SDIO_Argument =0;//����CMD12+�������� 	  
					SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_STOP_TRANSMISSION;
					SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
					SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
					SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
					SDIO_SendCommand(&SDIO_CmdInitStructure);	

					
					errorstatus=CmdResp1Error(SD_CMD_STOP_TRANSMISSION);//�ȴ�R1��Ӧ   
					if(errorstatus!=SD_OK)return errorstatus;	 
				}
			}
			INTX_ENABLE();//�������ж�
	 			SDIO_ClearFlag(0X5FF);//������б�� 
	    }else if(DeviceMode==SD_DMA_MODE)
		{
	   		TransferError=SD_OK;
			StopCondition=1;			//���д,��Ҫ����ֹͣ����ָ�� 
			TransferEnd=0;				//�����������λ�����жϷ�����1
			
			SDIO_ITConfig(SDIO_IT_DCRCFAIL|SDIO_IT_DTIMEOUT|SDIO_IT_DATAEND|SDIO_IT_STBITERR|SDIO_IT_TXUNDERR,ENABLE);//���ò������ݽ�������ж�
			SD_DMA_Config((u32*)buf,nblks*blksize,DMA_DIR_MemoryToPeripheral);		//SDIO DMA����
	 	 	SDIO_DMACmd(ENABLE);	//SDIO DMAʹ��	 
			timeout=SDIO_DATATIMEOUT;
	 		while(((DMA2->LISR&(1<<27))==RESET)&&timeout)timeout--;//�ȴ�������� 
			if(timeout==0)	 								//��ʱ
			{									  
  				SD_Init();	 					//���³�ʼ��SD��,���Խ��д������������
	 			return SD_DATA_TIMEOUT;			//��ʱ	 
	 		}
			timeout=SDIO_DATATIMEOUT;
			while((TransferEnd==0)&&(TransferError==SD_OK)&&timeout)timeout--;
	 		if(timeout==0)return SD_DATA_TIMEOUT;			//��ʱ	 
	 		if(TransferError!=SD_OK)return TransferError;	 
		}
  	}
 		SDIO_ClearFlag(0X5FF);//������б��
 	errorstatus=IsCardProgramming(&cardstate);
 	while((errorstatus==SD_OK)&&((cardstate==SD_CARD_PROGRAMMING)||(cardstate==SD_CARD_RECEIVING)))
	{
		errorstatus=IsCardProgramming(&cardstate);
	}   
	return errorstatus;	   
}
//SDIO�жϷ�����		  
void SDIO_IRQHandler(void) 
{											
 	SD_ProcessIRQSrc();//��������SDIO����ж�
}	 																    
//SDIO�жϴ�����
//����SDIO��������еĸ����ж�����
//����ֵ:�������
SD_Error SD_ProcessIRQSrc(void)
{
 if (StopCondition == 1)  //ʲôʱ����1�ˣ�
  {
    SDIO->ARG = 0x0;   //��������Ĵ���
    SDIO->CMD = 0x44C;	  // ����Ĵ����� 0100 	01 	 	001100
						//						[7:6]  	[5:0]
						//				CPSMEN  WAITRESP CMDINDEX
						//		��������״̬��	����Ӧ   cmd12 STOP_ TRANSMISSION						
    TransferError = CmdResp1Error(SD_CMD_STOP_TRANSMISSION);
  }
  else
  {
    TransferError = SD_OK;
  }
  SDIO_ClearITPendingBit(SDIO_IT_DATAEND); //���ж�
  SDIO_ITConfig(SDIO_IT_DATAEND, DISABLE); //�ر�sdio�ж�ʹ��
  TransferEnd = 1;
  return(TransferError);
}
  
//���CMD0��ִ��״̬
//����ֵ:sd��������
static SD_Error CmdError(void)
{
  SD_Error errorstatus = SD_OK;
  uint32_t timeout;

  timeout = SDIO_CMD0TIMEOUT; /*!< 10000 */

	 /*��������Ƿ��ѷ���*/
  while ((timeout > 0) && (SDIO_GetFlagStatus(SDIO_FLAG_CMDSENT) == RESET))	
  {
    timeout--;
  }

  if (timeout == 0)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    return(errorstatus);
  }

  /*!< Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);//�����̬��־λ

  return(errorstatus);
}
//���R7��Ӧ�Ĵ���״̬
//����ֵ:sd��������
static SD_Error CmdResp7Error(void)
{
	SD_Error errorstatus = SD_OK;
	uint32_t status;
	uint32_t timeout = SDIO_CMD0TIMEOUT;

	status = SDIO->STA;	//��ȡSDIO״̬�Ĵ��� ����״̬�Ĵ�����stm32�ļĴ���
 
	/* Command response received (CRC check failed) ��Command response received (CRC check passed)��Command response timeout */
	while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)) && (timeout > 0))
	{	
		timeout--;
		status = SDIO->STA;		  
	}
	 //������Ӧcmd8 ˵����������V2.0��
	if ((timeout == 0) || (status & SDIO_FLAG_CTIMEOUT))
	{
		errorstatus = SD_CMD_RSP_TIMEOUT;
		SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
		return(errorstatus);
	}

	//���յ�cmd8��Ӧ,˵������V2.0��
	if (status & SDIO_FLAG_CMDREND)
	{
		errorstatus = SD_OK;
		SDIO_ClearFlag(SDIO_FLAG_CMDREND);
		return(errorstatus);
	}
	return(errorstatus);
}
   
//���R1��Ӧ�Ĵ���״̬
//cmd:��ǰ����
//����ֵ:sd��������
static SD_Error CmdResp1Error(uint8_t cmd) //����Ĳ�����ʲô�ã�
{		   
	/*������Щ״̬�͵ȴ�	*/
  while (!(SDIO->STA & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)))
  {
  }

  SDIO->ICR = SDIO_STATIC_FLAGS;	//���жϱ�־

  return (SD_Error)(SDIO->RESP1 &  SD_OCR_ERRORBITS);		//�ж��Ƿ��ڹ��緶Χ
}
//���R3��Ӧ�Ĵ���״̬
//����ֵ:����״̬
static SD_Error CmdResp3Error(void)
{
  SD_Error errorstatus = SD_OK;
  uint32_t status;

  status = SDIO->STA;

  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)))
  {
    status = SDIO->STA;
  }

  if (status & SDIO_FLAG_CTIMEOUT)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  /*!< Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);
  return(errorstatus);
}

//���R2��Ӧ�Ĵ���״̬
//����ֵ:����״̬
static SD_Error CmdResp2Error(void)
{
  SD_Error errorstatus = SD_OK;
  uint32_t status;

  status = SDIO->STA;

  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CMDREND)))
  {
    status = SDIO->STA;
  }

  if (status & SDIO_FLAG_CTIMEOUT)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  else if (status & SDIO_FLAG_CCRCFAIL)
  {
    errorstatus = SD_CMD_CRC_FAIL;
    SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
    return(errorstatus);
  }

  /*!< Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  return(errorstatus);
}

//���R6��Ӧ�Ĵ���״̬
//cmd:֮ǰ���͵�����
//prca:�����ص�RCA��ַ
//����ֵ:����״̬
static SD_Error CmdResp6Error(uint8_t cmd, uint16_t *prca)
{
  SD_Error errorstatus = SD_OK;
  uint32_t status;
  uint32_t response_r1;

  status = SDIO->STA;

  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CMDREND)))
  {
    status = SDIO->STA;
  }

  if (status & SDIO_FLAG_CTIMEOUT)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  else if (status & SDIO_FLAG_CCRCFAIL)
  {
    errorstatus = SD_CMD_CRC_FAIL;
    SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
    return(errorstatus);
  }

  /*!< Check response received is of desired command */
  if (SDIO_GetCommandResponse() != cmd)		 //����Ƿ���յ���������
  {
    errorstatus = SD_ILLEGAL_CMD;
    return(errorstatus);
  }

  /*!< Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  /*!< We have received response, retrieve it.  */
  response_r1 = SDIO_GetResponse(SDIO_RESP1);

	/*����״̬ȫΪ0�����ɹ����յ�card���ص�rca */
  if (SD_ALLZERO == (response_r1 & (SD_R6_GENERAL_UNKNOWN_ERROR | SD_R6_ILLEGAL_CMD | SD_R6_COM_CRC_FAILED)))
  {
    *prca = (uint16_t) (response_r1 >> 16);//����16λ�����ǽ��յ��ķ���rca
    return(errorstatus);
  }

  if (response_r1 & SD_R6_GENERAL_UNKNOWN_ERROR)
  {
    return(SD_GENERAL_UNKNOWN_ERROR);
  }

  if (response_r1 & SD_R6_ILLEGAL_CMD)
  {
    return(SD_ILLEGAL_CMD);
  }

  if (response_r1 & SD_R6_COM_CRC_FAILED)
  {
    return(SD_COM_CRC_FAILED);
  }

  return(errorstatus);
}	   


//SDIOʹ�ܿ�����ģʽ
//����ֵ:����״̬
static SD_Error SDEnWideBus(FunctionalState NewState)
{
  SD_Error errorstatus = SD_OK;

  uint32_t scr[2] = {0, 0};

  if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED) //��⿨�Ƿ�������
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }

  /*!< Get SCR Register */
  errorstatus = FindSCR(RCA, scr);//��ȡscr�Ĵ������ݵ�scr������

  if (errorstatus != SD_OK)		  //degug,crc����scr��ȡ������ֵ
  {
    return(errorstatus);
  }

  /*!< If wide bus operation to be enabled */
  if (NewState == ENABLE)
  {
    /*!< If requested card supports wide bus operation */
    if ((scr[1] & SD_WIDE_BUS_SUPPORT) != SD_ALLZERO)  //�жϿ��Ƿ�֧��4λ��ʽ
    {
      /*!< Send CMD55 APP_CMD with argument as card's RCA.*/
      SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }

      /*!< Send ACMD6 APP_CMD with argument as 2 for wide bus mode */
	  /*����4bitģʽ������acmd6*/
      SDIO_CmdInitStructure.SDIO_Argument = 0x2;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_SD_SET_BUSWIDTH;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SD_CMD_APP_SD_SET_BUSWIDTH);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }
      return(errorstatus);
    }
    else
    {
      errorstatus = SD_REQUEST_NOT_APPLICABLE;
      return(errorstatus);
    }
  }   /*!< If wide bus operation to be disabled */
  else
  {
    /*!< If requested card supports 1 bit mode operation */
    if ((scr[1] & SD_SINGLE_BUS_SUPPORT) != SD_ALLZERO)
    {
      /*!< Send CMD55 APP_CMD with argument as card's RCA.*/
      SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);


      errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }

      /*!< Send ACMD6 APP_CMD with argument as 0 for single bus mode */
      SDIO_CmdInitStructure.SDIO_Argument = 0x00;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_SD_SET_BUSWIDTH;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SD_CMD_APP_SD_SET_BUSWIDTH);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }

      return(errorstatus);
    }
    else
    {
      errorstatus = SD_REQUEST_NOT_APPLICABLE;
      return(errorstatus);
    }
  }
}											   
//��鿨�Ƿ�����ִ��д����
//pstatus:��ǰ״̬.
//����ֵ:�������
SD_Error IsCardProgramming(u8 *pstatus)
{
  SD_Error errorstatus = SD_OK;
  __IO uint32_t respR1 = 0, status = 0;

	 /*cmd13�ÿ����Ϳ�״̬�Ĵ������洢��m3��λ��Ϊsdio_sta�Ĵ���*/
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16; //����Ե�ַ����
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_STATUS;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  status = SDIO->STA;
  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)))
  {
    status = SDIO->STA;
  }
	  /*һϵ�е�״̬�ж�*/
  if (status & SDIO_FLAG_CTIMEOUT)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  else if (status & SDIO_FLAG_CCRCFAIL)
  {
    errorstatus = SD_CMD_CRC_FAIL;
    SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
    return(errorstatus);
  }

  status = (uint32_t)SDIO_GetCommandResponse();

  /*!< Check response received is of desired command */
  if (status != SD_CMD_SEND_STATUS)
  {
    errorstatus = SD_ILLEGAL_CMD;
    return(errorstatus);
  }

  /*!< Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);


  /*!< We have received response, retrieve it for analysis  */
  respR1 = SDIO_GetResponse(SDIO_RESP1);

  /*!< Find out card status */
  *pstatus = (uint8_t) ((respR1 >> 9) & 0x0000000F);   //status[12:9] :cardstate 

  if ((respR1 & SD_OCR_ERRORBITS) == SD_ALLZERO)
  {
    return(errorstatus);
  }

  if (respR1 & SD_OCR_ADDR_OUT_OF_RANGE)
  {
    return(SD_ADDR_OUT_OF_RANGE);
  }

  if (respR1 & SD_OCR_ADDR_MISALIGNED)
  {
    return(SD_ADDR_MISALIGNED);
  }

  if (respR1 & SD_OCR_BLOCK_LEN_ERR)
  {
    return(SD_BLOCK_LEN_ERR);
  }

  if (respR1 & SD_OCR_ERASE_SEQ_ERR)
  {
    return(SD_ERASE_SEQ_ERR);
  }

  if (respR1 & SD_OCR_BAD_ERASE_PARAM)
  {
    return(SD_BAD_ERASE_PARAM);
  }

  if (respR1 & SD_OCR_WRITE_PROT_VIOLATION)
  {
    return(SD_WRITE_PROT_VIOLATION);
  }

  if (respR1 & SD_OCR_LOCK_UNLOCK_FAILED)
  {
    return(SD_LOCK_UNLOCK_FAILED);
  }

  if (respR1 & SD_OCR_COM_CRC_FAILED)
  {
    return(SD_COM_CRC_FAILED);
  }

  if (respR1 & SD_OCR_ILLEGAL_CMD)
  {
    return(SD_ILLEGAL_CMD);
  }

  if (respR1 & SD_OCR_CARD_ECC_FAILED)
  {
    return(SD_CARD_ECC_FAILED);
  }

  if (respR1 & SD_OCR_CC_ERROR)
  {
    return(SD_CC_ERROR);
  }

  if (respR1 & SD_OCR_GENERAL_UNKNOWN_ERROR)
  {
    return(SD_GENERAL_UNKNOWN_ERROR);
  }

  if (respR1 & SD_OCR_STREAM_READ_UNDERRUN)
  {
    return(SD_STREAM_READ_UNDERRUN);
  }

  if (respR1 & SD_OCR_STREAM_WRITE_OVERRUN)
  {
    return(SD_STREAM_WRITE_OVERRUN);
  }

  if (respR1 & SD_OCR_CID_CSD_OVERWRIETE)
  {
    return(SD_CID_CSD_OVERWRITE);
  }

  if (respR1 & SD_OCR_WP_ERASE_SKIP)
  {
    return(SD_WP_ERASE_SKIP);
  }

  if (respR1 & SD_OCR_CARD_ECC_DISABLED)
  {
    return(SD_CARD_ECC_DISABLED);
  }

  if (respR1 & SD_OCR_ERASE_RESET)
  {
    return(SD_ERASE_RESET);
  }

  if (respR1 & SD_OCR_AKE_SEQ_ERROR)
  {
    return(SD_AKE_SEQ_ERROR);
  }

  return(errorstatus);
}
//����SD����SCR�Ĵ���ֵ
//rca:����Ե�ַ
//pscr:���ݻ�����(�洢SCR����)
//����ֵ:����״̬		   
SD_Error FindSCR(u16 rca,u32 *pscr)
{ 

  uint32_t index = 0;
  SD_Error errorstatus = SD_OK;
  uint32_t tempscr[2] = {0, 0};	

  /*!< Set Block Size To 8 Bytes */ 
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)8;	 //���С�����sdhc�����޷��ı���С��	//ԭ����8
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN; //	 cmd16
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;  //r1
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  /*!< Send CMD55 APP_CMD with argument as card's RCA */
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16; 
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }
  /*�������ݽ��ռĴ���*/
  SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
  SDIO_DataInitStructure.SDIO_DataLength = 8;  //8byte,64λ
  SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_8b  ;  //���С8byte 
  SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
  SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
  SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
  SDIO_DataConfig(&SDIO_DataInitStructure);			 

  /*!< Send ACMD51 SD_APP_SEND_SCR with argument as 0 */
  SDIO_CmdInitStructure.SDIO_Argument = 0x0;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SD_APP_SEND_SCR;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;  //r1
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_SD_APP_SEND_SCR);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }			   
   
  /*�ȴ��������� */
  /*������Щ�����ѭ��*/																						  
 /*�������	  //����crcʧ��		//���ݳ�ʱ	  //�ѽ������ݿ飬crc���ɹ�	//û���������������ϼ�⵽��ʼ�ź�*/
 while (!(SDIO->STA & (SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DBCKEND| SDIO_FLAG_STBITERR)))
   {	   		
   if (SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET)	//���յ��������Ƿ����
	{  	
     	 	*(tempscr + index) = SDIO_ReadData();  
			   index++;	

		/*   //add������ڹٷ�Դ��û�м��ж�            */		     
		  	if(index > 1 ) 
				break;
    }
	
  }

  if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
    errorstatus = SD_DATA_TIMEOUT;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
    errorstatus = SD_DATA_CRC_FAIL;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
    errorstatus = SD_RX_OVERRUN;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_STBITERR);
    errorstatus = SD_START_BIT_ERR;
    return(errorstatus);
  }

  /*!< Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  *(pscr + 1) = ((tempscr[0] & SD_0TO7BITS) << 24) | ((tempscr[0] & SD_8TO15BITS) << 8) | ((tempscr[0] & SD_16TO23BITS) >> 8) | ((tempscr[0] & SD_24TO31BITS) >> 24);

  *(pscr) = ((tempscr[1] & SD_0TO7BITS) << 24) | ((tempscr[1] & SD_8TO15BITS) << 8) | ((tempscr[1] & SD_16TO23BITS) >> 8) | ((tempscr[1] & SD_24TO31BITS) >> 24);

  return(errorstatus);
}
//�õ�NumberOfBytes��2Ϊ�׵�ָ��.
//NumberOfBytes:�ֽ���.
//����ֵ:��2Ϊ�׵�ָ��ֵ
u8 convert_from_bytes_to_power_of_two(u16 NumberOfBytes)
{
	u8 count=0;
	while(NumberOfBytes!=1)
	{
		NumberOfBytes>>=1;
		count++;
	}
	return count;
} 	 



//����SDIO DMA  
//mbuf:�洢����ַ
//bufsize:����������
//dir:����;DMA_DIR_MemoryToPeripheral  �洢��-->SDIO(д����);DMA_DIR_PeripheralToMemory SDIO-->�洢��(������);
void SD_DMA_Config(u32*mbuf,u32 bufsize,u32 dir)
{		 

  DMA_InitTypeDef  DMA_InitStructure;
	
	while (DMA_GetCmdStatus(DMA2_Stream3) != DISABLE){}//�ȴ�DMA������ 
		
  DMA_DeInit(DMA2_Stream3);//���֮ǰ��stream3�ϵ������жϱ�־
	
 
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //ͨ��ѡ��
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SDIO->FIFO;//DMA�����ַ
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)mbuf;//DMA �洢��0��ַ
  DMA_InitStructure.DMA_DIR = dir;//�洢��������ģʽ
  DMA_InitStructure.DMA_BufferSize = 0;//���ݴ����� 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;//�������ݳ���:32λ
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;//�洢�����ݳ���:32λ
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// ʹ����ͨģʽ 
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//������ȼ�
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;   //FIFOʹ��      
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;//ȫFIFO
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;//����ͻ��4�δ���
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;//�洢��ͻ��4�δ���
  DMA_Init(DMA2_Stream3, &DMA_InitStructure);//��ʼ��DMA Stream

	DMA_FlowControllerConfig(DMA2_Stream3,DMA_FlowCtrl_Peripheral);//���������� 
	 
  DMA_Cmd(DMA2_Stream3 ,ENABLE);//����DMA����	 

}   
//��SD��
//buf:�����ݻ�����
//sector:������ַ
//cnt:��������	
//����ֵ:����״̬;0,����;����,�������;				  				 
u8 SD_ReadDisk(u8*buf,u32 sector,u8 cnt)
{
	u8 sta=SD_OK;
	long long lsector=sector;
	u8 n;
	if(CardType!=SDIO_STD_CAPACITY_SD_CARD_V1_1)lsector<<=9;
	if((u32)buf%4!=0)
	{
	 	for(n=0;n<cnt;n++)
		{
		 	sta=SD_ReadBlock(SDIO_DATA_BUFFER,lsector+512*n,512);//����sector�Ķ�����
			memcpy(buf,SDIO_DATA_BUFFER,512);
			buf+=512;
		} 
	}else
	{
		if(cnt==1)sta=SD_ReadBlock(buf,lsector,512);    	//����sector�Ķ�����
		else sta=SD_ReadMultiBlocks(buf,lsector,512,cnt);//���sector  
	}
	return sta;
}
//дSD��
//buf:д���ݻ�����
//sector:������ַ
//cnt:��������	
//����ֵ:����״̬;0,����;����,�������;	
u8 SD_WriteDisk(u8*buf,u32 sector,u8 cnt)
{
	u8 sta=SD_OK;
	u8 n;
	long long lsector=sector;
	if(CardType!=SDIO_STD_CAPACITY_SD_CARD_V1_1)lsector<<=9;
	if((u32)buf%4!=0)
	{
	 	for(n=0;n<cnt;n++)
		{
			memcpy(SDIO_DATA_BUFFER,buf,512);
		 	sta=SD_WriteBlock(SDIO_DATA_BUFFER,lsector+512*n,512);//����sector��д����
			buf+=512;
		} 
	}else
	{
		if(cnt==1)sta=SD_WriteBlock(buf,lsector,512);    	//����sector��д����
		else sta=SD_WriteMultiBlocks(buf,lsector,512,cnt);	//���sector  
	}
	return sta;
}







