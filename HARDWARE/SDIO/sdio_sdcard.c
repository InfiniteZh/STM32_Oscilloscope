#include "sdio_sdcard.h"
#include "string.h"	 
#include "sys.h"	 
#include "usart.h"	 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//SDIO 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/14
//版本：V1.1
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved		
//V1.1 
//1,加入超时判断,解决轮询接收死机的问题.
////////////////////////////////////////////////////////////////////////////////// 	 


/*用于sdio初始化的结构体*/
SDIO_InitTypeDef SDIO_InitStructure;
SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
SDIO_DataInitTypeDef SDIO_DataInitStructure;   


static u8 CardType=SDIO_STD_CAPACITY_SD_CARD_V1_1;		//SD卡类型（默认为1.x卡）
static u32 CSD_Tab[4],CID_Tab[4],RCA=0;					//SD卡CSD,CID以及相对地址(RCA)数据
static u8 DeviceMode=SD_DMA_MODE;		   				//工作模式,注意,工作模式必须通过SD_SetDeviceMode,后才算数.这里只是定义一个默认的模式(SD_DMA_MODE)
static u8 StopCondition=0; 								//是否发送停止传输标志位,DMA多块读写的时候用到  
volatile SD_Error TransferError=SD_OK;					//数据传输错误标志,DMA读写时使用	    
volatile u8 TransferEnd=0;								//传输结束标志,DMA读写时使用
SD_CardInfo SDCardInfo;									//SD卡信息


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


//SD_ReadDisk/SD_WriteDisk函数专用buf,当这两个函数的数据缓存区地址不是4字节对齐的时候,
//需要用到该数组,确保数据缓存区地址是4字节对齐的.
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

//初始化SD卡
//返回值:错误代码;(0,无错误)
SD_Error SD_Init(void)
{
	SD_Error errorstatus=SD_OK;	   
	//SDIO IO口初始化   	 
 
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_DMA2, ENABLE);//使能GPIOC,GPIOD DMA2时钟
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDIO, ENABLE);//SDIO时钟使能
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SDIO, ENABLE);//SDIO复位
	
	
	GPIO_InitStructure.GPIO_Pin =0X3F<<8; 	//PC8,9,10,11,12复用功能输出	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);// PC8,9,10,11,12复用功能输出

	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_2;
	GPIO_Init(GPIOD, &GPIO_InitStructure);//PD2复用功能输出
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_SDIO); //PC8,AF12
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_SDIO);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_SDIO);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_SDIO);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_SDIO);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_SDIO);		

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SDIO, DISABLE);//SDIO结束复位
	
 	//SDIO外设寄存器设置为默认值 			   
	SD_Register_DeInit();
	SDIO_DeInit();
 
	NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

	errorstatus=SD_PowerON();			//SD卡上电
 	
	if(errorstatus==SD_OK)errorstatus=SD_InitializeCards();			//初始化SD卡														  
	if(errorstatus==SD_OK)errorstatus=SD_GetCardInfo(&SDCardInfo);	//获取卡信息
 	if(errorstatus==SD_OK)errorstatus=SD_SelectDeselect((u32)(SDCardInfo.RCA<<16));//选中SD卡   
	if(errorstatus==SD_OK)errorstatus=SD_EnableWideBusOperation(SDIO_BusWide_4b);	//4位宽度,如果是MMC卡,则不能用4位模式 
	if((errorstatus==SD_OK)||(SDIO_MULTIMEDIA_CARD==CardType))
	{  		    
		SDIO_Clock_Set(SDIO_TRANSFER_CLK_DIV);			//设置时钟频率,SDIO时钟计算公式:SDIO_CK时钟=SDIOCLK/[clkdiv+2];其中,SDIOCLK固定为48Mhz 
		//errorstatus=SD_SetDeviceMode(SD_DMA_MODE);//设置为DMA模式
		errorstatus=SD_SetDeviceMode(SD_POLLING_MODE);//设置为查询模式
 	}
	return errorstatus;		 
}
//SDIO时钟初始化设置
//clkdiv:时钟分频系数
//CK时钟=SDIOCLK/[clkdiv+2];(SDIOCLK时钟固定为48Mhz)
void SDIO_Clock_Set(u8 clkdiv)
{
	u32 tmpreg=SDIO->CLKCR; 
  	tmpreg&=0XFFFFFF00; 
 	tmpreg|=clkdiv; 
	SDIO->CLKCR=tmpreg;
} 

//卡上电
//查询所有SDIO接口上的卡设备,并查询其电压和配置时钟
//返回值:错误代码;(0,无错误)
SD_Error SD_PowerON(void)
{
	
  SD_Error errorstatus = SD_OK;
  uint32_t response = 0, count = 0, validvoltage = 0;
  uint32_t SDType = SD_STD_CAPACITY;

  //初始化时的时钟不能大于400KHz 
  SDIO_InitStructure.SDIO_ClockDiv = SDIO_INIT_CLK_DIV;	//SDIOCLK = 48MHz, SDIO_CK = SDIOCLK/(118 + 2) = 400 KHz 
  SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
  SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;  //不使用bypass模式，直SDIOCLK进行分频得到SDIO_CK
  SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;	// 空闲时不关闭时钟电源
  SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;	 				//1位数据线
  SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;//关闭硬件流控
  SDIO_Init(&SDIO_InitStructure);

  SDIO_SetPowerState(SDIO_PowerState_ON); //打开SDIO电源

  SDIO_ClockCmd(ENABLE);  //开启SDIO时钟
   
  //下面发送一系列命令,开始卡识别流程
  SDIO_CmdInitStructure.SDIO_Argument = 0x0;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_GO_IDLE_STATE; //cmd0
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_No;  //无响应
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;  //则CPSM在开始发送命令之前等待数据传输结束。 
  SDIO_SendCommand(&SDIO_CmdInitStructure);	  		//写命令进命令寄存器

  errorstatus = CmdError();//检测是否正确接收到cmd0

  if (errorstatus != SD_OK)	//命令发送出错，返回
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
  SDIO_CmdInitStructure.SDIO_Argument = SD_CHECK_PATTERN;	//接收到命令sd会返回这个参数
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SEND_IF_COND;	//cmd8
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;	 //r7
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;			 //关闭等待中断
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);
   
  /*检查是否接收到命令*/
  errorstatus = CmdResp7Error(); 
	
  if (errorstatus == SD_OK)	  	//有响应则card遵循sd协议2.0版本
  {
    CardType = SDIO_STD_CAPACITY_SD_CARD_V2_0; /*!< SD Card 2.0 ，先把它定义会sdsc类型的卡*/
    SDType = SD_HIGH_CAPACITY;	//这个变量用作acmd41的参数，用来询问是sdsc卡还是sdhc卡
  }
  else	//无响应，说明是1.x的或mmc的卡
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
  /*!< CMD55 */		//为什么在else里和else外面都要发送CMD55? 
  //发送cmd55，用于检测是sd卡还是mmc卡，或是不支持的卡  
  SDIO_CmdInitStructure.SDIO_Argument = 0x00;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short; //r1
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);
  errorstatus = CmdResp1Error(SD_CMD_APP_CMD);	//是否响应，没响应的是mmc或不支持的卡


  if (errorstatus == SD_OK)	//响应了cmd55，是sd卡，可能为1.x,可能为2.0
  {
  	//下面开始循环地发送sdio支持的电压范围，循环一定次数

	 //发送ACMD42 参数为0X80100000
    while ((!validvoltage) && (count < SD_MAX_VOLT_TRIAL))
    {	 
	  //因为下面要用到ACMD41，是ACMD命令，在发送ACMD命令前都要先向卡发送CMD55
      SDIO_CmdInitStructure.SDIO_Argument = 0x00;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;	  //CMD55
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SD_CMD_APP_CMD); //检测响应

      if (errorstatus != SD_OK)
      {
        return(errorstatus);//没响应CMD55，返回  
      }
	  //acmd41，命令参数由支持的电压范围及HCS位组成，HCS位置一来区分卡是SDSc还是sdhc
      SDIO_CmdInitStructure.SDIO_Argument = SD_VOLTAGE_WINDOW_SD | SDType;	  //参数为主机可供电压范围及hcs位
      SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SD_APP_OP_COND;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;  //r3
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp3Error();	//检测是否正确接收到数据
      if (errorstatus != SD_OK)
      {
        return(errorstatus);  //没正确接收到acmd41，出错，返回
      }
	   //若卡需求电压在SDIO的供电电压范围内，会自动上电并标志pwr_up位
      response = SDIO_GetResponse(SDIO_RESP1);	 //读取卡寄存器，卡状态
      validvoltage = (((response >> 31) == 1) ? 1 : 0);	//读取卡的ocr寄存器的pwr_up位，看是否已工作在正常电压
      count++;			  //计算循环次数
    }
    if (count >= SD_MAX_VOLT_TRIAL)	//循环检测超过一定次数还没上电
    {
      errorstatus = SD_INVALID_VOLTRANGE;	   //SDIO不支持card的供电电压
      return(errorstatus);
    }
			 //检查卡返回信息中的HCS位
    if (response &= SD_HIGH_CAPACITY)  //判断ocr中的ccs位 ，如果是sdsc卡则不执行下面的语句
    {
      CardType = SDIO_HIGH_CAPACITY_SD_CARD;  //把卡类型从初始化的sdsc型改为sdhc型
    }

  }/*!< else MMC Card */

  return(errorstatus);		
}

//SD卡 Power OFF
//返回值:错误代码;(0,无错误)
SD_Error SD_PowerOFF(void)
{
  SD_Error errorstatus = SD_OK;

  //关闭SD供电
  SDIO_SetPowerState(SDIO_PowerState_OFF);

  return(errorstatus);
}
 
//初始化所有的卡,并让卡进入就绪状态
//返回值:错误代码
SD_Error SD_InitializeCards(void)
{
  SD_Error errorstatus = SD_OK;
  uint16_t rca = 0x01;

  if (SDIO_GetPowerState() == SDIO_PowerState_OFF)  
  {
    errorstatus = SD_REQUEST_NOT_APPLICABLE;
    return(errorstatus);
  }
	
  if (SDIO_SECURE_DIGITAL_IO_CARD != CardType)//判断卡的类型
  {
	 //发送CMD2命令 获取卡CID信息
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

     /*下面开始SD卡初始化流程*/
  if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) ||  (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) ||  (SDIO_SECURE_DIGITAL_IO_COMBO_CARD == CardType)
      ||  (SDIO_HIGH_CAPACITY_SD_CARD == CardType))	 //使用的是2.0的卡
  {
	 //发送CMD3命令 获取SD卡相对地址
    SDIO_CmdInitStructure.SDIO_Argument = 0x00;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_REL_ADDR;	//cmd3
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short; //r6
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp6Error(SD_CMD_SET_REL_ADDR, &rca);	//把接收到的卡相对地址存起来。

    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }
  }

  if (SDIO_SECURE_DIGITAL_IO_CARD != CardType)
  {
    RCA = rca;

    /*!< Send CMD9 SEND_CSD with argument as card's RCA */
	 //发送CMD9  获取SD卡相关信息
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
//得到卡信息
//cardinfo:卡信息存储区
//返回值:错误状态
SD_Error SD_GetCardInfo(SD_CardInfo *cardinfo)
{
 	SD_Error errorstatus=SD_OK;
	u8 tmp=0;	   
	cardinfo->CardType=(u8)CardType; 				//卡类型
	cardinfo->RCA=(u16)RCA;							//卡RCA值
	tmp=(u8)((CSD_Tab[0]&0xFF000000)>>24);
	cardinfo->SD_csd.CSDStruct=(tmp&0xC0)>>6;		//CSD结构
	cardinfo->SD_csd.SysSpecVersion=(tmp&0x3C)>>2;	//2.0协议还没定义这部分(为保留),应该是后续协议定义的
	cardinfo->SD_csd.Reserved1=tmp&0x03;			//2个保留位 
	
	tmp=(u8)((CSD_Tab[0]&0x00FF0000)>>16);			//第1个字节
	cardinfo->SD_csd.TAAC=tmp;				   		//数据读时间1
	
	tmp=(u8)((CSD_Tab[0]&0x0000FF00)>>8);	  		//第2个字节
	cardinfo->SD_csd.NSAC=tmp;		  				//数据读时间2
	
	tmp=(u8)(CSD_Tab[0]&0x000000FF);				//第3个字节
	cardinfo->SD_csd.MaxBusClkFrec=tmp;		  		//传输速度	 
  
	tmp=(u8)((CSD_Tab[1]&0xFF000000)>>24);			//第4个字节
	cardinfo->SD_csd.CardComdClasses=tmp<<4;    	//卡指令类高四位
	
	tmp=(u8)((CSD_Tab[1]&0x00FF0000)>>16);	 		//第5个字节
	cardinfo->SD_csd.CardComdClasses|=(tmp&0xF0)>>4;//卡指令类低四位
	cardinfo->SD_csd.RdBlockLen=tmp&0x0F;	    	//最大读取数据长度
	
	tmp=(u8)((CSD_Tab[1]&0x0000FF00)>>8);			//第6个字节
	cardinfo->SD_csd.PartBlockRead=(tmp&0x80)>>7;	//允许分块读
	cardinfo->SD_csd.WrBlockMisalign=(tmp&0x40)>>6;	//写块错位
	cardinfo->SD_csd.RdBlockMisalign=(tmp&0x20)>>5;	//读块错位
	cardinfo->SD_csd.DSRImpl=(tmp&0x10)>>4;
	cardinfo->SD_csd.Reserved2=0; 					//保留
	
	
 	if((CardType==SDIO_STD_CAPACITY_SD_CARD_V1_1)||(CardType==SDIO_STD_CAPACITY_SD_CARD_V2_0)||(SDIO_MULTIMEDIA_CARD==CardType))//标准1.1/2.0卡/MMC卡
	{
		cardinfo->SD_csd.DeviceSize=(tmp&0x03)<<10;	//C_SIZE(12位)
	 	tmp=(u8)(CSD_Tab[1]&0x000000FF); 			//第7个字节	
		cardinfo->SD_csd.DeviceSize|=(tmp)<<2;
		
 		tmp=(u8)((CSD_Tab[2]&0xFF000000)>>24);		//第8个字节	
		cardinfo->SD_csd.DeviceSize|=(tmp&0xC0)>>6;
 		cardinfo->SD_csd.MaxRdCurrentVDDMin=(tmp&0x38)>>3;
		cardinfo->SD_csd.MaxRdCurrentVDDMax=(tmp&0x07);
		
 		tmp=(u8)((CSD_Tab[2]&0x00FF0000)>>16);		//第9个字节	
		cardinfo->SD_csd.MaxWrCurrentVDDMin=(tmp&0xE0)>>5;
		cardinfo->SD_csd.MaxWrCurrentVDDMax=(tmp&0x1C)>>2;
		cardinfo->SD_csd.DeviceSizeMul=(tmp&0x03)<<1;//C_SIZE_MULT
		
 		tmp=(u8)((CSD_Tab[2]&0x0000FF00)>>8);	  	//第10个字节	
		cardinfo->SD_csd.DeviceSizeMul|=(tmp&0x80)>>7;
 		cardinfo->CardCapacity=(cardinfo->SD_csd.DeviceSize+1);//计算卡容量------------------------------------
		cardinfo->CardCapacity*=(1<<(cardinfo->SD_csd.DeviceSizeMul+2));
		cardinfo->CardBlockSize=1<<(cardinfo->SD_csd.RdBlockLen);//块大小
		cardinfo->CardCapacity*=cardinfo->CardBlockSize;
	}else if(CardType==SDIO_HIGH_CAPACITY_SD_CARD)	//高容量卡
	{
 		tmp=(u8)(CSD_Tab[1]&0x000000FF); 		//第7个字节	
		cardinfo->SD_csd.DeviceSize=(tmp&0x3F)<<16;//C_SIZE
		
 		tmp=(u8)((CSD_Tab[2]&0xFF000000)>>24); 	//第8个字节	
 		cardinfo->SD_csd.DeviceSize|=(tmp<<8);
		
 		tmp=(u8)((CSD_Tab[2]&0x00FF0000)>>16);	//第9个字节	
 		cardinfo->SD_csd.DeviceSize|=(tmp);
		
 		tmp=(u8)((CSD_Tab[2]&0x0000FF00)>>8); 	//第10个字节	
 		cardinfo->CardCapacity=(long long)(cardinfo->SD_csd.DeviceSize+1)*512*1024;//计算卡容量
		cardinfo->CardBlockSize=512; 			//块大小固定为512字节
	}	  
	cardinfo->SD_csd.EraseGrSize=(tmp&0x40)>>6;
	cardinfo->SD_csd.EraseGrMul=(tmp&0x3F)<<1;	   
	tmp=(u8)(CSD_Tab[2]&0x000000FF);			//第11个字节	
	
	
	cardinfo->SD_csd.EraseGrMul|=(tmp&0x80)>>7;
	cardinfo->SD_csd.WrProtectGrSize=(tmp&0x7F);
 	tmp=(u8)((CSD_Tab[3]&0xFF000000)>>24);		//第12个字节	
	
	
	cardinfo->SD_csd.WrProtectGrEnable=(tmp&0x80)>>7;
	cardinfo->SD_csd.ManDeflECC=(tmp&0x60)>>5;
	cardinfo->SD_csd.WrSpeedFact=(tmp&0x1C)>>2;
	cardinfo->SD_csd.MaxWrBlockLen=(tmp&0x03)<<2;	 
	tmp=(u8)((CSD_Tab[3]&0x00FF0000)>>16);		//第13个字节
	
	
	cardinfo->SD_csd.MaxWrBlockLen|=(tmp&0xC0)>>6;
	cardinfo->SD_csd.WriteBlockPaPartial=(tmp&0x20)>>5;
	cardinfo->SD_csd.Reserved3=0;
	cardinfo->SD_csd.ContentProtectAppli=(tmp&0x01);  
	
	
	tmp=(u8)((CSD_Tab[3]&0x0000FF00)>>8);		//第14个字节
	cardinfo->SD_csd.FileFormatGrouop=(tmp&0x80)>>7;
	cardinfo->SD_csd.CopyFlag=(tmp&0x40)>>6;
	cardinfo->SD_csd.PermWrProtect=(tmp&0x20)>>5;
	cardinfo->SD_csd.TempWrProtect=(tmp&0x10)>>4;
	cardinfo->SD_csd.FileFormat=(tmp&0x0C)>>2;
	cardinfo->SD_csd.ECC=(tmp&0x03);  
	
	
	tmp=(u8)(CSD_Tab[3]&0x000000FF);			//第15个字节
	cardinfo->SD_csd.CSD_CRC=(tmp&0xFE)>>1;
	cardinfo->SD_csd.Reserved4=1;		 
	tmp=(u8)((CID_Tab[0]&0xFF000000)>>24);		//第0个字节
	cardinfo->SD_cid.ManufacturerID=tmp;		    
	tmp=(u8)((CID_Tab[0]&0x00FF0000)>>16);		//第1个字节
	cardinfo->SD_cid.OEM_AppliID=tmp<<8;	  
	tmp=(u8)((CID_Tab[0]&0x000000FF00)>>8);		//第2个字节
	cardinfo->SD_cid.OEM_AppliID|=tmp;	    
	tmp=(u8)(CID_Tab[0]&0x000000FF);			//第3个字节	
	cardinfo->SD_cid.ProdName1=tmp<<24;				  
	tmp=(u8)((CID_Tab[1]&0xFF000000)>>24); 		//第4个字节
	cardinfo->SD_cid.ProdName1|=tmp<<16;	  
	tmp=(u8)((CID_Tab[1]&0x00FF0000)>>16);	   	//第5个字节
	cardinfo->SD_cid.ProdName1|=tmp<<8;		 
	tmp=(u8)((CID_Tab[1]&0x0000FF00)>>8);		//第6个字节
	cardinfo->SD_cid.ProdName1|=tmp;		   
	tmp=(u8)(CID_Tab[1]&0x000000FF);	  		//第7个字节
	cardinfo->SD_cid.ProdName2=tmp;			  
	tmp=(u8)((CID_Tab[2]&0xFF000000)>>24); 		//第8个字节
	cardinfo->SD_cid.ProdRev=tmp;		 
	tmp=(u8)((CID_Tab[2]&0x00FF0000)>>16);		//第9个字节
	cardinfo->SD_cid.ProdSN=tmp<<24;	   
	tmp=(u8)((CID_Tab[2]&0x0000FF00)>>8); 		//第10个字节
	cardinfo->SD_cid.ProdSN|=tmp<<16;	   
	tmp=(u8)(CID_Tab[2]&0x000000FF);   			//第11个字节
	cardinfo->SD_cid.ProdSN|=tmp<<8;		   
	tmp=(u8)((CID_Tab[3]&0xFF000000)>>24); 		//第12个字节
	cardinfo->SD_cid.ProdSN|=tmp;			     
	tmp=(u8)((CID_Tab[3]&0x00FF0000)>>16);	 	//第13个字节
	cardinfo->SD_cid.Reserved1|=(tmp&0xF0)>>4;
	cardinfo->SD_cid.ManufactDate=(tmp&0x0F)<<8;    
	tmp=(u8)((CID_Tab[3]&0x0000FF00)>>8);		//第14个字节
	cardinfo->SD_cid.ManufactDate|=tmp;		 	  
	tmp=(u8)(CID_Tab[3]&0x000000FF);			//第15个字节
	cardinfo->SD_cid.CID_CRC=(tmp&0xFE)>>1;
	cardinfo->SD_cid.Reserved2=1;	 
	return errorstatus;
}

//设置SDIO总线宽度(MMC卡不支持4bit模式)
//   @arg SDIO_BusWide_8b: 8-bit data transfer (Only for MMC)
//   @arg SDIO_BusWide_4b: 4-bit data transfer
//   @arg SDIO_BusWide_1b: 1-bit data transfer (默认)
//返回值:SD卡错误状态
SD_Error SD_EnableWideBusOperation(uint32_t WideMode)
{
  SD_Error errorstatus = SD_OK;

	//MMC卡不支持
  if (SDIO_MULTIMEDIA_CARD == CardType)
  {
    errorstatus = SD_UNSUPPORTED_FEATURE;
    return(errorstatus);
  }
  else if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) || (SDIO_HIGH_CAPACITY_SD_CARD == CardType))
  {														 
    if (SDIO_BusWide_8b == WideMode)   //2.0 sd不支持8bits
    {
      errorstatus = SD_UNSUPPORTED_FEATURE;
      return(errorstatus);
    }
    else if (SDIO_BusWide_4b == WideMode)//4数据线模式
    {
      errorstatus = SDEnWideBus(ENABLE);//使用acmd6设置总线宽度，设置卡的传输方式

      if (SD_OK == errorstatus)
      {
        /*!< Configure the SDIO peripheral */
        SDIO_InitStructure.SDIO_ClockDiv = SDIO_TRANSFER_CLK_DIV; 
        SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
        SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
        SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
        SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_4b;	//这个是设置stm32的sdio的传输方式 ，切换模式必须从卡和sdio都对应好
        SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
        SDIO_Init(&SDIO_InitStructure);
      }
    }
    else//单数据线模式
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

//设置SD卡工作模式
//Mode:
//返回值:错误状态
SD_Error SD_SetDeviceMode(u32 Mode)
{
	SD_Error errorstatus = SD_OK;
 	if((Mode==SD_DMA_MODE)||(Mode==SD_POLLING_MODE))DeviceMode=Mode;
	else errorstatus=SD_INVALID_PARAMETER;
	return errorstatus;	    
}
//选卡
//发送CMD7,选择相对地址(rca)为addr的卡,取消其他卡.如果为0,则都不选择.
//addr:卡的RCA地址
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
//SD卡读取一个块 
//buf:读数据缓存区(必须4字节对齐!!)
//addr:读取地址
//blksize:块大小
SD_Error SD_ReadBlock(u8 *buf,long long addr,u16 blksize)
{	  
	SD_Error errorstatus=SD_OK;
	u8 power;
   	u32 count=0,*tempbuff=(u32*)buf;//转换为u32指针 
	u32 timeout=SDIO_DATATIMEOUT;   
   	if(NULL==buf)return SD_INVALID_PARAMETER; 
   	SDIO->DCTRL=0x0;	//数据控制寄存器清零(关DMA)   
	if(CardType==SDIO_HIGH_CAPACITY_SD_CARD)//大容量卡
	{
		blksize=512;
		addr>>=9;
	}   
 
	SDIO_DataInitStructure.SDIO_DataBlockSize= 0 ;//清除DPSM状态机配置
	SDIO_DataInitStructure.SDIO_DataLength= 0 ;
	SDIO_DataInitStructure.SDIO_DataTimeOut=SD_DATATIMEOUT ;
	SDIO_DataInitStructure.SDIO_DPSM=SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir=SDIO_TransferDir_ToCard;
	SDIO_DataInitStructure.SDIO_TransferMode=SDIO_TransferMode_Block;
	SDIO_DataConfig(&SDIO_DataInitStructure);

	
	if(SDIO->RESP1&SD_CARD_LOCKED)return SD_LOCK_UNLOCK_FAILED;//卡锁了
	if((blksize>0)&&(blksize<=2048)&&((blksize&(blksize-1))==0))
	{
		power=convert_from_bytes_to_power_of_two(blksize);	    	   

		SDIO_CmdInitStructure.SDIO_Argument =  blksize;//发送CMD16+设置数据长度为blksize,短响应 
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);
		
		errorstatus=CmdResp1Error(SD_CMD_SET_BLOCKLEN);	//等待R1响应   
		if(errorstatus!=SD_OK)return errorstatus;   	//响应错误	 
	}else return SD_INVALID_PARAMETER;	  	  									    
   
	SDIO_DataInitStructure.SDIO_DataBlockSize= power<<4; ;//清除DPSM状态机配置
	SDIO_DataInitStructure.SDIO_DataLength= blksize ;
	SDIO_DataInitStructure.SDIO_DataTimeOut=SD_DATATIMEOUT ;
	SDIO_DataInitStructure.SDIO_DPSM=SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir=SDIO_TransferDir_ToSDIO;
	SDIO_DataInitStructure.SDIO_TransferMode=SDIO_TransferMode_Block;
	SDIO_DataConfig(&SDIO_DataInitStructure);
	
	
	SDIO_CmdInitStructure.SDIO_Argument =  addr;//发送CMD17+从addr地址出读取数据,短响应 	
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_READ_SINGLE_BLOCK;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);
	
	errorstatus=CmdResp1Error(SD_CMD_READ_SINGLE_BLOCK);//等待R1响应   
	if(errorstatus!=SD_OK)return errorstatus;   		//响应错误	 
 	if(DeviceMode==SD_POLLING_MODE)						//查询模式,轮询数据	 
	{
 		INTX_DISABLE();//关闭总中断(POLLING模式,严禁中断打断SDIO读写操作!!!)
		while(!(SDIO->STA&((1<<5)|(1<<1)|(1<<3)|(1<<10)|(1<<9))))//无上溢/CRC/超时/完成(标志)/起始位错误
		{
			if(SDIO->STA&(1<<15))						//接收区半满,表示至少存了8个字
			{
				for(count=0;count<8;count++)			//循环读取数据
				{
					*(tempbuff+count)=SDIO->FIFO;
				}
				tempbuff+=8;	 
				timeout=0X7FFFF; 
			}else 	//处理超时
			{
				if(timeout==0)return SD_DATA_TIMEOUT;
				timeout--;
			}
		} 
		if(SDIO->STA&(1<<3))		//数据超时错误
		{										   
	 		SDIO->ICR|=1<<3; 		//清错误标志
			return SD_DATA_TIMEOUT;
	 	}else if(SDIO->STA&(1<<1))	//数据块CRC错误
		{
	 		SDIO->ICR|=1<<1; 		//清错误标志
			return SD_DATA_CRC_FAIL;		   
		}else if(SDIO->STA&(1<<5)) 	//接收fifo上溢错误
		{
	 		SDIO->ICR|=1<<5; 		//清错误标志
			return SD_RX_OVERRUN;		 
		}else if(SDIO->STA&(1<<9)) 	//接收起始位错误
		{
	 		SDIO->ICR|=1<<9; 		//清错误标志
			return SD_START_BIT_ERR;		 
		}   
		while(SDIO->STA&(1<<21))	//FIFO里面,还存在可用数据
		{
			*tempbuff=SDIO->FIFO;	//循环读取数据
			tempbuff++;
		}
		INTX_ENABLE();//开启总中断
		SDIO->ICR=0X5FF;	 		//清除所有标记
	}else if(DeviceMode==SD_DMA_MODE)
	{
 		TransferError=SD_OK;
		StopCondition=0;			//单块读,不需要发送停止传输指令
		TransferEnd=0;				//传输结束标置位，在中断服务置1
		SDIO->MASK|=(1<<1)|(1<<3)|(1<<8)|(1<<5)|(1<<9);	//配置需要的中断 
	 	SDIO->DCTRL|=1<<3;		 	//SDIO DMA使能 
 	    SD_DMA_Config((u32*)buf,blksize,DMA_DIR_PeripheralToMemory); 
 		while(((DMA2->LISR&(1<<27))==RESET)&&(TransferEnd==0)&&(TransferError==SD_OK)&&timeout)timeout--;//等待传输完成 
		if(timeout==0)return SD_DATA_TIMEOUT;//超时
		if(TransferError!=SD_OK)errorstatus=TransferError;  
    }   
 	return errorstatus; 
}
//SD卡读取多个块 
//buf:读数据缓存区
//addr:读取地址
//blksize:块大小
//nblks:要读取的块数
//返回值:错误状态
__align(4) u32 *tempbuff;
SD_Error SD_ReadMultiBlocks(u8 *buf,long long addr,u16 blksize,u32 nblks)
{
  	SD_Error errorstatus=SD_OK;
	u8 power;
   	u32 count=0;
	u32 timeout=SDIO_DATATIMEOUT;  
	tempbuff=(u32*)buf;//转换为u32指针
	
    SDIO->DCTRL=0x0;		//数据控制寄存器清零(关DMA)   
	if(CardType==SDIO_HIGH_CAPACITY_SD_CARD)//大容量卡
	{
		blksize=512;
		addr>>=9;
	}  

	SDIO_DataInitStructure.SDIO_DataBlockSize= 0; ;//清除DPSM状态机配置
	SDIO_DataInitStructure.SDIO_DataLength= 0 ;
	SDIO_DataInitStructure.SDIO_DataTimeOut=SD_DATATIMEOUT ;
	SDIO_DataInitStructure.SDIO_DPSM=SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir=SDIO_TransferDir_ToCard;
	SDIO_DataInitStructure.SDIO_TransferMode=SDIO_TransferMode_Block;
	SDIO_DataConfig(&SDIO_DataInitStructure);
	
	
	if(SDIO->RESP1&SD_CARD_LOCKED)return SD_LOCK_UNLOCK_FAILED;//卡锁了
	if((blksize>0)&&(blksize<=2048)&&((blksize&(blksize-1))==0))
	{
		power=convert_from_bytes_to_power_of_two(blksize);	    

		SDIO_CmdInitStructure.SDIO_Argument =  blksize;//发送CMD16+设置数据长度为blksize,短响应 
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);		
		
		errorstatus=CmdResp1Error(SD_CMD_SET_BLOCKLEN);	//等待R1响应   
		if(errorstatus!=SD_OK)return errorstatus;   	//响应错误	 
	}else return SD_INVALID_PARAMETER;	  
	if(nblks>1)											//多块读  
	{									    
 	  	if(nblks*blksize>SD_MAX_DATA_LENGTH)return SD_INVALID_PARAMETER;//判断是否超过最大接收长度
	 
		SDIO_DataInitStructure.SDIO_DataBlockSize= power<<4; ;//nblks*blksize,512块大小,卡到控制器
		SDIO_DataInitStructure.SDIO_DataLength= nblks*blksize ;
		SDIO_DataInitStructure.SDIO_DataTimeOut=SD_DATATIMEOUT ;
		SDIO_DataInitStructure.SDIO_DPSM=SDIO_DPSM_Enable;
		SDIO_DataInitStructure.SDIO_TransferDir=SDIO_TransferDir_ToSDIO;
		SDIO_DataInitStructure.SDIO_TransferMode=SDIO_TransferMode_Block;
		SDIO_DataConfig(&SDIO_DataInitStructure);
		
		

		SDIO_CmdInitStructure.SDIO_Argument =  addr;//发送CMD18+从addr地址出读取数据,短响应 
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_READ_MULT_BLOCK;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);		
		
		errorstatus=CmdResp1Error(SD_CMD_READ_MULT_BLOCK);//等待R1响应   
		if(errorstatus!=SD_OK)return errorstatus;   	//响应错误	  
 		if(DeviceMode==SD_POLLING_MODE)
		{
			INTX_DISABLE();//关闭总中断(POLLING模式,严禁中断打断SDIO读写操作!!!)
			while(!(SDIO->STA&((1<<5)|(1<<1)|(1<<3)|(1<<8)|(1<<9))))//无上溢/CRC/超时/完成(标志)/起始位错误
			{
				if(SDIO->STA&(1<<15))						//接收区半满,表示至少存了8个字
				{
					for(count=0;count<8;count++)			//循环读取数据
					{
						*(tempbuff+count)=SDIO->FIFO;
					}
					tempbuff+=8;	 
					timeout=0X7FFFF; 
				}else 	//处理超时
				{
					if(timeout==0)return SD_DATA_TIMEOUT;
					timeout--;
				}
			}  
			if(SDIO->STA&(1<<3))		//数据超时错误
			{										   
		 		SDIO->ICR|=1<<3; 		//清错误标志
				return SD_DATA_TIMEOUT;
		 	}else if(SDIO->STA&(1<<1))	//数据块CRC错误
			{
		 		SDIO->ICR|=1<<1; 		//清错误标志
				return SD_DATA_CRC_FAIL;		   
			}else if(SDIO->STA&(1<<5)) 	//接收fifo上溢错误
			{
		 		SDIO->ICR|=1<<5; 		//清错误标志
				return SD_RX_OVERRUN;		 
			}else if(SDIO->STA&(1<<9)) 	//接收起始位错误
			{
		 		SDIO->ICR|=1<<9; 		//清错误标志
				return SD_START_BIT_ERR;		 
			}   
			while(SDIO->STA&(1<<21))	//FIFO里面,还存在可用数据
			{
				*tempbuff=SDIO->FIFO;	//循环读取数据
				tempbuff++;
			}
	 		if(SDIO->STA&(1<<8))		//接收结束
			{
				if((SDIO_STD_CAPACITY_SD_CARD_V1_1==CardType)||(SDIO_STD_CAPACITY_SD_CARD_V2_0==CardType)||(SDIO_HIGH_CAPACITY_SD_CARD==CardType))
				{
  
				SDIO_CmdInitStructure.SDIO_Argument =  0;//发送CMD12+结束传输
				SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_STOP_TRANSMISSION;
				SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
				SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
				SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
				SDIO_SendCommand(&SDIO_CmdInitStructure);		

					errorstatus=CmdResp1Error(SD_CMD_STOP_TRANSMISSION);//等待R1响应   
					if(errorstatus!=SD_OK)return errorstatus;	 
				}
 			}
			INTX_ENABLE();//开启总中断
	 		SDIO->ICR=0X5FF;	 		//清除所有标记 
 		}else if(DeviceMode==SD_DMA_MODE)
		{
	   		TransferError=SD_OK;
			StopCondition=1;			//多块读,需要发送停止传输指令 
			TransferEnd=0;				//传输结束标置位，在中断服务置1
			SDIO->MASK|=(1<<1)|(1<<3)|(1<<8)|(1<<5)|(1<<9);	//配置需要的中断 
		 	SDIO->DCTRL|=1<<3;		 						//SDIO DMA使能 
	 	    SD_DMA_Config((u32*)buf,nblks*blksize,DMA_DIR_PeripheralToMemory); 
	 		while(((DMA2->LISR&(1<<27))==RESET)&&timeout)timeout--;//等待传输完成 
			if(timeout==0)return SD_DATA_TIMEOUT;//超时
			while((TransferEnd==0)&&(TransferError==SD_OK)); 
			if(TransferError!=SD_OK)errorstatus=TransferError;  	 
		}		 
  	}
	return errorstatus;
}			    																  
//SD卡写1个块 
//buf:数据缓存区
//addr:写地址
//blksize:块大小	  
//返回值:错误状态
SD_Error SD_WriteBlock(u8 *buf,long long addr,  u16 blksize)
{
	SD_Error errorstatus = SD_OK;
	u8  power=0,cardstate=0;
	u32 timeout=0,bytestransferred=0;
	u32 cardstatus=0,count=0,restwords=0;
	u32	tlen=blksize;						//总长度(字节)
	u32*tempbuff=(u32*)buf;								 
 	if(buf==NULL)return SD_INVALID_PARAMETER;//参数错误   
  	SDIO->DCTRL=0x0;							//数据控制寄存器清零(关DMA)   


	SDIO_DataInitStructure.SDIO_DataBlockSize= 0; ;//清除DPSM状态机配置
	SDIO_DataInitStructure.SDIO_DataLength= 0 ;
	SDIO_DataInitStructure.SDIO_DataTimeOut=SD_DATATIMEOUT ;
	SDIO_DataInitStructure.SDIO_DPSM=SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir=SDIO_TransferDir_ToCard;
	SDIO_DataInitStructure.SDIO_TransferMode=SDIO_TransferMode_Block;
  SDIO_DataConfig(&SDIO_DataInitStructure);
		
	
	
	if(SDIO->RESP1&SD_CARD_LOCKED)return SD_LOCK_UNLOCK_FAILED;//卡锁了
 	if(CardType==SDIO_HIGH_CAPACITY_SD_CARD)	//大容量卡
	{
		blksize=512;
		addr>>=9;
	}    
	if((blksize>0)&&(blksize<=2048)&&((blksize&(blksize-1))==0))
	{
		power=convert_from_bytes_to_power_of_two(blksize);	    

		SDIO_CmdInitStructure.SDIO_Argument = blksize;//发送CMD16+设置数据长度为blksize,短响应 	
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);		

		
		errorstatus=CmdResp1Error(SD_CMD_SET_BLOCKLEN);	//等待R1响应   
		if(errorstatus!=SD_OK)return errorstatus;   	//响应错误	 
	}else return SD_INVALID_PARAMETER;	 
  	

	SDIO_CmdInitStructure.SDIO_Argument = (u32)RCA<<16;//发送CMD13,查询卡的状态,短响应 	
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_STATUS;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);	

	
	errorstatus=CmdResp1Error(SD_CMD_SEND_STATUS);		//等待R1响应   		   
	if(errorstatus!=SD_OK)return errorstatus;
	cardstatus=SDIO->RESP1;													  
	timeout=SD_DATATIMEOUT;
   	while(((cardstatus&0x00000100)==0)&&(timeout>0)) 	//检查READY_FOR_DATA位是否置位
	{
		timeout--;
 
		SDIO_CmdInitStructure.SDIO_Argument = (u32)RCA<<16;//发送CMD13,查询卡的状态,短响应
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_STATUS;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);	

		
		errorstatus=CmdResp1Error(SD_CMD_SEND_STATUS);	//等待R1响应   		   
		if(errorstatus!=SD_OK)return errorstatus;				    
		cardstatus=SDIO->RESP1;													  
	}
	if(timeout==0)return SD_ERROR;
 
	SDIO_CmdInitStructure.SDIO_Argument = addr;//发送CMD13,查询卡的状态,短响应
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_WRITE_SINGLE_BLOCK;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);	
	
	errorstatus=CmdResp1Error(SD_CMD_WRITE_SINGLE_BLOCK);//等待R1响应   		   
	if(errorstatus!=SD_OK)return errorstatus;   	  
	StopCondition=0;									//单块写,不需要发送停止传输指令 
 
		SDIO_DataInitStructure.SDIO_DataBlockSize= power<<4; ;	//blksize, 控制器到卡	
	SDIO_DataInitStructure.SDIO_DataLength= blksize ;
	SDIO_DataInitStructure.SDIO_DataTimeOut=SD_DATATIMEOUT ;
	SDIO_DataInitStructure.SDIO_DPSM=SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir=SDIO_TransferDir_ToCard;
	SDIO_DataInitStructure.SDIO_TransferMode=SDIO_TransferMode_Block;
  SDIO_DataConfig(&SDIO_DataInitStructure);
	
	
	timeout=SDIO_DATATIMEOUT;
	if (DeviceMode == SD_POLLING_MODE)
	{
		INTX_DISABLE();//关闭总中断(POLLING模式,严禁中断打断SDIO读写操作!!!)
		while(!(SDIO->STA&((1<<10)|(1<<4)|(1<<1)|(1<<3)|(1<<9))))//数据块发送成功/下溢/CRC/超时/起始位错误
		{
			if(SDIO->STA&(1<<14))							//发送区半空,表示至少存了8个字
			{
				if((tlen-bytestransferred)<SD_HALFFIFOBYTES)//不够32字节了
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
		if(SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)		//数据超时错误
		{										   
      SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
			return SD_DATA_TIMEOUT;
	 	}else if(SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)	//数据块CRC错误
		{
      SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
			return SD_DATA_CRC_FAIL;		   
		}else if(SDIO_GetFlagStatus(SDIO_FLAG_TXUNDERR) != RESET) 	//接收fifo下溢错误
		{
      SDIO_ClearFlag(SDIO_FLAG_TXUNDERR);
			return SD_TX_UNDERRUN;		 
		}else if(SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET) 	//接收起始位错误
		{
	 	  SDIO_ClearFlag(SDIO_FLAG_STBITERR);
			return SD_START_BIT_ERR;		 
		}   
		INTX_ENABLE();//开启总中断
		SDIO_ClearFlag(0X5FF);//清除所有标记
	}else if(DeviceMode==SD_DMA_MODE)
	{
   		TransferError=SD_OK;
		StopCondition=0;			//单块写,不需要发送停止传输指令 
		TransferEnd=0;				//传输结束标置位，在中断服务置1
	
		SDIO_ITConfig(SDIO_IT_DCRCFAIL|SDIO_IT_DTIMEOUT|SDIO_IT_DATAEND|SDIO_IT_STBITERR|SDIO_IT_TXUNDERR,ENABLE);//配置产生数据接收完成中断
		
		SD_DMA_Config((u32*)buf,blksize,DMA_DIR_MemoryToPeripheral);				//SDIO DMA配置							.
    SDIO_DMACmd(ENABLE);	//SDIO DMA使能	
 		while(((DMA2->LISR&(1<<27))==RESET)&&timeout)timeout--;//等待传输完成 
		if(timeout==0)
		{
  			SD_Init();	 					//重新初始化SD卡,可以解决写入死机的问题
			return SD_DATA_TIMEOUT;			//超时	 
 		}
		timeout=SDIO_DATATIMEOUT;
		while((TransferEnd==0)&&(TransferError==SD_OK)&&timeout)timeout--;
 		if(timeout==0)return SD_DATA_TIMEOUT;			//超时	 
  		if(TransferError!=SD_OK)return TransferError;
 	}  
	SDIO_ClearFlag(0X5FF);//清除所有标记
 	errorstatus=IsCardProgramming(&cardstate);
 	while((errorstatus==SD_OK)&&((cardstate==SD_CARD_PROGRAMMING)||(cardstate==SD_CARD_RECEIVING)))
	{
		errorstatus=IsCardProgramming(&cardstate);
	}   
	return errorstatus;
}

//SD卡写多个块 
//buf:数据缓存区
//addr:写地址
//blksize:块大小
//nblks:要写入的块数
//返回值:错误状态	
SD_Error SD_WriteMultiBlocks(u8 *buf,long long addr,u16 blksize,u32 nblks)
{
	SD_Error errorstatus = SD_OK;
	u8  power = 0, cardstate = 0;
	u32 timeout=0,bytestransferred=0;
	u32 count = 0, restwords = 0;
	u32 tlen=nblks*blksize;				//总长度(字节)
	u32 *tempbuff = (u32*)buf;  
  	if(buf==NULL)return SD_INVALID_PARAMETER; //参数错误  
  	SDIO->DCTRL=0x0;							//数据控制寄存器清零(关DMA)   

	SDIO_DataInitStructure.SDIO_DataBlockSize= 0; ;	//清除DPSM状态机配置	
	SDIO_DataInitStructure.SDIO_DataLength= 0 ;
	SDIO_DataInitStructure.SDIO_DataTimeOut=SD_DATATIMEOUT ;
	SDIO_DataInitStructure.SDIO_DPSM=SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir=SDIO_TransferDir_ToCard;
	SDIO_DataInitStructure.SDIO_TransferMode=SDIO_TransferMode_Block;
	SDIO_DataConfig(&SDIO_DataInitStructure);
	
	
	if(SDIO->RESP1&SD_CARD_LOCKED)return SD_LOCK_UNLOCK_FAILED;//卡锁了
 	if(CardType==SDIO_HIGH_CAPACITY_SD_CARD)//大容量卡
	{
		blksize=512;
		addr>>=9;
	}    
	if((blksize>0)&&(blksize<=2048)&&((blksize&(blksize-1))==0))
	{
		power=convert_from_bytes_to_power_of_two(blksize);	    
 
		SDIO_CmdInitStructure.SDIO_Argument = blksize;	//发送CMD16+设置数据长度为blksize,短响应
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);	

		 errorstatus=CmdResp1Error(SD_CMD_SET_BLOCKLEN);	//等待R1响应   
		if(errorstatus!=SD_OK)return errorstatus;   	//响应错误	 
	}else return SD_INVALID_PARAMETER;	 
	if(nblks>1)
	{					  
		if(nblks*blksize>SD_MAX_DATA_LENGTH)return SD_INVALID_PARAMETER;   
     	if((SDIO_STD_CAPACITY_SD_CARD_V1_1==CardType)||(SDIO_STD_CAPACITY_SD_CARD_V2_0==CardType)||(SDIO_HIGH_CAPACITY_SD_CARD==CardType))
    	{
 
				SDIO_CmdInitStructure.SDIO_Argument = (u32)RCA<<16;		//发送ACMD55,短响应 	
				SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
				SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
				SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
				SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
				SDIO_SendCommand(&SDIO_CmdInitStructure);	
				
			errorstatus=CmdResp1Error(SD_CMD_APP_CMD);		//等待R1响应   		   
			if(errorstatus!=SD_OK)return errorstatus;				    
 
				SDIO_CmdInitStructure.SDIO_Argument =nblks;		//发送CMD23,设置块数量,短响应 	 
				SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCK_COUNT;
				SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
				SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
				SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
				SDIO_SendCommand(&SDIO_CmdInitStructure);	
				
			errorstatus=CmdResp1Error(SD_CMD_SET_BLOCK_COUNT);//等待R1响应   		   
			if(errorstatus!=SD_OK)return errorstatus;				    
		} 
 
				SDIO_CmdInitStructure.SDIO_Argument =addr;	//发送CMD25,多块写指令,短响应 	  
				SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_WRITE_MULT_BLOCK;
				SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
				SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
				SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
				SDIO_SendCommand(&SDIO_CmdInitStructure);	
		
		errorstatus=CmdResp1Error(SD_CMD_WRITE_MULT_BLOCK);	//等待R1响应   		   
		if(errorstatus!=SD_OK)return errorstatus;

				SDIO_DataInitStructure.SDIO_DataBlockSize= power<<4; ;	//清除DPSM状态机配置	
				SDIO_DataInitStructure.SDIO_DataLength= nblks*blksize ;
				SDIO_DataInitStructure.SDIO_DataTimeOut=SD_DATATIMEOUT ;
				SDIO_DataInitStructure.SDIO_DPSM=SDIO_DPSM_Enable;
				SDIO_DataInitStructure.SDIO_TransferDir=SDIO_TransferDir_ToCard;
				SDIO_DataInitStructure.SDIO_TransferMode=SDIO_TransferMode_Block;
				SDIO_DataConfig(&SDIO_DataInitStructure);
		
		
		if(DeviceMode==SD_POLLING_MODE)
	    {
			timeout=SDIO_DATATIMEOUT;
			INTX_DISABLE();//关闭总中断(POLLING模式,严禁中断打断SDIO读写操作!!!)
			while(!(SDIO->STA&((1<<4)|(1<<1)|(1<<8)|(1<<3)|(1<<9))))//下溢/CRC/数据结束/超时/起始位错误
			{
				if(SDIO->STA&(1<<14))							//发送区半空,表示至少存了8字(32字节)
				{	  
					if((tlen-bytestransferred)<SD_HALFFIFOBYTES)//不够32字节了
					{
						restwords=((tlen-bytestransferred)%4==0)?((tlen-bytestransferred)/4):((tlen-bytestransferred)/4+1);
						for(count=0;count<restwords;count++,tempbuff++,bytestransferred+=4)
						{
							SDIO->FIFO=*tempbuff;
						}
					}else 										//发送区半空,可以发送至少8字(32字节)数据
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
			if(SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)		//数据超时错误
			{										   
		 		 SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);	//清错误标志
				 return SD_DATA_TIMEOUT;
		 	}else if(SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)	//数据块CRC错误
			{
		 		 SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);		//清错误标志
				return SD_DATA_CRC_FAIL;		   
			}else if(SDIO_GetFlagStatus(SDIO_FLAG_TXUNDERR) != RESET) 	//接收fifo下溢错误
			{
		 		SDIO_ClearFlag(SDIO_FLAG_TXUNDERR);		//清错误标志
				return SD_TX_UNDERRUN;		 
			}else if(SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET) 	//接收起始位错误
			{
		 		SDIO_ClearFlag(SDIO_FLAG_STBITERR); 		//清错误标志
				return SD_START_BIT_ERR;		 
			}   										   
			if(SDIO_GetFlagStatus(SDIO_FLAG_DATAEND) != RESET)		//发送结束
			{															 
				if((SDIO_STD_CAPACITY_SD_CARD_V1_1==CardType)||(SDIO_STD_CAPACITY_SD_CARD_V2_0==CardType)||(SDIO_HIGH_CAPACITY_SD_CARD==CardType))
				{
 
					SDIO_CmdInitStructure.SDIO_Argument =0;//发送CMD12+结束传输 	  
					SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_STOP_TRANSMISSION;
					SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
					SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
					SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
					SDIO_SendCommand(&SDIO_CmdInitStructure);	

					
					errorstatus=CmdResp1Error(SD_CMD_STOP_TRANSMISSION);//等待R1响应   
					if(errorstatus!=SD_OK)return errorstatus;	 
				}
			}
			INTX_ENABLE();//开启总中断
	 			SDIO_ClearFlag(0X5FF);//清除所有标记 
	    }else if(DeviceMode==SD_DMA_MODE)
		{
	   		TransferError=SD_OK;
			StopCondition=1;			//多块写,需要发送停止传输指令 
			TransferEnd=0;				//传输结束标置位，在中断服务置1
			
			SDIO_ITConfig(SDIO_IT_DCRCFAIL|SDIO_IT_DTIMEOUT|SDIO_IT_DATAEND|SDIO_IT_STBITERR|SDIO_IT_TXUNDERR,ENABLE);//配置产生数据接收完成中断
			SD_DMA_Config((u32*)buf,nblks*blksize,DMA_DIR_MemoryToPeripheral);		//SDIO DMA配置
	 	 	SDIO_DMACmd(ENABLE);	//SDIO DMA使能	 
			timeout=SDIO_DATATIMEOUT;
	 		while(((DMA2->LISR&(1<<27))==RESET)&&timeout)timeout--;//等待传输完成 
			if(timeout==0)	 								//超时
			{									  
  				SD_Init();	 					//重新初始化SD卡,可以解决写入死机的问题
	 			return SD_DATA_TIMEOUT;			//超时	 
	 		}
			timeout=SDIO_DATATIMEOUT;
			while((TransferEnd==0)&&(TransferError==SD_OK)&&timeout)timeout--;
	 		if(timeout==0)return SD_DATA_TIMEOUT;			//超时	 
	 		if(TransferError!=SD_OK)return TransferError;	 
		}
  	}
 		SDIO_ClearFlag(0X5FF);//清除所有标记
 	errorstatus=IsCardProgramming(&cardstate);
 	while((errorstatus==SD_OK)&&((cardstate==SD_CARD_PROGRAMMING)||(cardstate==SD_CARD_RECEIVING)))
	{
		errorstatus=IsCardProgramming(&cardstate);
	}   
	return errorstatus;	   
}
//SDIO中断服务函数		  
void SDIO_IRQHandler(void) 
{											
 	SD_ProcessIRQSrc();//处理所有SDIO相关中断
}	 																    
//SDIO中断处理函数
//处理SDIO传输过程中的各种中断事务
//返回值:错误代码
SD_Error SD_ProcessIRQSrc(void)
{
 if (StopCondition == 1)  //什么时候置1了？
  {
    SDIO->ARG = 0x0;   //命令参数寄存器
    SDIO->CMD = 0x44C;	  // 命令寄存器： 0100 	01 	 	001100
						//						[7:6]  	[5:0]
						//				CPSMEN  WAITRESP CMDINDEX
						//		开启命令状态机	短响应   cmd12 STOP_ TRANSMISSION						
    TransferError = CmdResp1Error(SD_CMD_STOP_TRANSMISSION);
  }
  else
  {
    TransferError = SD_OK;
  }
  SDIO_ClearITPendingBit(SDIO_IT_DATAEND); //清中断
  SDIO_ITConfig(SDIO_IT_DATAEND, DISABLE); //关闭sdio中断使能
  TransferEnd = 1;
  return(TransferError);
}
  
//检查CMD0的执行状态
//返回值:sd卡错误码
static SD_Error CmdError(void)
{
  SD_Error errorstatus = SD_OK;
  uint32_t timeout;

  timeout = SDIO_CMD0TIMEOUT; /*!< 10000 */

	 /*检查命令是否已发送*/
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
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);//清除静态标志位

  return(errorstatus);
}
//检查R7响应的错误状态
//返回值:sd卡错误码
static SD_Error CmdResp7Error(void)
{
	SD_Error errorstatus = SD_OK;
	uint32_t status;
	uint32_t timeout = SDIO_CMD0TIMEOUT;

	status = SDIO->STA;	//读取SDIO状态寄存器 ，此状态寄存器是stm32的寄存器
 
	/* Command response received (CRC check failed) ：Command response received (CRC check passed)：Command response timeout */
	while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)) && (timeout > 0))
	{	
		timeout--;
		status = SDIO->STA;		  
	}
	 //卡不响应cmd8 说明卡不属于V2.0卡
	if ((timeout == 0) || (status & SDIO_FLAG_CTIMEOUT))
	{
		errorstatus = SD_CMD_RSP_TIMEOUT;
		SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
		return(errorstatus);
	}

	//接收到cmd8响应,说明卡是V2.0卡
	if (status & SDIO_FLAG_CMDREND)
	{
		errorstatus = SD_OK;
		SDIO_ClearFlag(SDIO_FLAG_CMDREND);
		return(errorstatus);
	}
	return(errorstatus);
}
   
//检查R1响应的错误状态
//cmd:当前命令
//返回值:sd卡错误码
static SD_Error CmdResp1Error(uint8_t cmd) //传入的参数有什么用？
{		   
	/*不是这些状态就等待	*/
  while (!(SDIO->STA & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)))
  {
  }

  SDIO->ICR = SDIO_STATIC_FLAGS;	//清中断标志

  return (SD_Error)(SDIO->RESP1 &  SD_OCR_ERRORBITS);		//判断是否在供电范围
}
//检查R3响应的错误状态
//返回值:错误状态
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

//检查R2响应的错误状态
//返回值:错误状态
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

//检查R6响应的错误状态
//cmd:之前发送的命令
//prca:卡返回的RCA地址
//返回值:错误状态
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
  if (SDIO_GetCommandResponse() != cmd)		 //检测是否接收到正常命令
  {
    errorstatus = SD_ILLEGAL_CMD;
    return(errorstatus);
  }

  /*!< Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  /*!< We have received response, retrieve it.  */
  response_r1 = SDIO_GetResponse(SDIO_RESP1);

	/*以下状态全为0表明成功接收到card返回的rca */
  if (SD_ALLZERO == (response_r1 & (SD_R6_GENERAL_UNKNOWN_ERROR | SD_R6_ILLEGAL_CMD | SD_R6_COM_CRC_FAILED)))
  {
    *prca = (uint16_t) (response_r1 >> 16);//右移16位，就是接收到的返回rca
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


//SDIO使能宽总线模式
//返回值:错误状态
static SD_Error SDEnWideBus(FunctionalState NewState)
{
  SD_Error errorstatus = SD_OK;

  uint32_t scr[2] = {0, 0};

  if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED) //检测卡是否已上锁
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }

  /*!< Get SCR Register */
  errorstatus = FindSCR(RCA, scr);//获取scr寄存器内容到scr数组中

  if (errorstatus != SD_OK)		  //degug,crc错误，scr读取不了数值
  {
    return(errorstatus);
  }

  /*!< If wide bus operation to be enabled */
  if (NewState == ENABLE)
  {
    /*!< If requested card supports wide bus operation */
    if ((scr[1] & SD_WIDE_BUS_SUPPORT) != SD_ALLZERO)  //判断卡是否支持4位方式
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
	  /*开启4bit模式的命令acmd6*/
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
//检查卡是否正在执行写操作
//pstatus:当前状态.
//返回值:错误代码
SD_Error IsCardProgramming(u8 *pstatus)
{
  SD_Error errorstatus = SD_OK;
  __IO uint32_t respR1 = 0, status = 0;

	 /*cmd13让卡发送卡状态寄存器，存储到m3的位置为sdio_sta寄存器*/
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16; //卡相对地址参数
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
	  /*一系列的状态判断*/
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
//查找SD卡的SCR寄存器值
//rca:卡相对地址
//pscr:数据缓存区(存储SCR内容)
//返回值:错误状态		   
SD_Error FindSCR(u16 rca,u32 *pscr)
{ 

  uint32_t index = 0;
  SD_Error errorstatus = SD_OK;
  uint32_t tempscr[2] = {0, 0};	

  /*!< Set Block Size To 8 Bytes */ 
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)8;	 //块大小如果是sdhc卡是无法改变块大小的	//原参数8
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
  /*设置数据接收寄存器*/
  SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
  SDIO_DataInitStructure.SDIO_DataLength = 8;  //8byte,64位
  SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_8b  ;  //块大小8byte 
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
   
  /*等待接收数据 */
  /*不是这些情况就循环*/																						  
 /*上溢出错	  //数据crc失败		//数据超时	  //已接收数据块，crc检测成功	//没有在所有数据线上检测到起始信号*/
 while (!(SDIO->STA & (SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DBCKEND| SDIO_FLAG_STBITERR)))
   {	   		
   if (SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET)	//接收到的数据是否可用
	{  	
     	 	*(tempscr + index) = SDIO_ReadData();  
			   index++;	

		/*   //add。这段在官方源码没有加判断            */		     
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
//得到NumberOfBytes以2为底的指数.
//NumberOfBytes:字节数.
//返回值:以2为底的指数值
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



//配置SDIO DMA  
//mbuf:存储器地址
//bufsize:传输数据量
//dir:方向;DMA_DIR_MemoryToPeripheral  存储器-->SDIO(写数据);DMA_DIR_PeripheralToMemory SDIO-->存储器(读数据);
void SD_DMA_Config(u32*mbuf,u32 bufsize,u32 dir)
{		 

  DMA_InitTypeDef  DMA_InitStructure;
	
	while (DMA_GetCmdStatus(DMA2_Stream3) != DISABLE){}//等待DMA可配置 
		
  DMA_DeInit(DMA2_Stream3);//清空之前该stream3上的所有中断标志
	
 
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SDIO->FIFO;//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)mbuf;//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = dir;//存储器到外设模式
  DMA_InitStructure.DMA_BufferSize = 0;//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;//外设数据长度:32位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;//存储器数据长度:32位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//最高优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;   //FIFO使能      
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;//全FIFO
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;//外设突发4次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;//存储器突发4次传输
  DMA_Init(DMA2_Stream3, &DMA_InitStructure);//初始化DMA Stream

	DMA_FlowControllerConfig(DMA2_Stream3,DMA_FlowCtrl_Peripheral);//外设流控制 
	 
  DMA_Cmd(DMA2_Stream3 ,ENABLE);//开启DMA传输	 

}   
//读SD卡
//buf:读数据缓存区
//sector:扇区地址
//cnt:扇区个数	
//返回值:错误状态;0,正常;其他,错误代码;				  				 
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
		 	sta=SD_ReadBlock(SDIO_DATA_BUFFER,lsector+512*n,512);//单个sector的读操作
			memcpy(buf,SDIO_DATA_BUFFER,512);
			buf+=512;
		} 
	}else
	{
		if(cnt==1)sta=SD_ReadBlock(buf,lsector,512);    	//单个sector的读操作
		else sta=SD_ReadMultiBlocks(buf,lsector,512,cnt);//多个sector  
	}
	return sta;
}
//写SD卡
//buf:写数据缓存区
//sector:扇区地址
//cnt:扇区个数	
//返回值:错误状态;0,正常;其他,错误代码;	
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
		 	sta=SD_WriteBlock(SDIO_DATA_BUFFER,lsector+512*n,512);//单个sector的写操作
			buf+=512;
		} 
	}else
	{
		if(cnt==1)sta=SD_WriteBlock(buf,lsector,512);    	//单个sector的写操作
		else sta=SD_WriteMultiBlocks(buf,lsector,512,cnt);	//多个sector  
	}
	return sta;
}







