/****************************************************************
* FILENAME: spi.c
*
* BRIEF: Contains functions to initialize SPI interface using
* USCI_B0 and communicate with the TRF796x via this interface.
*
* Copyright (C) 2010 Texas Instruments, Inc.
*
* AUTHOR(S): Reiser Peter		DATE: 02 DEC 2010
*
* CHANGES:
* REV.	DATE		WHO	DETAIL
* 00	02Dec2010	RP	Orginal Version
* 01	07Dec2010	RP	Changed SPI clock frequency from 6.78 MHz
* 						to 1.70 MHz in SpiUsciExtClkSet() and
* 						also reduced frequency in SpiUsciSet()
* 01	07Dec2010	RP	integrated wait while busy loops in
* 						spi-communication
*
****************************************************************/

#include "spi.h"
#include "trf797x.h"
#include "hardware.h"

//===============================================================

u08_t	temp = 0;
extern u08_t direct_mode;

//===============================================================

void SpiStartCondition(void);
void SpiUsciSet(void);

//===============================================================
// NAME: void SpiDirectCommand (u08_t *pbuf)
//
// BRIEF: Is used in SPI mode to transmit a Direct Command to
// reader chip.
//
// INPUTS:
//	Parameters:
//		u08_t		*pbuf		Direct Command
//
// OUTPUTS:
//
// PROCESS:	[1] transmit Direct Command
//
// CHANGE:
// DATE  		WHO	DETAIL
// 24Nov2010	RP	Original Code
// 07Dec2010	RP	integrated wait while busy loops
//===============================================================
/************************************************
函数名称 ： SPI_GPIO_Configuration
功    能 ： SPI引脚配置
参    数 ： 无
返 回 值 ： 无
作    者 ： strongerHuang
*************************************************/
void SPI_GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* SPI2: SCK\MOSI\MISO复用功能*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* SPI2: NSS复用功能 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/************************************************
函数名称 ： SPI_Configuration
功    能 ： SPI配置
参    数 ： 无
返 回 值 ： 无
作    者 ： strongerHuang
*************************************************/
void SPI_Configuration(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB| RCC_APB2Periph_GPIOC ,ENABLE);//使能SPI  GPIOA时钟
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2 , ENABLE );
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	SLAVE_SELECT_HIGH;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//时钟悬空低
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//数据捕获开始于第一个时钟沿
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;

	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2,&SPI_InitStructure);

	SPI_Cmd(SPI2,ENABLE);
}

/************************************************
函数名称 ： SPI_Initializes
功    能 ： SPI初始化
参    数 ： 无
返 回 值 ： 无
作    者 ： strongerHuang
*************************************************/
void SPI_Initializes(void)
{
  //SPI_GPIO_Configuration();
  SPI_Configuration();
}
void TRF7970_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOB,&GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
		GPIO_Init(GPIOB,&GPIO_InitStructure);

		GPIO_ResetBits(GPIOB,GPIO_Pin_8);
		GPIO_ResetBits(GPIOB,GPIO_Pin_9);

}
/************************************************
函数名称 ： SPI_WriteReadByte
功    能 ： 对SPI写读字节数据
参    数 ： TxData --- 发送的字节数据
返 回 值 ： 读回来的字节数据
作    者 ： strongerHuang
*************************************************/
void SpiDirectCommand(u8 *pbuf)
{	
  GPIO_ResetBits(GPIOB,GPIO_Pin_12);						// Start SPI Mode
  // set Address/Command Word Bit Distribution to command
  *pbuf = (0x80 | *pbuf);					// command
  *pbuf = (0x9f &*pbuf);					// command code
  
  /* Wait for SPI1 Tx buffer empty */
  while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET)
  {
  }  
  /* Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(SPI2,*pbuf);	  
  
  /* Wait for SPI1 data reception	*/
  while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE) == RESET)
  {
  }  
  /* Read & return SPI1 received data	*/
  temp=SPI_I2S_ReceiveData(SPI2);
  
  while (SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_BSY)==SET)
  {
  }

  
   GPIO_SetBits(GPIOB,GPIO_Pin_12);	 						//Stop SPI Mode
  
  //P3SEL |=  BIT3;      					//Revert Back
}

//===============================================================
// NAME: void SpiDirectMode (void)
//
// BRIEF: Is used in SPI mode to start Direct Mode.
//
// INPUTS:
//
// OUTPUTS:
//
// PROCESS:	[1] start Direct Mode
//
// NOTE: No stop condition
//
// CHANGE:
// DATE  		WHO	DETAIL
// 24Nov2010	RP	Original Code
// 07Dec2010	RP	integrated wait while busy loops
//===============================================================

void
SpiDirectMode(void)
{		
  u08_t command [2];
  
  command[0] = CHIP_STATE_CONTROL;
  command[1] = CHIP_STATE_CONTROL;
  SpiReadSingle(&command[1],1);
  command[1] |= 0x60;						// RF on and BIT 6 in Chip Status Control Register set
  SpiWriteSingle(command, 2);
}  	

//===============================================================
// NAME: void SpiRawWrite (u08_t *pbuf, u08_t length)
//
// BRIEF: Is used in SPI mode to write direct to the reader chip.
//
// INPUTS:
//	Parameters:
//		u08_t		*pbuf		raw data
//		u08_t		length		number of data bytes
//
// OUTPUTS:
//
// PROCESS:	[1] send raw data to reader chip
//
// CHANGE:
// DATE  		WHO	DETAIL
// 24Nov2010	RP	Original Code
// 07Dec2010	RP	integrated wait while busy loops
//===============================================================

void 
SpiRawWrite(u08_t *pbuf, u08_t length)
{	
  SLAVE_SELECT_LOW; 						//Start SPI Mode
  while(length > 0)
  {	
    //                while (!(IFG2 & UCB0TXIFG));		// USCI_B0 TX buffer ready?
    //		{	
    //		}
    //
    //		UCB0TXBUF = *pbuf;				// Previous data to TX, RX
    //
    //		while(UCB0STAT & UCBUSY)
    //		{
    //		}
    //
    //		temp=UCB0RXBUF;
    
    /* Wait for SPI2 Tx buffer empty */
    //while(SPI_GetFlagStatus(SPI_I2S_FLAG_TXE) == RESET)
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)//SPI_I2S_GetFlagStatus
    {
    }  
    /* Send byte through the SPI2 peripheral */
    //SPI_SendData(*pbuf);	
    SPI_I2S_SendData(SPI2, *pbuf);		
    
    /* Wait for SPI2 data reception	*/
    //while(SPI_GetFlagStatus(SPI_I2S_FLAG_TXE) == RESET)
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)//SPI_I2S_GetFlagStatus
    {
    }  
    /* Read & return SPI2 received data	*/
    temp=SPI_I2S_ReceiveData(SPI2);
    
    pbuf++;
    length--;
  }
  //	while(UCB0STAT & UCBUSY)
  //	{
  //	}
  
	//while (SPI_GetFlagStatus(SPI_I2S_FLAG_TXE)==SET)
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET)//SPI_I2S_GetFlagStatus
  {
  }
  SLAVE_SELECT_HIGH; 						// Stop SPI Mode		
}

//===============================================================
// NAME: void SpiReadCont (u08_t *pbuf, u08_t length)
//
// BRIEF: Is used in SPI mode to read a specified number of
// reader chip registers from a specified address upwards.
//
// INPUTS:
//	Parameters:
//		u08_t		*pbuf		address of first register
//		u08_t		length		number of registers
//
// OUTPUTS:
//
// PROCESS:	[1] read registers
//			[2] write contents to *pbuf
//
// CHANGE:
// DATE  		WHO	DETAIL
// 24Nov2010	RP	Original Code
// 07Dec2010	RP	integrated wait while busy loops
//===============================================================

void
SpiReadCont(u08_t *pbuf, u08_t length)
{	
  //u08_t	j = 0;
  
  SLAVE_SELECT_LOW; 							//Start SPI Mode
  // Address/Command Word Bit Distribution
  *pbuf = (0x60 | *pbuf); 					// address, read, continuous
  *pbuf = (0x7f &*pbuf);						// register address
//  while (!(IFG2 & UCB0TXIFG))					// USCI_B0 TX buffer ready?
//  {
//  }
//  UCB0TXBUF = *pbuf;  						// Previous data to TX, RX
//  
//  while(UCB0STAT & UCBUSY)
//  {
//  }
//  
//  temp = UCB0RXBUF;
  
  /* Wait for SPI2 Tx buffer empty */
    //while(SPI_GetFlagStatus(SPI_I2S_FLAG_TXE) == RESET)
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)//SPI_I2S_GetFlagStatus
    {
    }  
    /* Send byte through the SPI2 peripheral */
    //SPI_SendData(*pbuf);
    SPI_I2S_SendData(SPI2, *pbuf); 
    
    /* Wait for SPI2 data reception	*/
    //while(SPI_GetFlagStatus(SPI_I2S_FLAG_TXE) == RESET)
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)//SPI_I2S_GetFlagStatus
    {
    }  
    /* Read & return SPI2 received data	*/
    temp=SPI_I2S_ReceiveData(SPI2);
  //UCB0CTL0 &= ~UCCKPH;
  
  /*_NOP();
  _NOP();
  _NOP();
  _NOP();
  _NOP();
  _NOP();
  _NOP();
  
  if(*pbuf != 0x6C)							// execute only when IRQRead is not called
  {	
  if (length != 0x1F)
  {	
  for (j=0;j<2;j++)
  {	
  while (!(IFG2 & UCB0TXIFG))		// USCI_B0 TX buffer ready?
  {
}
  UCB0TXBUF = 0x00; 				// Receive initiated by a dummy TX write
  
  while(UCB0STAT & UCBUSY)
  {
}
  _NOP();
  _NOP();
  temp = UCB0RXBUF;
}
}
}*/
  while(length > 0)
  {	
//    while (!(IFG2 & UCB0TXIFG))
//    {
//    }
//    UCB0TXBUF = 0x00; 					// Receive initiated by a dummy TX write
//    
//    while(UCB0STAT & UCBUSY)
//      //while (!(IFG2 & UCB0RXIFG));
//    {
//    }
    
    /* Wait for SPI2 Tx buffer empty */
    //while(SPI_GetFlagStatus(SPI_I2S_FLAG_TXE) == RESET)
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)//SPI_I2S_GetFlagStatus
    {
    }  
    /* Send byte through the SPI2 peripheral */
    //SPI_SendData(0x00);	
    SPI_I2S_SendData(SPI2, 0x00);		
    
    /* Wait for SPI2 data reception	*/
    //while(SPI_GetFlagStatus(SPI_I2S_FLAG_TXE) == RESET)
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)//SPI_I2S_GetFlagStatus
    {
    }  
    
    
    //_NOP();
    //_NOP();
    //_NOP();
    //_NOP();
    //_NOP();
    //_NOP();
    //_NOP();
    //_NOP();
    
//    *pbuf = UCB0RXBUF;
    /* Read & return SPI2 received data	*/
    *pbuf=SPI_I2S_ReceiveData(SPI2);
    pbuf++;
    length--;
  }
  //UCB0CTL0 |= UCCKPH;
//  while(UCB0STAT & UCBUSY)
//  {
//  }
  //while (SPI_GetFlagStatus(SPI_I2S_FLAG_TXE)==SET)
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET)//SPI_I2S_GetFlagStatus
  {
  }
  SLAVE_SELECT_HIGH; 						// Stop SPI Mode
  
}

//===============================================================
// NAME: void SpiReadSingle (u08_t *pbuf, u08_t number)
//
// BRIEF: Is used in SPI mode to read specified reader chip
// registers.
//
// INPUTS:
//	Parameters:
//		u08_t		*pbuf		addresses of the registers
//		u08_t		number		number of the registers
//
// OUTPUTS:
//
// PROCESS:	[1] read registers
//			[2] write contents to *pbuf
//
// CHANGE:
// DATE  		WHO	DETAIL
// 24Nov2010	RP	Original Code
// 07Dec2010	RP	integrated wait while busy loops
//===============================================================
void SpiReadSingle(u8 *pbuf, u8 number)
{			
  GPIO_ResetBits(GPIOB,GPIO_Pin_12);	 						
  while(number > 0)
  {	
    // Address/Command Word Bit Distribution
    *pbuf = (0x40 | *pbuf); 			// address, read, single
    *pbuf = (0x5f & *pbuf);				// register address
    
//    while (!(IFG2 & UCB0TXIFG))			// USCI_B0 TX buffer ready?
//    {
//    }
//    UCB0TXBUF = *pbuf;					// Previous data to TX, RX
//    
//    while(UCB0STAT & UCBUSY)
//    {
//    }
//    
//    temp=UCB0RXBUF;
    
    /* Wait for SPI1 Tx buffer empty */
    while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET)
  {
  }    
    /* Send byte through the SPI1 peripheral */
    SPI_I2S_SendData(SPI2,*pbuf);	   
    
    /* Wait for SPI1 data reception	*/
  while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE) == RESET)
  {
  }   
    /* Read & return SPI1 received data	*/
    temp=SPI_I2S_ReceiveData(SPI2);
    
    
    /* Wait for SPI1 Tx buffer empty */
    while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET)
    {
    }  
    /* Send byte through the SPI1 peripheral */
    SPI_I2S_SendData(SPI2,0x00);	  
    
    /* Wait for SPI1 data reception	*/
     while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE) == RESET)
    {
    }  
		//mod_control[0]=0;
    /* Read & return SPI1 received data	*/
    *pbuf=SPI_I2S_ReceiveData(SPI2);
    pbuf++;
    number--;
  }
  while (SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_BSY)==SET)
  {
  }
//  if(direct_mode == 0x00)
  {
    GPIO_SetBits(GPIOB,GPIO_Pin_12);	 						//Stop SPI Mode
  }
}

//===============================================================
// Settings for SPI Mode                                        ;
// 02DEC2010	RP	Original Code
//===============================================================

void
SpiSetup(void)
{	
  /*TRF_WRITE = 0x00;
  TRF_DIR_OUT;
  
  TRF_WRITE = 0x16;
  
  TRF_ENABLE;
  McuDelayMillisecond(1);
  TRF_WRITE = 0x00;*/
  
  //ENABLE_SET;
  
//  IRQ_PIN_SET;
//  IRQ_EDGE_SET;								// rising edge interrupt
//  
////  SpiUsciSet();								// Set the USART
//  
//  SPI_DeInit();
//  
//  SPI_Init(SPI_FIRSTBIT_MSB, SPI_BAUDRATEPRESCALER_8, SPI_MODE_MASTER,\
//    SPI_CLOCKPOLARITY_LOW, SPI_CLOCKPHASE_2EDGE, \
//      SPI_DATADIRECTION_2LINES_FULLDUPLEX, SPI_NSS_SOFT, 0x07); //SPI INITIAL
  
//  SPI_Cmd(ENABLE);
//  
//  LED_ALL_OFF;
//  LED_PORT_SET;
}

//===============================================================
// 02DEC2010	RP	Original Code
//===============================================================

void
SpiStartCondition(void)					//Make the SCLK High
{
//  P3SEL &= ~ BIT3;
//  P3DIR |= BIT3;
//  P3OUT |= BIT3; 			//Make SCLK High
}

//===============================================================
// NAME: void SpiUsciExtClkSet (void)
//
// BRIEF: Is used to switch SPI data clock from DCO to more
// stabile extern clock
//
// INPUTS:
//
// OUTPUTS:
//
// PROCESS:	[1] switch SPI data clock
//
// CHANGE:
// DATE  		WHO	DETAIL
// 24Nov2010	RP	Original Code
// 07Dec2010	RP	changed SPI clock from 6.78MHz to 1.70MHz
//===============================================================

void
SpiUsciExtClkSet(void)	  							//Uses USCI_B0
{
//  UCB0CTL1 |= UCSWRST;                     		// Enable SW reset
//  UCB0CTL0 |= UCCKPH + UCMSB + UCMST + UCSYNC;  	// 3-pin, 8-bit SPI master
//  UCB0CTL0 &= ~UCCKPH;
//  
//  UCB0CTL1 |= UCSSEL_2;                     		// SMCLK
//  
//  UCB0BR0 = 0x04;									// 1.70 MHz
//  UCB0BR1 = 0;
//  
//  P3SEL |= BIT1 + BIT2 + BIT3;                   	// P3.1, 3.2, 3.3 UCB0SIMO,UCB0SOMI,UCBOCLK option select
//  
//  SLAVE_SELECT_PORT_SET;  						// P3.0 - Slave Select
//  SLAVE_SELECT_HIGH;     							// Slave Select - inactive ( high)
//  
//  UCB0CTL1 &= ~UCSWRST;                     		// **Initialize USCI state machine**
}

//===============================================================
// NAME: void SpiUsciSet (void)
//
// BRIEF: Is used to set USCI B0 for SPI communication
//
// INPUTS:
//
// OUTPUTS:
//
// PROCESS:	[1] make settings
//
// CHANGE:
// DATE  		WHO	DETAIL
// 24Nov2010	RP	Original Code
// 07Dec2010	RP	reduced SPI clock frequency
//===============================================================

void
SpiUsciSet(void)									//Uses USCI_B0
{
//  UCB0CTL1 |= UCSWRST;							// Enable SW reset
//  UCB0CTL0 |= UCCKPH + UCMSB + UCMST + UCSYNC;	// 3-pin, 8-bit SPI master
//  UCB0CTL0 &= ~UCCKPH;
//  UCB0CTL1 |= UCSSEL_2;							// SMCLK
//  
//  UCB0BR0 = 0x04;
//  UCB0BR1 = 0;	
//  P3SEL |= BIT1 + BIT2 + BIT3;					// P3.1,3.2,3.3 UCB0SIMO,UCB0SOMI,UCBOCLK option select
//  
//  SLAVE_SELECT_PORT_SET;							// P3.0 - Slave Select
//  SLAVE_SELECT_HIGH;								// Slave Select - inactive ( high)
//  
//  UCB0CTL1 &= ~UCSWRST;							// **Initialize USCI state machine**
}


//===============================================================
// NAME: void SpiWriteCont (u08_t *pbuf, u08_t length)
//
// BRIEF: Is used in SPI mode to write to a specific number of
// reader chip registers from a specific address upwards.
//
// INPUTS:
//	u08_t	*pbuf	address of first register followed by the
//					contents to write
//	u08_t	length	number of registers + 1
//
// OUTPUTS:
//
// PROCESS:	[1] write to the registers
//
// CHANGE:
// DATE  		WHO	DETAIL
// 24Nov2010	RP	Original Code
// 07Dec2010	RP	integrated wait while busy loops
//===============================================================

void
SpiWriteCont(u08_t *pbuf, u08_t length)
{	
  SLAVE_SELECT_LOW; 						// Start SPI Mode
  // Address/Command Wort Bit Distribution
  *pbuf = (0x20 | *pbuf); 				// address, write, continuous
  *pbuf = (0x3f &*pbuf);					// register address
  while(length > 0)
  {	
//    while (!(IFG2 & UCB0TXIFG))			// USCI_B0 TX buffer ready?
//    {
//    }
//    UCB0TXBUF = *pbuf;					// Previous data to TX, RX
//    while(UCB0STAT & UCBUSY)
//      //while (!(IFG2 & UCB0RXIFG))
//    {
//    }
//    temp = UCB0RXBUF;
    
    /* Wait for SPI2 Tx buffer empty */
    //while(SPI_GetFlagStatus(SPI_I2S_FLAG_TXE) == RESET)
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)//SPI_I2S_GetFlagStatus
    {
    }    
    /* Send byte through the SPI2 peripheral */
    //SPI_SendData(*pbuf);	
    SPI_I2S_SendData(SPI2, *pbuf); 		
    
    /* Wait for SPI2 data reception	*/
    //while(SPI_GetFlagStatus(SPI_I2S_FLAG_TXE) == RESET)
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)//SPI_I2S_GetFlagStatus
    {
    }   
    /* Read & return SPI2 received data	*/
    temp = SPI_I2S_ReceiveData(SPI2);
    
    pbuf++;
    length--;
  }
//  while(UCB0STAT & UCBUSY)
//  {
//  }
  //while (SPI_GetFlagStatus(SPI_I2S_FLAG_TXE)==SET)
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET)//SPI_I2S_GetFlagStatus
  {
  }
  //_NOP();
  //_NOP();
  //_NOP();
  //_NOP();
  //_NOP();
  //_NOP();
  
  SLAVE_SELECT_HIGH; 						// Stop SPI Mode
}

//===============================================================
// NAME: void SpiWriteSingle (u08_t *pbuf, u08_t length)
//
// BRIEF: Is used in SPI mode to write to a specified reader chip
// registers.
//
// INPUTS:
//	u08_t	*pbuf	addresses of the registers followed by the
//					contends to write
//	u08_t	length	number of registers * 2
//
// OUTPUTS:
//
// PROCESS:	[1] write to the registers
//
// CHANGE:
// DATE  		WHO	DETAIL
// 24Nov2010	RP	Original Code
// 07Dec2010	RP	integrated wait while busy loops
//===============================================================

void
SpiWriteSingle(u8 *pbuf, u8 length)
{
  u8	i = 0;
  
 GPIO_ResetBits(GPIOB,GPIO_Pin_12);						// Start SPI Mode
  
  while(length > 0)
  {	
    // Address/Command Word Bit Distribution
    // address, write, single (fist 3 bits = 0)
    *pbuf = (0x1f &*pbuf);				// register address
    for(i = 0; i < 2; i++)
    {	
//      while (!(IFG2 & UCB0TXIFG))		// USCI_B0 TX buffer ready?
//      {
//      }
//      UCB0TXBUF = *pbuf;  			// Previous data to TX, RX
//      
//      while(UCB0STAT & UCBUSY)
//      {
//      }
//      
//      temp = UCB0RXBUF;
      /* Wait for SPI1 Tx buffer empty */
			SPI_I2S_ClearFlag(SPI2,SPI_I2S_FLAG_TXE);
  while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET)
  {
  }    
    /* Send byte through the SPI1 peripheral */
    SPI_I2S_SendData(SPI2,*pbuf);	   
    
    /* Wait for SPI1 data reception	*/
  while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE) == RESET)
  {
  }   
    /* Read & return SPI1 received data	*/
    temp=SPI_I2S_ReceiveData(SPI2);
      
      pbuf++;
      length--;
    }
  }
//  while(UCB0STAT & UCBUSY)
//  {
//  }
  while (SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_BSY)==SET)
  {
  }
  
//  if(direct_mode == 0x00)
  {
    GPIO_SetBits(GPIOB,GPIO_Pin_12);	 						//Stop SPI Mode
  }
}
