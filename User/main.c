#include "stm32f10x.h"
#include "main.h"
#include "Globe.h"
#include "sysinit.h"

uint8_t abc = 0;//
uint8_t buf[300];   // 原来是300，现在改为130，因为太大了会造成内存不够
uint8_t TRF_INITIAL_DONE = 0;
uint8_t enable = 0;

uint8_t rx_error_flag = 0x00;
int8_t rxtx_state = 1;							// used for transmit recieve byte count

int16_t nfc_state;

uint8_t remote_flag = 0;
uint8_t stand_alone_flag = 1;
uint8_t reader_mode = 0x00;					// determines how interrups will be handled



/*----------------CAN--------------------*/

uint8_t  slave_id=13;
uint8_t  pin_S0=0;
uint8_t doreceivenum=0,framenum=0;
extern uint8_t  CanDataVar1;
uint8_t CAN_TXDATA[8]={0x5A,0x01,};
uint16_t VirtAddVarTab[8] = {0x0001, 0x0002, 0x0003,0x0004,0x0005,0x0006,0x0007,0x0008};
extern uint8_t CAN_Rec_Data_Temp[8];
uint8_t  PcACK = 0;				// PC是否返回ACK的标志，ACK返回成功(0),无法回需重发(1)
uint8_t  ResendNum = 0;			// 重发次数计数器
uint32_t ResendTimer = 0;     	// 重发计时器
unsigned char ReadBuff[8];
/* Private function prototypes -----------------------------------------------*/
void Read_CAN_ID(void);
void CAN_Unpack(void);
u8 buff_last[8];
extern void GetUID(unsigned char *ReadBuff);

/*----------------CAN--------------------*/

int main(void)
{	
	u8 can_tx_data[8];
	u8 cake=0;
	RCC_Configuration();
	NVIC_Configuration();
	GPIO_Configuration();
	ExtInt_Configuration ();
	TIM2_Configuration();
	TIM3_Configuration();
	SLAVE_SELECT_HIGH;
	SPI_Initializes();
	
	TRF7970_Init();

	//Read_CAN_ID();
	CAN_Configuration();
	SysTick_Init(72);   //滴答时钟初始化
	
	RFID_LED_ON;
	
//	ENABLE_SET;							// P1.0 is switched in output direction
  			TRF_DISABLE;
		delay_ms(500);
		TRF_ENABLE;
  delay_ms(500);				// wait until system clock started 
//  Trf797xCommunicationSetup();		// settings for communication with TRF
SLAVE_SELECT_HIGH;
  Trf797xInitialSettings();			// Set MCU Clock Frequency to 6.78 MHz and OOK Modulation
  delay_ms(10);
  TRF_INITIAL_DONE = 1;							// indicates, that setting are done
	reader_mode = 0x00;
  stand_alone_flag = 1;				// stand alone mode
  remote_flag = 0;					// host is not active
    //	Settings done

  
  delay_ms(10);
	while(1)
	{
		delay_ms(200);
		GetUID(ReadBuff);
		for(cake=0;cake<8;cake++)
		{
				if((ReadBuff[cake]!=buff_last[cake])&&(ReadBuff[cake]!=0))
				{
						can_tx_data[0]=0x5A;
						can_tx_data[1]=0x40;
						can_tx_data[2]=0x00;
						can_tx_data[3]=ReadBuff[0];	
						can_tx_data[4]=ReadBuff[1];		
						can_tx_data[5]=ReadBuff[2];
						can_tx_data[6]=ReadBuff[3];						
						CanWriteData(can_tx_data);
						delay_ms(50);
						can_tx_data[0]=0x5A;
						can_tx_data[1]=0x41;
						can_tx_data[2]=0x00;
						can_tx_data[3]=ReadBuff[4];	
						can_tx_data[4]=ReadBuff[5];		
						can_tx_data[5]=ReadBuff[6];
						can_tx_data[6]=ReadBuff[7];						
						CanWriteData(can_tx_data);
						delay_ms(50);
						
						break;
				}
		
		}
		for(cake=0;cake<8;cake++)
		{
				buff_last[cake]=ReadBuff[cake];
		}

		

		
		
						
		
//		delay_ms(500);
		
//		GetUID(ReadBuff);
		//delay_ms(500);
		//CanWriteData(CAN_TXDATA);
		//delay_ms(500);

		
//		if(CanDataVar1)
//		{
//			CAN_Unpack();
//			CanDataVar1=0;
//		}	
//		Find_RFID();
//		if(PcACK &&(ResendTimer >= 5))
//		{
//			ResendTimer = 0;
//			if((ResendNum++) < 3)
//			{
//				CAN_TXDATA[2]=0x01;
//				CAN_TXDATA[3]=0x02;
//				CanWriteData(CAN_TXDATA);
//			}
//			else
//			{
//				PcACK = 0;  
//			}
//		}
	}
}

void Read_CAN_ID(void)
 {
   slave_id=GPIO_ReadInputData(GPIOA);
  // slave_id=(slave_id&0x0018)>>3;
  // slave_id=slave_id+13;
 }

void Find_RFID(void)
{  
#if TRIGGER						// in Mcu.h
    LED_OPEN1_ON;
    delay_ms(1);
    LED_OPEN1_OFF;
#endif
    if(remote_flag == 1) 			// if in remote mode
    {								// remote mode can be disabled in host.h
#ifdef ENABLE_HOST
      buf[4] = 0xff;			// "TRF7960 EVM" message in GUI
      stand_alone_flag = 0;
      LED_ALL_OFF;
      LED_POWER_ON;
      HostCommands();			// device works according host commands
#endif
    }
    else
    {	
#ifdef ENABLE15693				// this standard can be disabled in ISO15693.h
      Iso15693FindTag();			// detects ISO15693 in stand-alone mode
#endif
      if(remote_flag == 0)
      {	
#ifdef ENABLE14443A			// this standard can be disabled in ISO14443A.h 
        Iso14443aFindTag();		// detects ISO14443A in stand-alone mode
#endif
      }
      if(remote_flag == 0)
      {
#ifdef ENABLE14443B			// this standard can be disabled in ISO14443B.h
        Iso14443bFindTag();		// detects ISO14443B in stand-alone mode
#endif
      }
      if(remote_flag == 0)
      {
#ifdef ENABLE_FELICA		// this standard can be disabled in felica.h
        FindFelica();			// detects FeliCa in stand-alone mode
#endif
      }
    }
 }
 
 
//------------------------------------------------------------------------------------
void CAN_Unpack(void)
{  	
	if(CAN_Rec_Data_Temp[0]==0x55)	                                
    {
		switch(CAN_Rec_Data_Temp[1])
		{
			case  0x01:    //橱窗门锁控制						
				switch(CAN_Rec_Data_Temp[3])
				{
					case 0x01 :
						pin_S0 = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15);  //门开关状态
						if (pin_S0 == 1)       //此时门关闭 动作一次锁
						{
							delay_ms(500);
							delay_ms(500);
						}
						break;
					case 0x02 :
						//LOCK_OFF; 
						break;
					case 0x03 :      //查询门开关状态
						pin_S0 = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15);
						if (pin_S0 == 1)    //门关闭
						{
							CAN_TXDATA[2]=0x00;
							CAN_TXDATA[3]=0x02;//WZL 0x01 to 0x02
							CanWriteData(CAN_TXDATA);	

							PcACK = 1; 			// PC反馈ACK标志
							ResendNum = 0;		// 重发计数器清除
							ResendTimer = 0; 	// 重发计时器清除							
						}
						else		    //门打开
						{
							CAN_TXDATA[2]=0x00;
							CAN_TXDATA[3]=0x01;//WZL 0x02 to 0x01
							CanWriteData(CAN_TXDATA);
							
							PcACK = 0; 			// PC反馈ACK标志
							ResendNum = 0;		// 重发计数器清除
							ResendTimer = 0; 	// 重发计时器清除
						}
						break;
					case 0xaa :				// PC反馈ACK
						PcACK = 0;			// 重发标志清除
						ResendNum = 0;		// 重发计数器清除
						ResendTimer = 0; 	// 重发计时器清除
					    break;
				}					  
				break;			
			case 0x02:   //点灯控制			
				switch (CAN_Rec_Data_Temp[3])
				{
					case 0x01 :
						//LAMP_ON  ;
						break;
					case 0x02 :
						//LAMP_OFF  ; 
						break;
				} 
				break; 
			case 0x03:  //块把手led灯控制
				switch (CAN_Rec_Data_Temp[3])
				{
					case 0x01 :
						//HANDLE_LED_ON      ;
						break;

					case 0x02 :
						//HANDLE_LED_OFF     ; 
						break;
				}
				break;
		}
	}  
}
/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
