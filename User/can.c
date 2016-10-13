#include "stm32f10x.h"
//#include "SysTick/systick.h" 
//#include <stdlib.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
uint16_t CAN_ID;
//uint8_t CAN_DATA0,CAN_DATA1,CAN_DATA2,CAN_DATA3,CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
uint8_t CanFlag,Display,CanDataVar,receivenum=0,CAN_Rec_Data_Temp[8],CAN_Rec_Data_Temp1[80];
uint8_t CanDataVar1;
uint8_t slave_id,framenum;

/* Private function prototypes -----------------------------------------------*/
void CAN_Configuration(void);
void CanWriteData(uint8_t *CAN_TX_DATA);
//void CAN_Unpack(void);
/*******************************************************************************
* Function Name  : Delay
* Description    : Delay Time
* Input          : - nCount: Delay Time
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void  Delay (uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}


/*******************************************************************************
* Function Name  : CAN_Configuration
* Description    : Configures the CAN
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void CAN_Configuration(void)
{
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;

  CAN_DeInit(CAN1);
  CAN_StructInit(&CAN_InitStructure);
	//NVIC_CAN_Configuration();
  CAN_InitStructure.CAN_TTCM=DISABLE;
  CAN_InitStructure.CAN_ABOM=DISABLE;
  CAN_InitStructure.CAN_AWUM=DISABLE;
  CAN_InitStructure.CAN_NART=DISABLE;
  CAN_InitStructure.CAN_RFLM=DISABLE;
  CAN_InitStructure.CAN_TXFP=DISABLE;
  CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
  CAN_InitStructure.CAN_BS1=CAN_BS1_4tq;
  CAN_InitStructure.CAN_BS2=CAN_BS2_7tq;
  CAN_InitStructure.CAN_Prescaler=6; //500K

  CAN_Init(CAN1,&CAN_InitStructure);

  /* CAN filter init */
  CAN_FilterInitStructure.CAN_FilterNumber=0;
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000|(slave_id<<5);					 //
  CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xffe0;;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);

  /* CAN FIFO0 message pending interrupt enable */ 
  CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);
}


/*******************************************************************************
* Function Name  : CanWriteData
* Description    : Can Write Date to CAN-BUS
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void CanWriteData(uint8_t *CAN_TX_DATA)
{
  CanTxMsg TxMessage;

  /* transmit */
  TxMessage.StdId = 0x0000|slave_id;
  TxMessage.RTR = CAN_RTR_DATA;
  TxMessage.IDE = CAN_ID_STD;
  TxMessage.DLC = 8;
  TxMessage.Data[0] = *CAN_TX_DATA;    
  TxMessage.Data[1] = *(CAN_TX_DATA+1);    
  TxMessage.Data[2] = *(CAN_TX_DATA+2);    
  TxMessage.Data[3] = *(CAN_TX_DATA+3);    
  TxMessage.Data[4] = *(CAN_TX_DATA+4);    
  TxMessage.Data[5] = *(CAN_TX_DATA+5);     
  TxMessage.Data[6] = *(CAN_TX_DATA+6);    
  TxMessage.Data[7] = *(CAN_TX_DATA+7);      
  CAN_Transmit(CAN1,&TxMessage);
}

/*******************************************************************************
* Function Name  : USB_LP_CAN1_RX0_IRQHandler
* Description    : This function handles USB Low Priority or CAN RX0 interrupts 
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  CanRxMsg RxMessage;
  CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);
  CAN_ID=RxMessage.StdId;
  CAN_Rec_Data_Temp[0]=RxMessage.Data[0];
  CAN_Rec_Data_Temp[1]=RxMessage.Data[1];
  CAN_Rec_Data_Temp[2]=RxMessage.Data[2];
  CAN_Rec_Data_Temp[3]=RxMessage.Data[3];
  CAN_Rec_Data_Temp[4]=RxMessage.Data[4];
  CAN_Rec_Data_Temp[5]=RxMessage.Data[5];
  CAN_Rec_Data_Temp[6]=RxMessage.Data[6];
  CAN_Rec_Data_Temp[7]=RxMessage.Data[7];
	CanDataVar1=1;
	CAN_FIFORelease(CAN1,CAN_FIFO0);//
  CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);
  CanFlag = ENABLE;

}


  
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

