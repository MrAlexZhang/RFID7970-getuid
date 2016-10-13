/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTI
  
  AL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "main.h"
#include "Globe.h"

uint8_t i_reg;
uint8_t irq_flag;

extern uint8_t abc;
uint16_t CAN_ID;
uint8_t CanFlag,CanDataVar1;
uint8_t CAN_Rec_Data_Temp[8];

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/*******************************************************************************
* Function Name  : TIM2_IRQHandler
* Description    : This function handles TIM2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM2_IRQHandler(void)
{
	
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)	//TIM_IT_Update
 	{	
		if(abc)
		{
			//RFID_LED_OFF;
			abc = 0;
		}
		else
		{
			RFID_LED_ON;
			abc = 1;
		}
		
		TIM_ClearITPendingBit (TIM2, TIM_FLAG_Update);	//必须要清除中断标志位//TIM_IT_Update
	}
}
/*******************************************************************************
* Function Name  : TIM3_IRQHandler
* Description    : This function handles TIM3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM3_IRQHandler(void)
{
	uint8_t irq_status[4];
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)	//TIM_IT_Update
 	{	
		
    
    STOP_COUNTER;
    CLEAR_COUNTER
      
    irq_flag = 0x03;
    
    Trf797xReadIrqStatus(irq_status);
    
    *irq_status = *irq_status & 0xF7;				// set the parity flag to 0
    
    if(*irq_status == 0x00 || *irq_status == 0x80)
    {
      i_reg = 0x00;								// timer interrupt
    }
    else
    {
      i_reg = 0x01;
    }
		TIM_ClearITPendingBit (TIM3, TIM_FLAG_Update);	//必须要清除中断标志位
	}
}
/*******************************************************************************
* Function Name : EXTI9_5_IRQHandler
* Description   : This function handles External lines 9 to 5 interrupt request.
* Input         : None
* Return        : None
* Created By	:  	
* Created date	: 2007.11.28
*-------------------------------------------------------------------------------
*******************************************************************************/
//void EXTI9_5_IRQHandler(void)
//{

//	if(EXTI_GetITStatus(EXTI_Line6) != RESET)//handle key
//	{
//		if(abc)
//		{
//			RFID_LED_OFF;
//			abc = 0;
//		}
//		else
//		{
//			RFID_LED_ON;
//			abc = 1;
//		}
//		/* Clear the EXTI line 5 pending bit */
//		EXTI_ClearITPendingBit(EXTI_Line6);
//	}
//}

/*******************************************************************************
* Function Name  : EXTI15_10_IRQHandler
* Description    : This function handles External lines 15 to 10 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI15_10_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line6) != RESET)
  {
		
    /* Clear the EXTI Line 15 */  
    EXTI_ClearITPendingBit(EXTI_Line6);
  }
 
}
//void EXTI9_5_IRQHandler(void)
//{
//		if(EXTI_GetITStatus(EXTI_Line6) != RESET)
//		{
//				EXTI_ClearITPendingBit(EXTI_Line6);

//		}			
//}
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

/*******************************************************************************
* Function Name  : USART1_IRQHandler
* Description    : This function handles External lines 15 to 10 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void USART1_IRQHandler(void)
{
  //uint8_t usart1_rcvdata;                       

  if((USART1->SR & USART_FLAG_RXNE) == USART_FLAG_RXNE)
  {                                              
    //usart1_rcvdata = (uint8_t)(USART1->DR);

    //USART1_SendByte(usart1_rcvdata);            
  }
}

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
