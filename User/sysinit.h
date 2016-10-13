/**
  ******************************************************************************
  * @file    sysinit.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the EXTI firmware
  *          library.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SYSINIT_H
#define __SYSINIT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//#define EXTI_Line0       ((uint32_t)0x00001)  /*!< External interrupt line 0 */
//#define EXTI_Line1       ((uint32_t)0x00002)  /*!< External interrupt line 1 */
//#define EXTI_Line2       ((uint32_t)0x00004)  /*!< External interrupt line 2 */
//#define EXTI_Line3       ((uint32_t)0x00008)  /*!< External interrupt line 3 */
//#define EXTI_Line4       ((uint32_t)0x00010)  /*!< External interrupt line 4 */

void GPIO_Configuration(void);
void CAN_Configuration(void);
void TIM2_Configuration( void );
void TIM3_Configuration( void );
void RCC_Configuration(void);
void NVIC_Configuration(void);
void ExtInt_Configuration (void);
void SysTick_Init(uint8_t SYSCLK);
void CanWriteData(uint8_t *CAN_TX_DATA);
void delay_ms(uint16_t nms);
void delay_us(uint32_t nus);



#ifdef __cplusplus
}
#endif

#endif /* __SYSINIT_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
