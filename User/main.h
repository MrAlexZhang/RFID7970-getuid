#ifndef __MAIN_H
#define	__MAIN_H


#include "stm32f10x.h"


#define RFID_LED_OFF        GPIO_SetBits(GPIOB, GPIO_Pin_5 )
#define RFID_LED_ON         GPIO_ResetBits(GPIOB, GPIO_Pin_5 )

void Find_RFID(void);
void CAN_Unpack(void);
void Read_CAN_ID(void);

#endif /* __MAIN_H */
