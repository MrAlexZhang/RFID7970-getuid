//***************************************************************
//------------<27.Jan.2011 by Peter Reiser>----------------------
//***************************************************************

#ifndef MCU_H_
#define MCU_H_

//===============================================================

//#define	MSP430F23X0

//===============================================================

#ifdef MSP430F23X0
	#include <MSP430x23x0.h>
	#include "msp430f23x0.h"
#endif

#include "types.h"

//===============================================================

void McuCounterSet(void);
void McuDelayMillisecond(u32_t n_ms);
void McuOscSel(u08_t mode);

#define McuDelayMillisecond(n_ms) delay_ms(n_ms);//delay_ms(uint16_t nms);


//===============================================================

#endif
