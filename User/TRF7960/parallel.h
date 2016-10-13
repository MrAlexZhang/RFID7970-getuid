//***************************************************************
//------------<02.Dec.2010 by Peter Reiser>----------------------
//***************************************************************

#ifndef _PARALLEL_H_
#define _PARALLEL_H_

//================================================================
#include "stm32f10x.h"
#include <stm32f10x_conf.h>
//#include <stdio.h>							// standard input/output header
#include "mcu.h"
#include "types.h"

//===============================================================

void ParallelDirectCommand(u08_t *pbuf);
void ParallelDirectMode(void);
void ParallelRawWrite(u08_t *pbuf, u08_t length);
void ParallelReadCont(u08_t *pbuf, u08_t length);
void ParallelReadSingle(u08_t *pbuf, u08_t length);
void ParallelSetup(void);
void ParallelWriteCont(u08_t *pbuf, u08_t length);
void ParallelWriteSingle(u08_t *pbuf, u08_t length);

//===============================================================

#endif
