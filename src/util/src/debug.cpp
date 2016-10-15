/*
 * debug.cpp
 *
 *  Created on: Oct 14, 2016
 *      Author: nigil
 */

#include "chip.h"

extern "C" {
	extern void init_runtime_stats_timer( void );

	void init_runtime_stats_timer( void )
	{
		Chip_TIMER_Init(LPC_TIMER3);
		Chip_TIMER_Reset(LPC_TIMER3);
		Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_TIMER3, SYSCTL_CLKDIV_1);
		Chip_TIMER_PrescaleSet(LPC_TIMER3, 3200);
		Chip_TIMER_Enable(LPC_TIMER3);
	}
}


