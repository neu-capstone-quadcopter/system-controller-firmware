/*
 * util.cpp
 *
 *  Created on: Nov 22, 2016
 *      Author: nigil
 */


#include "chip.h"
#include "util.hpp"

#include "FreeRTOS.h"

IRQn_Type get_timer_nvic_irq(LPC_TIMER_T *timer) {
	switch((uint32_t)timer)
	{
	case LPC_TIMER0_BASE:
		return TIMER0_IRQn;
	case LPC_TIMER1_BASE:
		return TIMER1_IRQn;
	case LPC_TIMER2_BASE:
		return TIMER1_IRQn;
	case LPC_TIMER3_BASE:
		return TIMER1_IRQn;
	default:
		configASSERT(0);
	}
	return TIMER0_IRQn;
}

