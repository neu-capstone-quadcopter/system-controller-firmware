/*
 * wdt.cpp
 *
 *  Created on: Nov 27, 2016
 *      Author: bsoper
 */

#include "wdt.hpp"
#include "chip.h"
#include "board.hpp"

WDT::WDT(uint32_t freq) {
	frequency = freq;
}

void WDT::init_driver(void) {
	Chip_WWDT_Init(LPC_WWDT);

	// Set WDT timeout to 100 ticks.
	Chip_WWDT_SetTimeOut(LPC_WWDT, frequency);
	// Reset WDT on timeout.
	Chip_WWDT_SetOption(LPC_WWDT, WWDT_WDMOD_WDRESET);
	Chip_WWDT_ClearStatusFlag(LPC_WWDT, WWDT_WDMOD_WDTOF | WWDT_WDMOD_WDINT);

	NVIC_ClearPendingIRQ(WDT_IRQn);
	NVIC_EnableIRQ(WDT_IRQn);
}

void WDT::feed() {
	Chip_WWDT_Feed(LPC_WWDT);
}

void WDT::start() {
	Chip_WWDT_Start(LPC_WWDT);
}


