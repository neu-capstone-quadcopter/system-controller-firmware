/*
 * hal.cpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nigil
 */

#include "chip.h"

#include "hal.hpp"
#include "cc1120.hpp"
#include "driver.hpp"
#include "exampleled.hpp"

#define DRIVER_COUNT 3

Driver *drivers[DRIVER_COUNT];

namespace hal {
	void init(void) {
		Chip_GPIO_Init(LPC_GPIO);
		Chip_IOCON_Init(LPC_IOCON);

		// Instantiate device drivers in order of dependence
		drivers[LED_0] = new ExampleLed(2, 11);
		drivers[LED_1] = new ExampleLed(2, 12);
		drivers[TELEM_CC1120] = new Cc1120(LPC_SSP1);

		// Initialize all drivers
		for(int i = 0; i < DRIVER_COUNT; i++) {
			drivers[i]->init_driver();
		}
	}

	Driver *get_driver(driver_identifier id) {
		return drivers[id];
	}
}

void SSP1_IRQHandler() {
	static_cast<Cc1120*>(drivers[hal::TELEM_CC1120])->ssp_interrupt_handler();
}
