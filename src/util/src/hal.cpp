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

		// Instantiate device drivers in order of dependence
		drivers[LED_0] = new ExampleLed(0, 9);
		drivers[LED_1] = new ExampleLed(0, 8);
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
