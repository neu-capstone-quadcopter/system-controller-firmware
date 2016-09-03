/*
 * hal.cpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nigil
 */

#include "chip.h"

#include "hal.hpp"
#include "driver.hpp"
#include "exampleled.hpp"

#define DRIVER_COUNT 2

Driver *drivers[DRIVER_COUNT];

void hal_init(void) {
	Chip_GPIO_Init(LPC_GPIO);

	// Instantiate device drivers in order of dependence
	drivers[LED_0] = new ExampleLed(0, 9);
	drivers[LED_1] = new ExampleLed(0, 8);

	// Initialize all drivers
	for(int i = 0; i < DRIVER_COUNT; i++) {
		drivers[i]->init_driver();
	}
}

Driver *hal_get_driver(hal_driver_identifier id) {
	return drivers[id];
}
