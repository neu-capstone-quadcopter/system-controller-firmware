/*
 * hal.cpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nigil
 */

#include "chip.h"

#include "hal.hpp"
#include "sspio.hpp"
#include "cc1120.hpp"
#include "driver.hpp"
#include "exampleled.hpp"

namespace hal {
	void add_devices(void);

	Driver *drivers[INTENTIFIER_MAX];

	void init(void) {
		Chip_GPIO_Init(LPC_GPIO);
		Chip_IOCON_Init(LPC_IOCON);

		// Initialize all drivers
		for(int i = 0; i < INTENTIFIER_MAX; i++) {
			drivers[i]->init_driver();
		}
	}

	void add_devices(void) {
		// Instantiate devices
		SspIo *telem_cc1120_ssp = new SspIo(LPC_SSP1);
		ExampleLed *led_0 = new ExampleLed(2, 11);
		ExampleLed *led_1 = new ExampleLed(2, 11);
		Cc1120 *telem_cc1120 = new Cc1120(telem_cc1120_ssp);

		// Add devices to device array
		drivers[TELEM_CC1120_SSP] = telem_cc1120_ssp;
		drivers[LED_0] = led_0;
		drivers[LED_1] = led_1;
		drivers[TELEM_CC1120] = telem_cc1120;
	}

	Driver *get_driver(driver_identifier id) {
		return drivers[id];
	}
}

extern "C" {
	using namespace hal;
	void SSP1_IRQHandler() {
		static_cast<SspIo*>(drivers[TELEM_CC1120_SSP])->ssp_interrupt_handler();
	}
}
