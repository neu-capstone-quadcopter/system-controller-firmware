/*
 * telemetry_radio_task.cpp
 *
 *  Created on: Sep 12, 2016
 *      Author: justin
 */

#include "telemetry_radio_task.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include "hal.hpp"
#include "cc1120.hpp"

namespace telemetry_radio_task {
	static void task_loop(void *p);

	TaskHandle_t task_handle;

	Cc1120 *telem_cc1120;

	void start(void) {
		// Retrieve driver instances from HAL
		telem_cc1120 = static_cast<Cc1120*>(hal::get_driver(hal::TELEM_CC1120));

		xTaskCreate(task_loop, "telem radio task", 1536, NULL, 2, &task_handle);
	}

	static void task_loop(void *p) {
		telem_cc1120->ssp_write(0xCC);
		for (;;) {
			asm("nop;");
		}
	}
}
