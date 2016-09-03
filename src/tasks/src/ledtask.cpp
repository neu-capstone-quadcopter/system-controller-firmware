/*
 * ledtask.cpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nigil
 */

#include "ledtask.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include "hal.hpp"
#include "exampleled.hpp"

namespace led_task {
	static void task_loop(void *p);

	TaskHandle_t task_handle;

	ExampleLed *led0;
	ExampleLed *led1;

	void start(void) {
		// Retrieve driver instances from HAL
		led0 = static_cast<ExampleLed*>(hal::get_driver(hal::LED_0));
		led1 = static_cast<ExampleLed*>(hal::get_driver(hal::LED_1));

		xTaskCreate(task_loop, "led task", 1536, NULL, 2, &task_handle);
	}

	static void task_loop(void *p) {
		for(;;) {
			led0->set_led(false);
			led1->set_led(true);
			vTaskDelay(100);
			led0->set_led(true);
			led1->set_led(false);
			vTaskDelay(100);
		}
	}
}

