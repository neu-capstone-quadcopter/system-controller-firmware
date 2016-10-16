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
#include "message_types.hpp"
#include "telemetry_radio_task.hpp"

namespace led_task {
	static void task_loop(void *p);

	TaskHandle_t task_handle;

	ExampleLed *led0;
	ExampleLed *led1;

	void start(void) {
		// Retrieve driver instances from HAL
		led0 = hal::get_driver<ExampleLed>(hal::LED_0);
		led1 = hal::get_driver<ExampleLed>(hal::LED_1);

		xTaskCreate(task_loop, "led task", 128, NULL, 2, &task_handle);
	}

	static void task_loop(void *p) {
		for(;;) {
			led0->set_led(false);
			led1->set_led(true);
			vTaskDelay(500);
			led0->set_led(true);
			led1->set_led(false);
			vTaskDelay(500);

			ExampleMessage* m = new ExampleMessage();
			m->x = 1337;
			m->y = "Im in the house!!";
			telemetry_radio_task::queue_outgoing_message(m);
		}
	}
}

