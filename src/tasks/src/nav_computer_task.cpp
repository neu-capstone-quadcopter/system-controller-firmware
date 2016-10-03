/*
 * nav_computer_task.cpp
 *
 *  Created on: Sep 27, 2016
 *      Author: bsoper
 */

#include "nav_computer_task.hpp"
#include "hal.hpp"
#include "FreeRTOS.h"
#include "task.h"

#define EVENT_QUEUE_DEPTH 128

namespace nav_computer_task {

static void task_loop(void *p);
void send_data(sensor_task::adc_values_t data);

//NavComputer* nav;
static TaskHandle_t task_handle;

void start() {
	//nav = static_cast<NavComputer*>(hal::get_driver(hal::NAV_COMPUTER));
	xTaskCreate(task_loop, "nav computer task", 1536, NULL, 2, &task_handle);
	event_queue = xQueueCreate(EVENT_QUEUE_DEPTH, sizeof(nav_event_t));
}

static void task_loop(void *p) {
	// UART initialization stuff.

	nav_event_t current_event;
	for(;;) {
		xQueueReceive(event_queue, &current_event, portMAX_DELAY);
		switch (current_event.type) {
		case ADC_SCAN:
			// Package and send data frame
			send_data(current_event.data);
			break;
		}
	}
}

void send_data(sensor_task::adc_values_t data) {
	return;
}

} // End nav_computer_task namespace.


