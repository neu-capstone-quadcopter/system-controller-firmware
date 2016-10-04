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

#define EVENT_QUEUE_DEPTH 8

namespace nav_computer_task {

static void task_loop(void *p);
void send_data(sensor_task::adc_values_t data);

//NavComputer* nav;
static TaskHandle_t task_handle;

void start() {
	//nav = static_cast<NavComputer*>(hal::get_driver(hal::NAV_COMPUTER));
	BaseType_t xReturned;
	xReturned = xTaskCreate(task_loop, "nav computer", 400, NULL, 2, &task_handle); //1536
	if (xReturned == pdPASS)
		asm("nop;");
	else
		asm("nop;");
	nav_event_queue = xQueueCreate(EVENT_QUEUE_DEPTH, sizeof(nav_event_t));
}

static void task_loop(void *p) {
	// UART initialization stuff.

	nav_event_t current_event;
	for(;;) {
		xQueueReceive(nav_event_queue, &current_event, portMAX_DELAY);
		switch (current_event.type) {
		case ADC_SCAN:
			// Package and send data frame
			send_data(current_event.data);
			break;
		default:
			break;
		}
	}
}

void add_event_to_queue(nav_event_t event) {
	xQueueSendToBack(nav_event_queue, &event, 0);
}

void send_data(sensor_task::adc_values_t data) {
	asm("nop;");
	return;
}

} // End nav_computer_task namespace.


