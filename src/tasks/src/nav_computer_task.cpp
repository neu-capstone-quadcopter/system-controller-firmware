/*
 * nav_computer_task.cpp
 *
 *  Created on: Sep 27, 2016
 *      Author: bsoper
 */

#include "nav_computer_task.hpp"
#include "hal.hpp"
#include "uartio.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "api.pb.h"

//#include "sockets.h"
//#include <arpa/inet.h>

#define EVENT_QUEUE_DEPTH 8

namespace nav_computer_task {

static void task_loop(void *p);
void send_data(sensor_task::adc_values_t data);
void write_to_uart(uint8_t *data, uint16_t len);

UartIo* nav_uart;
static TaskHandle_t task_handle;

void start() {
	nav_uart = hal::get_driver<UartIo>(hal::NAV_COMPUTER);
	BaseType_t xReturned;
	xReturned = xTaskCreate(task_loop, "nav computer", 400, NULL, 2, &task_handle);
	if (xReturned == pdPASS)
		asm("nop;");
	else
		asm("nop;");
	nav_event_queue = xQueueCreate(EVENT_QUEUE_DEPTH, sizeof(nav_event_t));
}

static void task_loop(void *p) {
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

void write_to_uart(uint8_t *data, uint16_t len) {
	nav_uart->write((uint8_t*) &len, 2);
	nav_uart->write(data, (uint8_t) len);
}

void send_data(sensor_task::adc_values_t data) {
	uint8_t buffer[128];
	bool status;

	monarcpb_SysCtrlToNavCPU_Telemetry message = {42};

	/* Create a stream that will write to our buffer. */
	pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

	/* Now we are ready to encode the message! */
	status = pb_encode(&stream, monarcpb_NavCPUToSysCtrl_Telemetry_fields, &message);
	uint16_t message_len = stream.bytes_written;
	write_to_uart(buffer, message_len);
	return;
}

} // End nav_computer_task namespace.


