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

#include <stdlib.h>

#define EVENT_QUEUE_DEPTH 8
#define MAX_BUFFER_SIZE 1150

namespace nav_computer_task {

static void task_loop(void *p);
void send_data(sensor_task::adc_values_t data);
void write_to_uart(uint8_t *data, uint16_t len);
void read_from_uart();

UartIo* nav_uart;
static TaskHandle_t task_handle;

void start() {
	nav_uart = hal::get_driver<UartIo>(hal::NAV_COMPUTER);
	xTaskCreate(task_loop, "nav computer", 400, NULL, 2, &task_handle);
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
			read_from_uart();
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

void read_from_uart() {
	uint8_t buffer[128];
	// Read length of incoming message.
	uint8_t len[2];
	nav_uart->read(len, 2);
	uint8_t temp = len[0];
	len[0] = len[1];
	len[1] = temp;
	uint16_t message_len = *((uint16_t*)len);
	// Read message.
	nav_uart->read(buffer, message_len);
	pb_istream_t stream = pb_istream_from_buffer(buffer, (uint8_t) message_len);
	// Decode message.
	monarcpb_NavCPUToSysCtrl message = monarcpb_NavCPUToSysCtrl_init_zero;
	pb_decode(&stream, monarcpb_NavCPUToSysCtrl_fields, &message);
	int32_t gps = message.telemetry.GPS;
}


void send_data(sensor_task::adc_values_t data) {
	static uint8_t buffer[MAX_BUFFER_SIZE];
	memset(buffer, 0x00, MAX_BUFFER_SIZE);

	monarcpb_SysCtrlToNavCPU message = monarcpb_SysCtrlToNavCPU_init_zero;
	message.has_telemetry = true;
	message.telemetry.has_altitude = true;
	message.telemetry.altitude = data.sensor_values[8];
	message.telemetry.has_atmospheric_pressure = true;
	message.telemetry.atmospheric_pressure = data.sensor_values[13];

	/* Create a stream that will write to our buffer. */
	pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

	/* Now we are ready to encode the message! */
	pb_encode(&stream, monarcpb_SysCtrlToNavCPU_fields, &message);
	uint16_t message_len = stream.bytes_written;

	write_to_uart(buffer, message_len);
	return;
}

} // End nav_computer_task namespace.


