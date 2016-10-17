/*
 * flight_controller_task.cpp
 *
 *  Created on: Oct 14, 2016
 *      Author: ncasale
 */

#include "flight_controller_task.hpp"
#include "hal.hpp"
#include "uartio.hpp"
#include "FreeRTOS.h"
#include "task.h"

#include <stdlib.h>
#include <cstring>

#define EVENT_QUEUE_DEPTH 8
#define MAX_BUFFER_SIZE 255

namespace flight_controller_task {

static void task_loop(void *p);
void send_data(uint8_t *data);
void write_to_sbus(uint8_t *data, uint8_t len);
void read_from_uart();
void fc_bb_read_handler(std::shared_ptr<UartReadData> read_status);

UartIo* fc_blackbox_uart;
UartIo* fc_sbus_uart;
static TaskHandle_t task_handle;

auto fc_bb_read_del = dlgt::make_delegate(&fc_bb_read_handler);

void start() {
	fc_blackbox_uart = hal::get_driver<UartIo>(hal::FC_BLACKBOX_UART);
	fc_sbus_uart = hal::get_driver<UartIo>(hal::FC_SBUS_UART);
	xTaskCreate(task_loop, "flight controller", 400, NULL, 2, &task_handle);
	flight_cont_event_queue = xQueueCreate(EVENT_QUEUE_DEPTH, sizeof(flight_cont_event_t));
}

static void task_loop(void *p) {
	flight_cont_event_t current_event;
	fc_blackbox_uart->allocate_buffers(150,150);
	//Checking for blackbox data on async basis
	fc_blackbox_uart->read_async(100, fc_bb_read_del);
	for(;;) {
		xQueueReceive(flight_cont_event_queue, &current_event, portMAX_DELAY);
		switch (current_event.type) {
		case BLACKBOX_READ:
			// Receive data frame from blackbox
			read_from_uart();
			break;
		case FLIGHT_COMMAND:
			//Do operations to send command
			//to flight controller
		default:
			break;
		}
	}
}

void add_event_to_queue(flight_cont_event_t event) {
	xQueueSendToBack(flight_cont_event_queue, &event, 0);
}

//Modify this function to send flight command over
//SBUS Interface
void write_to_sbus(uint8_t *data, uint8_t len) {
	//SBUS uses a 25 byte data frame
	uint8_t buffer[25];

	//Will need to accept a command from the nav computer
	//in some predefined format which we will then send to SBUS

	//Send over SBUS -- Uart2
	fc_sbus_uart->write(data, 25);

}

//Modify this function to read in blackbox data from
//the serial interface
void read_from_uart() {
	//Our initial buffer
	uint8_t buffer[128];
	memset(buffer, 0x00, 128);

	//Figure out length of our incoming message
	uint8_t length = 128;

	//Read incoming message into buffer
	fc_blackbox_uart->read(buffer, length);

	//Do something with this message
	buffer[127] = 0x00;

}

void fc_bb_read_handler(std::shared_ptr<UartReadData> read_status)
	{
		std::unique_ptr<uint8_t[]> read_data = std::move(read_status->data);

		flight_cont_event_t e;
		e.type = BLACKBOX_READ;
		e.length = 100;

		e.data = read_data.get();
		xQueueSendToBackFromISR(flight_cont_event_queue, &e, 0);
		fc_blackbox_uart->read_async(100, fc_bb_read_del);
	}

//This function will package up our data to then
//send over our SBUS
void send_data(uint8_t* data) {
	static uint8_t buffer[MAX_BUFFER_SIZE];
	memset(buffer, 0x00, MAX_BUFFER_SIZE);

	uint8_t message_len = 0;
	write_to_sbus(buffer, MAX_BUFFER_SIZE);
	return;
}

} // End flight_controller_task namespace.
