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
#define SBUS_CHANNEL_MASK 0x07ff
#define SBUS_CHANNEL_BIT_LEN 11
#define SBUS_CHANNEL_17 16
#define SBUS_CHANNEL_18 17

namespace flight_controller_task {

const uint8_t SBUS_INTERVAL_MS = 7;
const uint8_t SBUS_FRAME_LEN = 25;

struct SBusFrame{
	uint16_t channels[18];
	bool frame_lost;
	bool fail_safe_activated;

	/*
	 * Make a raw data copy of the frame suitable for sending to cleanflight
	 */
	void serialize(uint8_t* raw_frame) {
		memset(raw_frame, 0, 25);
		raw_frame[0] = 0xF0;
		uint8_t byte_idx = 1;
		int8_t start_pos = 8;

		for(uint8_t i = 0; i < 16; i++) {
			uint16_t ch_masked = this->channels[i] & SBUS_CHANNEL_MASK;
			int8_t bits_to_write = 11;
			raw_frame[byte_idx++] |= ch_masked >> (bits_to_write -= start_pos);
			start_pos = 8 - bits_to_write;
			if(start_pos > 0) {
				raw_frame[byte_idx] |= ch_masked << start_pos;
			}
			else {
				start_pos += 8 - start_pos;
				raw_frame[byte_idx++] |= ch_masked >> (bits_to_write -= start_pos);
				start_pos = 8 - bits_to_write;
				raw_frame[byte_idx] |= ch_masked << start_pos;
			}
		}

		if(this->channels[SBUS_CHANNEL_17]) {
			raw_frame[23] |= (1 << 8);
		}

		if(this->channels[SBUS_CHANNEL_18]) {
			raw_frame[23] |= (1 << 7);
		}
	}
};

static void task_loop(void *p);
void setup_ritimer(void);
void send_data(uint8_t *data);
void write_to_sbus(uint8_t *data, uint8_t len);
void read_from_uart();
void fc_bb_read_handler(std::shared_ptr<UartReadData> read_status);
static void sbus_frame_written_handler(UartError status);

static UartIo* fc_blackbox_uart;
static UartIo* fc_sbus_uart;
static TaskHandle_t task_handle;
static SBusFrame sbus_frame;

auto fc_bb_read_del = dlgt::make_delegate(&fc_bb_read_handler);
auto sbus_written_del = dlgt::make_delegate(&sbus_frame_written_handler);

void start() {
	fc_blackbox_uart = hal::get_driver<UartIo>(hal::FC_BLACKBOX_UART);
	fc_sbus_uart = hal::get_driver<UartIo>(hal::FC_SBUS_UART);
	xTaskCreate(task_loop, "flight controller", 400, NULL, 2, &task_handle);
	flight_cont_event_queue = xQueueCreate(EVENT_QUEUE_DEPTH, sizeof(flight_cont_event_t));
}

static void task_loop(void *p) {
	fc_sbus_uart->allocate_buffers(30, 0);
	fc_blackbox_uart->allocate_buffers(0,150);

	/*
	sbus_frame.channels[0] = 0x401;
	sbus_frame.channels[1] = 0x401;
	sbus_frame.channels[2] = 0x401;
	sbus_frame.channels[3] = 0x401;
	sbus_frame.channels[4] = 0x401;
	sbus_frame.channels[5] = 0x401;
	sbus_frame.channels[6] = 0x401;
	*/
	memset(sbus_frame.channels, 0, 18);
	setup_ritimer();

	//Checking for blackbox data on async basis
	fc_blackbox_uart->read_async(100, fc_bb_read_del);
	flight_cont_event_t current_event;
	for(;;) {
		xQueueReceive(flight_cont_event_queue, &current_event, portMAX_DELAY);
		switch (current_event.type) {
		case BLACKBOX_READ:
			// Receive data frame from blackbox
			//read_from_uart();
			break;
		case FLIGHT_COMMAND:
			//Do operations to send command
			//to flight controller
		default:
			break;
		}
	}
}

void set_frame_channel_cmd(uint8_t channel, uint16_t value) {
	sbus_frame.channels[channel] = value & SBUS_CHANNEL_MASK;
}

void add_event_to_queue(flight_cont_event_t event) {
	xQueueSendToBack(flight_cont_event_queue, &event, 0);
}

void setup_ritimer(void) {
	Chip_RIT_Init(LPC_RITIMER);
	Chip_RIT_TimerDebugEnable(LPC_RITIMER);
	Chip_RIT_SetTimerInterval(LPC_RITIMER, SBUS_INTERVAL_MS);
	Chip_RIT_Enable(LPC_RITIMER);
	NVIC_EnableIRQ(RITIMER_IRQn); // TODO: Make this a very high priority interrupt
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

static void sbus_frame_written_handler(UartError status) {
	// TODO: Check status
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

extern "C" {
	using namespace flight_controller_task;
	void RIT_IRQHandler(void) {
		Chip_RIT_ClearInt(LPC_RITIMER);
		uint8_t raw_frame[SBUS_FRAME_LEN];
		sbus_frame.serialize(raw_frame);
		fc_sbus_uart->write_async(raw_frame, SBUS_FRAME_LEN, sbus_written_del);
	}
}
