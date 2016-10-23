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
#define STREAM_BUFFER_SIZE 200
#define ERROR_VAL 999999

namespace flight_controller_task {

class Stream {
public:

	Stream()
	{
		read_ptr = stream_buffer;
		write_ptr = stream_buffer;
	}

	void addToStream(uint8_t* data, uint8_t len)
	{

		if(len >= STREAM_BUFFER_SIZE)
		{
			//Potentially throw some error if len too large for buffer
		}

		if((write_ptr + len) <= (stream_buffer + STREAM_BUFFER_SIZE))
		{
			memcpy(write_ptr, data, len);
			write_ptr += len;
		}
		else
		{
			uint8_t rollover = (uint8_t)((write_ptr + len) - (stream_buffer + STREAM_BUFFER_SIZE));
			uint8_t amt_to_write = (reinterpret_cast<uint32_t>(write_ptr) + len) - rollover;
			memcpy(write_ptr, data, amt_to_write);
			write_ptr = stream_buffer;
			memcpy(write_ptr, data + amt_to_write, rollover);
			write_ptr += rollover;
		}


	}

	uint8_t popFromStream()
	{
		//Pop one byte at a time based on our read ptr
		uint8_t val = *read_ptr;
		if(read_ptr < (stream_buffer + STREAM_BUFFER_SIZE))
			read_ptr ++;
		else
			read_ptr = stream_buffer;

		return val;
	}

private:
	uint8_t stream_buffer[STREAM_BUFFER_SIZE]; //Use char* to leverage str functions
	uint8_t *read_ptr;
	uint8_t *write_ptr;
};

struct SBusFrame{
	uint16_t channels[18];
	bool frame_lost;
	bool fail_safe_activated;
};

static void task_loop(void *p);
void send_data(uint8_t *data);
void write_to_sbus(uint8_t *data, uint8_t len);
void read_from_uart();
void fc_bb_read_handler(std::shared_ptr<UartReadData> read_status);

UartIo* fc_blackbox_uart;
UartIo* fc_sbus_uart;
static TaskHandle_t task_handle;

auto fc_bb_read_del = dlgt::make_delegate(&fc_bb_read_handler);

Stream blackbox_stream;

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
			// Now that we have gotten read event, we know that we
			// have new data in our stream
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

uint8_t* serializeSbusFrame(SBusFrame s)
{
	uint8_t* raw_frame = new uint8_t[25];
	raw_frame[0] = 0xF0;
	raw_frame[1] = s.channels[0] >> 3;
	raw_frame[2] = ((s.channels[0] & 0xF3) << 5) & s.channels[1] ;
}

void fc_bb_read_handler(std::shared_ptr<UartReadData> read_status)
	{
		//std::unique_ptr<uint8_t[]> read_data = std::move(read_status->data);
		uint16_t len = 16;
		uint8_t *data;

		//Add data to stream
		blackbox_stream.addToStream(data, len);

		//Create read event
		flight_cont_event_t e;
		e.type = BLACKBOX_READ;;

		//Add to the queue
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
