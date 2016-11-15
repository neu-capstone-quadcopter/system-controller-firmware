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
#include "blackbox_parser.hpp"

#include <stdlib.h>
#include <cstring>
#include <cr_section_macros.h>

#include "gpdma.hpp"

#define EVENT_QUEUE_DEPTH 8
#define MAX_BUFFER_SIZE 255
#define ERROR_VAL 999999

namespace flight_controller_task {

#define SBUS_CHANNEL_MASK 0x07ff
#define SBUS_CHANNEL_BIT_LEN 11
#define SBUS_CHANNEL_17 16
#define SBUS_CHANNEL_18 17

const uint8_t SBUS_INTERVAL_MS = 14;
const uint8_t SBUS_FRAME_LEN = 25;

struct SBusFrame{
	uint16_t channels[18];
	bool frame_lost;
	bool fail_safe_activated;

	/*
	 * Make a raw data copy of the frame suitable for sending to cleanflight
	 * This function encodes the data in a format ready for sending via S.BUS
	 * S.BUS signal has 16, 11 bit channels along with 2 digital channels. The
	 * data is to be sent in a big endian format while the LPC1759 is little endian
	 * as such after building a byte we need to reverse the bits contained.
	 */
	void serialize(uint8_t* raw_frame) {
		memset(raw_frame, 0, 25);

		raw_frame[0] = 0x0F; // Reversed start byte
		uint8_t byte_idx = 1;
		int8_t start_pos = 0;

		// TODO: Could have just used a packed struct...
		for(uint8_t i = 0; i < 16; i++) {
			uint16_t channel = this->channels[i] & SBUS_CHANNEL_MASK;

			int8_t bits_to_write = 11;
			raw_frame[byte_idx++] |= channel << (start_pos);
			bits_to_write -= 8 - start_pos;

			start_pos = 11 - bits_to_write;
			if(bits_to_write <= 8) {
				raw_frame[byte_idx] |= channel >> (start_pos);
				if(bits_to_write == 8) {
					byte_idx++;
					start_pos = 0;
				}
				else {
					start_pos = bits_to_write;
				}
			}
			else {
				raw_frame[byte_idx++] |= channel >> (start_pos);
				bits_to_write -= 8;
				start_pos = bits_to_write;
				raw_frame[byte_idx] |= channel >> (11 - start_pos);
			}
		}

		if(this->channels[SBUS_CHANNEL_17]) {
			raw_frame[23] |= (1 << 0); // Bit 0 due to reversed nature
		}

		if(this->channels[SBUS_CHANNEL_18]) {
			raw_frame[23] |= (1 << 1); // Bit 1 due to reversed nature
		}
	}
};

static void task_loop(void *p);
void setup_ritimer(void);
void send_data(uint8_t *data);
void write_to_sbus(uint8_t *data, uint8_t len);
void read_from_uart();
void fc_bb_read_handler(UartError status, uint8_t *data, uint16_t len);
static void sbus_frame_written_handler(UartError status);

static UartIo* fc_blackbox_uart;
static UartIo* fc_sbus_uart;
static TaskHandle_t task_handle;
static SBusFrame sbus_frame;

auto fc_bb_read_del = dlgt::make_delegate(&fc_bb_read_handler);
auto sbus_written_del = dlgt::make_delegate(&sbus_frame_written_handler);

GpdmaManager *dma_man;
GpdmaChannel *test_channel_tx;
GpdmaChannel *test_channel_rx;

__BSS(RAM2)
Stream blackbox_stream;

bool added_to_stream = false;

BlackboxParser blackbox_parser;

void start() {
	fc_blackbox_uart = hal::get_driver<UartIo>(hal::FC_BLACKBOX_UART);
	fc_sbus_uart = hal::get_driver<UartIo>(hal::FC_SBUS_UART);
	xTaskCreate(task_loop, "flight controller", 400, NULL, 2, &task_handle);
	fc_blackbox_uart->setFractionalBaud(0xA3, 0xA, 0x0);
	blackbox_stream.allocate();
}

static void task_loop(void *p) {
	fc_blackbox_uart->allocate_buffers(0,150);


	fc_sbus_uart->allocate_buffers(30, 0);
	//fc_sbus_uart->set_baud(100000);
	fc_sbus_uart->setFractionalBaud(0x41, 0xC, 0x0);
	fc_sbus_uart->enable_interrupts();
	fc_sbus_uart->config_data_mode(UART_LCR_WLEN8, UART_LCR_PARITY_EVEN, UART_LCR_SBS_2BIT);
	dma_man = hal::get_driver<GpdmaManager>(hal::GPDMA_MAN);
	test_channel_tx = dma_man->allocate_channel(0);
	test_channel_rx = dma_man->allocate_channel(1);

	// Enabled DMA (Optional)
	fc_sbus_uart->bind_dma_channels(test_channel_tx, test_channel_rx);
	fc_sbus_uart->set_transfer_mode(UART_XFER_MODE_DMA);

	//memset(sbus_frame.channels, 0, 18);
	for(uint8_t i = 0; i< 18; i++) {
		sbus_frame.channels[i] = 1500; //2040;
	}
	setup_ritimer();

	//Checking for blackbox data on async basis
	fc_blackbox_uart->read_async(100, fc_bb_read_del);
	flight_cont_event_t current_event;
	for(;;) {
		//blackbox_parser.decodeFrameType(blackbox_stream);
	}
}

void set_frame_channel_cmd(uint8_t channel, uint16_t value) {
	sbus_frame.channels[channel] = value & SBUS_CHANNEL_MASK;
}

void add_event_to_queue(flight_cont_event_t event) {
	//xQueueSendToBack(flight_cont_event_queue, &event, 0);
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

uint8_t* serializeSbusFrame(SBusFrame s)
{

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

void fc_bb_read_handler(UartError status, uint8_t *data, uint16_t len)
{

		//Add data to stream
		//if(!added_to_stream)
		//{
		blackbox_stream.addToStream(data,len);
			//added_to_stream = true;
		//}
		//blackbox_stream.addToStream(data, len);

		//Create read event
		flight_cont_event_t e;
		e.type = BLACKBOX_READ;
		e.data = data;

		//Add to the queue
		//xQueueSendToBackFromISR(flight_cont_event_queue, &e, 0);
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
		uint8_t raw_frame[SBUS_FRAME_LEN];
		sbus_frame.serialize(raw_frame);
		fc_sbus_uart->write_async(raw_frame, SBUS_FRAME_LEN, sbus_written_del);
		Chip_RIT_ClearInt(LPC_RITIMER);
	}
}

