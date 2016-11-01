/*
<<<<<<< HEAD
 * nav_computer_task.cpp
=======
 * nav_computer_task.cp
>>>>>>> origin/nl-message-types
 *
 *  Created on: Sep 27, 2016
 *      Author: bsoper
 */

#include "nav_computer_task.hpp"
#include "gpdma.hpp"
#include "hal.hpp"
#include "uartio.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "api.pb.h"
#include <cr_section_macros.h>
#include <flight_ctrl_message.hpp>

#include <stdlib.h>

#define EVENT_QUEUE_DEPTH 8
#define MAX_BUFFER_SIZE 575

namespace nav_computer_task {

static void task_loop(void *p);
//void send_data(sensor_task::adc_values_t data);
//void package_data(sensor_task::adc_values_t data, monarcpb_SysCtrlToNavCPU &message);
void write_to_uart(uint8_t *data, uint16_t len);
void read_from_uart();
static void timer_handler(TimerHandle_t xTimer);
void distribute_data(uint8_t *data, uint16_t length);
static void read_len_handler(UartError status, uint8_t *data, uint16_t len);
static void read_data_handler(UartError status, uint8_t *data, uint16_t len);
void serialize_and_send_frame(monarcpb_SysCtrlToNavCPU frame);
// TODO: Crate a function that serializes the frame
//void send_data(sensor_task::adc_values_t data); TODO: Make this just send data
FlightCtrlMessage create_flight_message(monarcpb_NavCPUToSysCtrl message);

UartIo* nav_uart;
static TaskHandle_t task_handle;
TimerHandle_t timer;
GpdmaManager *dma_man;
GpdmaChannel *dma_channel_tx;
GpdmaChannel *dma_channel_rx;

static monarcpb_SysCtrlToNavCPU current_frame;

__BSS(RAM2)
static uint8_t serialization_buffer[MAX_BUFFER_SIZE];

auto read_len = dlgt::make_delegate(&read_len_handler);
auto read_data = dlgt::make_delegate(&read_data_handler);

void start() {
	nav_uart = hal::get_driver<UartIo>(hal::NAV_COMPUTER);
	nav_uart->allocate_buffers(128, 128);
	dma_man = hal::get_driver<GpdmaManager>(hal::GPDMA_MAN);
	dma_channel_tx = dma_man->allocate_channel(0);
	dma_channel_rx = dma_man->allocate_channel(1);

	// Enabled DMA (Optional)
	nav_uart->bind_dma_channels(dma_channel_tx, dma_channel_rx);
	//nav_uart->set_transfer_mode(UART_XFER_MODE_DMA);
	xTaskCreate(task_loop, "nav computer", 1200, NULL, 2, &task_handle);
	nav_event_queue = xQueueCreate(EVENT_QUEUE_DEPTH, sizeof(nav_event_t));

	//nav_uart->set_baud(230400);
	Chip_UART_EnableDivisorAccess(nav_uart->uart);
	nav_uart->uart->FDR = 0xA3;
	nav_uart->uart->DLL = 0x5;
	nav_uart->uart->DLM = 0x0;
	Chip_UART_DisableDivisorAccess(nav_uart->uart);

}

void initialize_timers() {
	timer = xTimerCreate("NavTimer", 3, pdTRUE, NULL, timer_handler);
	xTimerStart(timer, 0);
}

static void task_loop(void *p) {
	initialize_timers();
	nav_uart->read_async(4, read_len);
	current_frame = monarcpb_SysCtrlToNavCPU_init_zero;
	nav_event_t event;
	for(;;) {
		xQueueReceive(nav_event_queue, &event, portMAX_DELAY);
		char buffer[20] = "AAAAAAAAAA";
		switch (event.type) {
		case LoopTriggerEvent::SEND_FRAME:
			// TODO: Serialize data
			// TODO: Send data

			// Package and send data frame
			//send_data(current_event.data);
			//read_from_uart();
			serialize_and_send_frame(current_frame);
			current_frame = monarcpb_SysCtrlToNavCPU_init_zero;
			//write_to_uart((uint8_t*)serialization_buffer, 20);
			break;
		case LoopTriggerEvent::PROCESS_READ:
			distribute_data(event.buffer, event.length);
			break;
		default:
			break;
		}
	}
}

void serialize_and_send_frame(monarcpb_SysCtrlToNavCPU frame) {
	frame.has_telemetry = true;
	frame.telemetry.has_atmospheric_pressure = true;
	frame.telemetry.atmospheric_pressure = 103;
	pb_ostream_t stream = pb_ostream_from_buffer(serialization_buffer, MAX_BUFFER_SIZE);//sizeof(serialization_buffer));
	pb_encode(&stream, monarcpb_SysCtrlToNavCPU_fields, &frame);

	write_to_uart(serialization_buffer, stream.bytes_written);
	return;
}

void add_event_to_queue(nav_event_t event) {
	xQueueSendToBack(nav_event_queue, &event, 0);
}

void add_message_to_outgoing_frame(OutgoingNavComputerMessage &msg) {
	msg.serialize_protobuf(current_frame);
}

void write_to_uart(uint8_t *data, uint16_t len) {
	uint16_t sync_bytes = 0x91D3;
	nav_uart->write((uint8_t*) &sync_bytes, 2);
	nav_uart->write((uint8_t*) &len, 2);
	nav_uart->write(data, (uint8_t) len);
}

void distribute_data(uint8_t* data, uint16_t length) {
	// Decode message.
	pb_istream_t stream = pb_istream_from_buffer(data, (uint8_t) length);
	monarcpb_NavCPUToSysCtrl message = monarcpb_NavCPUToSysCtrl_init_zero;
	pb_decode(&stream, monarcpb_NavCPUToSysCtrl_fields, &message);
	// TODO: Distribute data to sysctrl nodes as needed.
	FlightCtrlMessage flight_message = create_flight_message(message);
}

FlightCtrlMessage create_flight_message(monarcpb_NavCPUToSysCtrl message) {
	FlightCtrlMessage flight_message;
	flight_message.roll = message.control.roll;
	flight_message.pitch = message.control.pitch;
	flight_message.yaw = message.control.yaw;
	flight_message.elevation = message.control.throttle;
	return flight_message;
}

static void read_len_handler(UartError status, uint8_t *data, uint16_t len) {
	uint16_t sync = data[1] << 8 | data[0];
	if (sync != 0x91D3) {
		TimerHandle_t timer_sync = xTimerCreate("SyncTimer", 1, pdTRUE, NULL, [](TimerHandle_t xTimer) {
			nav_uart->read_async(4, read_len);
			xTimerDelete(xTimer, 10);
		});
		xTimerStart(timer_sync, 0);
		delete[] data;
		return;
	}
	uint16_t length = data[3] << 8 | data[2];
	nav_uart->read_async(length, read_data);
	delete[] data;
}

static void read_data_handler(UartError status, uint8_t *data, uint16_t len) {
	nav_event_t event;
	event.type = LoopTriggerEvent::PROCESS_READ;
	event.buffer = data;
	event.length = len;
	add_event_to_queue(event);

	nav_uart->read_async(4, read_len);
	delete[] data;
}

static void timer_handler(TimerHandle_t xTimer) {
	nav_event_t event;
	event.type = LoopTriggerEvent::SEND_FRAME;
	add_event_to_queue(event);
}


} // End nav_computer_task namespace.


