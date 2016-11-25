/*
 * nav_computer_task.cpp
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
#include <flight_controller_task.hpp>

#include <stdlib.h>

#define EVENT_QUEUE_DEPTH 16
#define MAX_BUFFER_SIZE 575

#define USE_DMA false
#define HEADER_LEN 4


namespace nav_computer_task {

const uint16_t SYNC_BYTES = 0x91D3;

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
void send_flight_controls(monarcpb_NavCPUToSysCtrl message);

UartIo* nav_uart;
static TaskHandle_t task_handle;
TimerHandle_t timer;
GpdmaManager *dma_man;
GpdmaChannel *dma_channel_tx;
GpdmaChannel *dma_channel_rx;
static SemaphoreHandle_t protobuff_semaphore;

static monarcpb_SysCtrlToNavCPU current_frame;

__BSS(RAM2)
static uint8_t serialization_buffer[MAX_BUFFER_SIZE];
static uint8_t nav_data_buffer[MAX_BUFFER_SIZE];

auto read_len = dlgt::make_delegate(&read_len_handler);
auto read_data = dlgt::make_delegate(&read_data_handler);

void start() {
	nav_uart = hal::get_driver<UartIo>(hal::NAV_COMPUTER);
	nav_uart->allocate_buffers(128, 128);
	nav_uart->enable_interrupts();
	dma_man = hal::get_driver<GpdmaManager>(hal::GPDMA_MAN);
	dma_channel_tx = dma_man->allocate_channel(2);
	dma_channel_rx = dma_man->allocate_channel(3);

	// Enabled DMA (Optional)
	if (USE_DMA) {
		nav_uart->bind_dma_channels(dma_channel_tx, dma_channel_rx);
		nav_uart->set_transfer_mode(UART_XFER_MODE_DMA);
	}
	xTaskCreate(task_loop, "nav computer", 800, NULL, 5, &task_handle);
	nav_event_queue = xQueueCreate(EVENT_QUEUE_DEPTH, sizeof(nav_event_t));
	protobuff_semaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(protobuff_semaphore);

	nav_uart->set_baud_fractional(0xA3, 0x5, 0x0, SYSCTL_CLKDIV_4);
}

void initialize_timers() {
	timer = xTimerCreate("NavTimer", 3, pdTRUE, NULL, timer_handler);
	xTimerStart(timer, 0);
}

static void task_loop(void *p) {
	initialize_timers();
	nav_uart->read_async(HEADER_LEN, read_len);
	current_frame = monarcpb_SysCtrlToNavCPU_init_zero;
	nav_event_t event;
	for(;;) {
		xQueueReceive(nav_event_queue, &event, portMAX_DELAY);
		switch (event.type) {
		case LoopTriggerEvent::SEND_FRAME:
			// Package and send data frame
			xSemaphoreTake(protobuff_semaphore, 5);
			serialize_and_send_frame(current_frame);
			current_frame = monarcpb_SysCtrlToNavCPU_init_zero;
			xSemaphoreGive(protobuff_semaphore);
			break;
		case LoopTriggerEvent::PROCESS_READ:
			distribute_data(nav_data_buffer/*event.buffer*/, event.length);
			break;
		default:
			break;
		}
	}
}

void serialize_and_send_frame(monarcpb_SysCtrlToNavCPU frame) {
	const uint8_t DATA_OFFSET = 4;

	pb_ostream_t stream = pb_ostream_from_buffer(serialization_buffer + DATA_OFFSET, MAX_BUFFER_SIZE - DATA_OFFSET);
	pb_encode(&stream, monarcpb_SysCtrlToNavCPU_fields, &frame);

	serialization_buffer[0] = static_cast<uint8_t>(SYNC_BYTES);
	serialization_buffer[1] = static_cast<uint8_t>(SYNC_BYTES >> 8);
	serialization_buffer[2] = stream.bytes_written;
	serialization_buffer[3] = stream.bytes_written >> 8;
	nav_uart->write(serialization_buffer, stream.bytes_written + DATA_OFFSET);
}

void add_event_to_queue(nav_event_t event) {
	xQueueSendToBack(nav_event_queue, &event, 0);
}

void add_event_to_queue_isr(nav_event_t event) {
	BaseType_t task_woken = pdFALSE;
	xQueueSendToBackFromISR(nav_event_queue, &event, &task_woken);
	if(task_woken) {
		vPortYield();
	}
}

void add_message_to_outgoing_frame(OutgoingNavComputerMessage &msg) {
	xSemaphoreTake(protobuff_semaphore, 4);
	msg.serialize_protobuf(current_frame);
	xSemaphoreGive(protobuff_semaphore);
}

void distribute_data(uint8_t* data, uint16_t length) {
	// Decode message.
	pb_istream_t stream = pb_istream_from_buffer(data, (uint8_t) length);
	monarcpb_NavCPUToSysCtrl message = monarcpb_NavCPUToSysCtrl_init_zero;
	pb_decode(&stream, monarcpb_NavCPUToSysCtrl_fields, &message);
	// TODO: Distribute data to sysctrl nodes as needed.
	send_flight_controls(message);
	//delete[] data;
}

void send_flight_controls(monarcpb_NavCPUToSysCtrl message) {
	if(message.has_control) {
		if (message.control.has_roll)
			flight_controller_task::set_frame_channel_cmd(0, message.control.roll);
		if (message.control.has_pitch)
			flight_controller_task::set_frame_channel_cmd(1, message.control.pitch);
		if (message.control.has_yaw)
			flight_controller_task::set_frame_channel_cmd(2, message.control.yaw);
		if (message.control.has_throttle)
			flight_controller_task::set_frame_channel_cmd(3, message.control.throttle);
	}
}

static void read_len_handler(UartError status, uint8_t *data, uint16_t len) {
	uint16_t sync;
	if (USE_DMA)
		sync = data[2] << 8 | data[1];
	else
		sync = data[1] << 8 | data[0];

	if (sync != SYNC_BYTES) {
		TimerHandle_t timer_sync = xTimerCreate("SyncTimer", 1, pdTRUE, NULL, [](TimerHandle_t xTimer) {
			nav_uart->read_async(HEADER_LEN, read_len);
			xTimerDelete(xTimer, 10);
		});
		xTimerStartFromISR(timer_sync, 0);
		delete[] data;
		return;
	}
	uint16_t length;
	if (USE_DMA)
		length = data[4] << 8 | data[3];
	else
		length = data[3] << 8 | data[2];
	nav_uart->read_async(length, read_data);
	delete[] data;
}

static void read_data_handler(UartError status, uint8_t *data, uint16_t len) {
	nav_event_t event;
	event.type = LoopTriggerEvent::PROCESS_READ;
	memcpy(nav_data_buffer, data, len);
	//event.buffer = data;
	event.length = len;
	add_event_to_queue_isr(event);

	nav_uart->read_async(HEADER_LEN, read_len);
	delete[] data;
}

static void timer_handler(TimerHandle_t xTimer) {
	nav_event_t event;
	event.type = LoopTriggerEvent::SEND_FRAME;
	add_event_to_queue_isr(event);
}


} // End nav_computer_task namespace.


