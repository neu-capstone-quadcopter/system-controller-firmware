/*
 * uart_console_task.cpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nate
 */

#include <uartio.hpp>
#include <cstddef>
#include <cstring>
#include "uart_console_task.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "hal.hpp"

#define EVENT_QUEUE_DEPTH 10

namespace uart_task {

	enum event_type
	{
		READ_EVENT,
		DEBUG_MESSAGE_EVENT
	};

	struct Event
	{
		event_type type;
		uint8_t data[UART_EVENT_DATA_MAX_LEN];
		size_t length;
	};

	void uart_read_handler(uint8_t data);
	static void task_loop(void *p);

	//Define our Queue
	QueueHandle_t event_queue;
	TaskHandle_t task_handle;
	UartIo *uart;

	void start(void) {
		// Retrieve driver instances from HAL
		uart = static_cast<UartIo*>(hal::get_driver(hal::CONSOLE_UART));

		//Instantiate Queue
		event_queue = xQueueCreate(EVENT_QUEUE_DEPTH, sizeof(Event));

		xTaskCreate(task_loop, "uart task", 1536, NULL, 2, &task_handle);
	}

	static void task_loop(void *p) {
		Event current_event;

		for(;;) {
			uart->read_char_async(uart_read_handler);
			xQueueReceive(event_queue, &current_event, portMAX_DELAY);

			switch(current_event.type)
			{
			case READ_EVENT:
				uart->write_char(current_event.data[0]);
				break;

			case DEBUG_MESSAGE_EVENT:
				uart->write(current_event.data, current_event.length);
				break;
			}
		}
	}

	//Will save our data into the queue
	void uart_read_handler(uint8_t data)
	{
		Event e;
		e.type = READ_EVENT;
		e.length = 1;

		e.data[0] = data;
		xQueueSendToBackFromISR(event_queue, &e, 0);
	}

	void send_debug_message(uint8_t *data, size_t length)
	{
		Event e;
		e.type = DEBUG_MESSAGE_EVENT;
		e.length = length;

		if(length <= UART_EVENT_DATA_MAX_LEN)
		{
			memcpy(e.data, data, length);
			xQueueSendToBack(event_queue, &e, 0);
		}
	}
}

