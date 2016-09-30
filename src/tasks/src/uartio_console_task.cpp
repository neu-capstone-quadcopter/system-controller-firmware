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
#define NEW_LINE_LENGTH 4
#define MAX_COMMAND_INPUT_SIZE 255
#define ASCII_DEL ( 0x7F )

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

	void uartReadHandler(uint8_t data);
	static void task_loop(void *p);

	//Define our Queue
	QueueHandle_t event_queue;
	TaskHandle_t task_handle;
	UartIo *uart;
	char * new_line = "\r\n\r\n";

	//Command line variables
	static char cmd_input_string [MAX_COMMAND_INPUT_SIZE];
	static char cmd_last_input_string [MAX_COMMAND_INPUT_SIZE];
	uint8_t cmd_input_index = 0;

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
			//Handle reading and writing at the same time
			uart->readCharAsync(uartReadHandler);
			xQueueReceive(event_queue, &current_event, portMAX_DELAY);

			switch(current_event.type)
			{
			//If we typed a character, write it to the screen
			case READ_EVENT:
				//Okay -- a character has been typed to the screen. Let's check what that character is.
				//If we pressed return key
				if(current_event.data[0] == '\n' || current_event.data[0] == '\r')
				{
					uart->write((uint8_t*)new_line, NEW_LINE_LENGTH);
					//Execute whatever the command is
					break;
				}
				else
				{
					//Wasn't a newline character -- ignore carriage returns
					if(current_event.data[0] == '\r')
					{
						//IGNORE
						break;
					}
					else if(current_event.data[0] == '\b' || current_event.data[0] == ASCII_DEL)
					{
						//If backspace was pressed, delete the last character from both
						//the screen and the input string
						if(cmd_input_index > 0)
						{
							cmd_input_index--;
							cmd_input_string[cmd_input_index] = '\0';
						}
						char *back = "\b \b";
						uart->write((uint8_t*)back, 3);
						break;
					}
					else
					{
						//A character was written - write it to both the console and
						//the input string
						if((current_event.data[0] >= ' ') && (current_event.data[0] <= '~'))
						{
							if(cmd_input_index < MAX_COMMAND_INPUT_SIZE)
							{
								cmd_input_string [cmd_input_index] = current_event.data[0];
								cmd_input_index++;

								uart->writeChar(current_event.data[0]);
								break;
							}
						}
					}

				}

			//If we get a message from another task -- we'll write it to the screen
			case DEBUG_MESSAGE_EVENT:
				uart->write(current_event.data, current_event.length);
				break;
			}

		}
	}

	//Will save our data into the queue
	void uartReadHandler(uint8_t data)
	{
		Event e;
		e.type = READ_EVENT;
		e.length = 1;

		e.data[0] = data;
		xQueueSendToBackFromISR(event_queue, &e, 0);
	}

	void sendDebugMessage(uint8_t *data, size_t length)
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

	void interpretFunctionCall(char* input_string, char* output_string)
	{
		//the first thing we need to do is interpret the command string

	}

	uint8_t getNumberOfParameters(char* input_string)
	{

	}
}

