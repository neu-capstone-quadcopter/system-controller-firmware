/*
 * uart_console_task.cpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nate
 */


#include <cstddef>
#include <stdio.h>
#include <ctype.h>
#include <cstring>
#include <memory>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "hal.hpp"
#include "uartio.hpp"

#include "uart_console_task.hpp"
#include "console_functions.hpp"


#define EVENT_QUEUE_DEPTH 10
#define NEW_LINE_LENGTH 4
#define MAX_COMMAND_INPUT_SIZE 255
#define MAX_COMMAND_OUTPUT_SIZE 255
#define MAX_COMMAND_PARAM_SIZE 255
#define ASCII_DEL ( 0x7F )

namespace console_task {

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

	static void uart_read_handler(std::shared_ptr<UartReadData> read_status);
	static void task_loop(void *p);
	void interpret_command_call(char*, char*, char*, uint8_t);
	CommandFunction command_lookup(char* command);
	void tokanize_command_string(char *str, uint8_t &argc, char** argv);

	QueueHandle_t event_queue;
	TaskHandle_t task_handle;
	UartIo *uart;

	//Command line variables
	static char cmd_input_string[MAX_COMMAND_INPUT_SIZE] = {0};
	static char cmd_output_string[MAX_COMMAND_OUTPUT_SIZE] = {0};
	static char cmd_param_string[MAX_COMMAND_PARAM_SIZE] = {0};
	uint8_t cmd_input_index = 0;
	const char * new_line = "\r\n";

	auto uart_read_del = dlgt::make_delegate(&uart_read_handler);

	void start(void) {
		// Retrieve driver instances from HAL
		uart = hal::get_driver<UartIo>(hal::CONSOLE_UART);
		uart->allocate_buffers(128, 32);

		//Instantiate Queue
		event_queue = xQueueCreate(EVENT_QUEUE_DEPTH, sizeof(Event));

		xTaskCreate(task_loop, "console task", 1536, NULL, 2, &task_handle);
	}

	static void task_loop(void *p) {
		Event current_event;

		uart->read_async(1, uart_read_del);
		for(;;) {
			xQueueReceive(event_queue, &current_event, portMAX_DELAY);

			switch(current_event.type)
			{
			//If we typed a character, write it to the screen
			case READ_EVENT:
				//If we pressed return key
				if(current_event.data[0] == '\n' || current_event.data[0] == '\r')
				{
					//Write newline/execute command
					uart->write((uint8_t*)new_line, NEW_LINE_LENGTH);
					interpret_command_call(cmd_input_string, cmd_output_string, cmd_param_string, (uint8_t)cmd_input_index);

					//reset control strings
					cmd_input_index = 0;
					memset(cmd_input_string, 0x00, MAX_COMMAND_INPUT_SIZE);
					memset(cmd_output_string, 0x00, MAX_COMMAND_OUTPUT_SIZE);
					memset(cmd_param_string, 0x00, MAX_COMMAND_PARAM_SIZE);
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
						const char *back = "\b \b";
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
								cmd_input_string[cmd_input_index] = (char)current_event.data[0];
								cmd_input_index++;

								uart->write(current_event.data, 1);
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

	void uart_read_handler(std::shared_ptr<UartReadData> read_status)
	{
		std::unique_ptr<uint8_t[]> read_data = std::move(read_status->data);

		Event e;
		e.type = READ_EVENT;
		e.length = 1;

		e.data[0] = read_data[0];
		xQueueSendToBackFromISR(event_queue, &e, 0);
		uart->read_async(1, uart_read_del);
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

	void interpret_command_call(char* input_string, char* output_string, char* param_string, uint8_t length)
	{
		/*
		//Generate command string
		int space_index = 0;
		char command[MAX_COMMAND_INPUT_SIZE] = {0};

		//Everything up to first space is command
		for(int i = 0; i <= length; i++)
		{
			char curr_char = input_string[i];
			if(isspace(curr_char))
			{
				//Get current command/param string
				space_index = i;
				strncpy(command,input_string,space_index);

				for(int j = space_index + 1; j < MAX_COMMAND_INPUT_SIZE; j++)
				{
					int curr_index = j - space_index -1;
					param_string[curr_index] = input_string[j];
				}

				break;
			}
		}

		//If no params passed, command is whole input string
		if(space_index == 0)
			strcpy(command, input_string);
		*/

		uint8_t cmd_argc;
		char *cmd_argv[MAX_COMMAND_PARAMS];
		tokanize_command_string(input_string, cmd_argc, cmd_argv);

		CommandFunction command_function = command_lookup(cmd_argv[0]);

		if(command_function == NULL)
		{
			strcpy(output_string,"Invalid Command\r\n");
		}
		else
		{
			command_function(output_string, cmd_argc, cmd_argv);
		}

		uart->write(reinterpret_cast<uint8_t*>(output_string), (size_t)MAX_COMMAND_OUTPUT_SIZE);
	}

	/*
	 * Looks up command from command_list array. Returns NULL if function doesn't exist
	 */
	CommandFunction command_lookup(char* command)
	{
		for(uint16_t i = 0; i < NUMBER_COMMANDS; i++) {
			if(strcmp(command, command_list[i].call_string) == 0) {
				return command_list[i].function;
			}
		}
		return NULL;
	}

	void tokanize_command_string(char *str, uint8_t &argc, char** argv) {
		for(char *tok = strtok(str, " "); tok != NULL; tok = strtok(NULL, " ")) {
			argv[argc++] = tok;
			if(argc == MAX_COMMAND_PARAMS) {
				break;
			}
		}
	}
}

