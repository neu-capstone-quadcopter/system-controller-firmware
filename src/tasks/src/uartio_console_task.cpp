/*
 * uart_console_task.cpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nate
 */

#include <uartio.hpp>
#include <cstddef>
#include <stdio.h>
#include <ctype.h>
#include <cstring>
#include "uart_console_task.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "hal.hpp"

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

	enum function_index
	{
		INVALID,
		SAMPLE_FUNCTION,
		ACTIVATE_LED,
		DEACTIVATE_LED,
		SWITCH_LED
	};

	struct Event
	{
		event_type type;
		uint8_t data[UART_EVENT_DATA_MAX_LEN];
		size_t length;
	};

	typedef void (*commandLineFunction)(char*, char*, char*);

	static void uartReadHandler(std::shared_ptr<UartReadData> read_status);
	static void task_loop(void *p);
	function_index commandFunctionLookup(char*);
	void sampleFunction(char*, char*, char*);
	void activateLED(char*,char*,char*);
	void deactivateLED(char*,char*,char*);
	void switchLED(char*,char*,char*);
	void interpretFunctionCall(char*, char*, char*, uint8_t);

	QueueHandle_t event_queue;
	TaskHandle_t task_handle;
	UartIo *uart;

	//Command line variables
	static char cmd_input_string[MAX_COMMAND_INPUT_SIZE] = {0};
	static char cmd_output_string[MAX_COMMAND_OUTPUT_SIZE] = {0};
	static char cmd_param_string[MAX_COMMAND_PARAM_SIZE] = {0};
	static char last_cmd_input_stringv[MAX_COMMAND_INPUT_SIZE];
	uint8_t cmd_input_index = 0;
	const char * new_line = "\r\n\r\n";

	auto uart_read_del = dlgt::make_delegate(&uartReadHandler);

	//Add any debug function to this array in the same order it appears
	//in the function_index enum
	commandLineFunction cmdFunctions[] = {&sampleFunction,
										  &activateLED,
										  &deactivateLED,
										  &switchLED};

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

		for(;;) {
			//Handle reading and writing at the same time
			uart->read_async(1, uart_read_del);
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
					interpretFunctionCall(cmd_input_string, cmd_output_string, cmd_param_string, (uint8_t)cmd_input_index);

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

	void uartReadHandler(std::shared_ptr<UartReadData> read_status)
	{
		std::unique_ptr<uint8_t[]> read_data = std::move(read_status->data);

		Event e;
		e.type = READ_EVENT;
		e.length = 1;

		e.data[0] = read_data[0];
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

	void interpretFunctionCall(char* input_string, char* output_string, char* param_string, uint8_t length)
	{
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

		//Now let's look up this command using lookup function
		function_index command_index = commandFunctionLookup(command);

		if(command_index == INVALID)
		{
			strcpy(output_string,"Invalid Command\r\n\r\n");
			uart->write((uint8_t*)output_string, (size_t)MAX_COMMAND_OUTPUT_SIZE);
			return;
		}
		else
		{
			//Execute called function -- command_index - 1 because an INVALID flag is the
			//0th element of the enumeration. To actually access function array need to
			//subtract 1
			cmdFunctions[command_index - 1](input_string, output_string, cmd_param_string);
			return;
		}
	}

	void sampleFunction(char* input_string, char* output_string, char* parameter_string)
	{
		strcpy(output_string,"Sample function successfully called.\r\n\r\n");
		uart->write((uint8_t*)output_string, (size_t)MAX_COMMAND_OUTPUT_SIZE);
		return;
	}

	void switchLED(char* input_string, char* output_string, char* parameter_string)
	{
		if(!strcmp(parameter_string,"on"))
		{
			strcpy(output_string,"Turning LED on...\r\n\r\n");
			uart->write((uint8_t*)output_string, (size_t)MAX_COMMAND_OUTPUT_SIZE);
			Chip_GPIO_WritePortBit(LPC_GPIO, 2, 10, true);
		}
		else if(!strcmp(parameter_string,"off"))
		{
			strcpy(output_string,"Turning LED off...\r\n\r\n");
			uart->write((uint8_t*)output_string, (size_t)MAX_COMMAND_OUTPUT_SIZE);
			Chip_GPIO_WritePortBit(LPC_GPIO, 2, 10, false);
		}
		else
		{
			strcpy(output_string, "Invalid Parameter -- options: \'on\' or \'off\'...\r\n\r\n");
			uart->write((uint8_t*)output_string, (size_t)MAX_COMMAND_OUTPUT_SIZE);
		}

	}

	void activateLED(char* input_string, char* output_string, char* parameter_string)
	{
		strcpy(output_string,"Turning LED on...\r\n\r\n");
		uart->write((uint8_t*)output_string, (size_t)MAX_COMMAND_OUTPUT_SIZE);
		Chip_GPIO_WritePortBit(LPC_GPIO, 2, 10, true);
	}

	void deactivateLED(char* input_string, char* output_string, char* parameter_string)
	{
		strcpy(output_string,"Turning LED off...\r\n\r\n");
		uart->write((uint8_t*)output_string, (size_t)MAX_COMMAND_OUTPUT_SIZE);
		Chip_GPIO_WritePortBit(LPC_GPIO, 2, 10, false);
	}

	//Any debug functions must be added to this function else/if in addition
	//to the function_index enum.
	function_index commandFunctionLookup(char* command)
	{
		//Translate command string to a value
		if(strcmp(command,"sampleFunction") == 0)
			return SAMPLE_FUNCTION;
		else if(strcmp(command,"activateLED") == 0)
			return ACTIVATE_LED;
		else if(strcmp(command,"deactivateLED") == 0)
			return DEACTIVATE_LED;
		else if(strcmp(command,"switchLED") == 0)
			return SWITCH_LED;
		else
			return INVALID;
	}
}

