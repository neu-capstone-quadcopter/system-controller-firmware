/*
 * console_functions.hpp
 *
 *  Created on: Oct 14, 2016
 *      Author: nigil
 */

#ifndef UTIL_INC_CONSOLE_FUNCTIONS_HPP_
#define UTIL_INC_CONSOLE_FUNCTIONS_HPP_

#include <cstdio>
#include <cstdlib>

#include "board.hpp"
#include "hal.hpp"

namespace console_task {
#define MAX_COMMAND_PARAMS 5

	const char *NOT_ENOUGH_ARGS_STR = "Not enough arguments\r\n";
	const char *INVALID_ARGS_STR = "Invalid argument\r\n";

	/*
	 * set_pin_dir <port> <pin> <state>
	 * Set a GPIO on the MCU to input or output
	 * state: INPUT or OUTPUT
	 */
	void set_pin_dir(char* output_string, uint8_t argc, char** argv) {
		if(argc == 4) {
			if(!strcmp(argv[3], "OUTPUT")) {
				Chip_GPIO_WriteDirBit(LPC_GPIO, atoi(argv[1]), atoi(argv[2]), true);
			}
			else if(!strcmp(argv[3], "INPUT")) {
				Chip_GPIO_WriteDirBit(LPC_GPIO, atoi(argv[1]), atoi(argv[2]), false);
			}
			else {
				strcpy(output_string, INVALID_ARGS_STR);
			}
		}
		else {
			strcpy(output_string, NOT_ENOUGH_ARGS_STR);
		}
	}

	/*
	 * write_pin <port> <pin> <state>
	 * Set a GPIO output on the MCU
	 * state: HIGH or LOW
	 */
	void write_pin(char* output_string, uint8_t argc, char** argv) {
		if(argc == 4) {
			if(!strcmp(argv[3], "HIGH")) {
				Chip_GPIO_WritePortBit(LPC_GPIO, atoi(argv[1]), atoi(argv[2]), true);
			}
			else if(!strcmp(argv[3], "LOW")) {
				Chip_GPIO_WritePortBit(LPC_GPIO, atoi(argv[1]), atoi(argv[2]), false);
			}
			else {
				strcpy(output_string, argv[2]);
			}
		}
		else {
			strcpy(output_string, NOT_ENOUGH_ARGS_STR);
		}
	}

	/*
	 * read_pin <port> <pin>
	 * Read a GPIO on the MCU
	 */
	void read_pin(char* output_string, uint8_t argc, char** argv) {
		if(argc == 3) {
			sprintf(output_string, "%d\r\n", Chip_GPIO_ReadPortBit(LPC_GPIO, atoi(argv[1]), atoi(argv[2])));
		}
		else {
			strcpy(output_string, NOT_ENOUGH_ARGS_STR);
		}
	}

	void set_led(char* output_string, uint8_t argc, char** argv)
	{
		if(argc == 2) {
			if(!strcmp(argv[1],"on"))
			{
				strcpy(output_string,"Turning LED on...\r\n");
				Chip_GPIO_WritePortBit(LPC_GPIO, 2, 10, true);
			}
			else if(!strcmp(argv[1],"off"))
			{
				strcpy(output_string,"Turning LED off...\r\n");
				Chip_GPIO_WritePortBit(LPC_GPIO, 2, 10, false);
			}
			else
			{
				strcpy(output_string, "Invalid Parameter -- options: \'on\' or \'off\'...\r\n");
			}
		}
		else {
			strcpy(output_string, NOT_ENOUGH_ARGS_STR);
		}
	}

	void get_mem_info(char* output_string, uint8_t argc, char** argv)
	{
		sprintf(output_string, "Free Memory: %d\r\n"
				"Memory Watermark: %d\r\n",
				xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
	}

	void get_task_info(char* output_string, uint8_t argc, char** argv)
	{
		vTaskList(output_string);
	}

	void get_runtime_info(char* output_string, uint8_t argc, char** argv)
	{
		vTaskGetRunTimeStats(output_string);
	}

	void start_trace(char* output_string, uint8_t argc, char** argv)
	{
		uiTraceStart();
	}

#ifdef IS_FLIGHT_PCB
	void set_load_switch(char* output_string, uint8_t argc, char** argv)
	{
		if(argc == 3) {
			uint8_t port;
			uint8_t pin;
			bool state;
			if(!strcmp(argv[1],"navcmp"))
			{
				strcpy(output_string,"Setting Navigation Computer:");
				port = NAVCMP_EN_PORT;
				pin = NAVCMP_EN_PIN;
			}
			else if(!strcmp(argv[1],"fltctl"))
			{
				strcpy(output_string,"Setting Flight Controller:");
				port = FLTCTL_EN_PORT;
				pin = FLTCTL_EN_PIN;
			}
			else if(!strcmp(argv[1],"gps"))
			{
				strcpy(output_string,"Setting GPS:");
				port = GPS_EN_PORT;
				pin = GPS_EN_PIN;
			}
			else if(!strcmp(argv[1],"radio"))
			{
				strcpy(output_string,"Setting Radio:");
				port = RADIO_EN_PORT;
				pin = RADIO_EN_PIN;
			}
			else
			{
				strcpy(output_string, "Invalid Parameter -- options: \'navcmp\' or \'fltctl\' or \'gps\' or \'radio\'...\r\n");
				return;
			}

			if(!strcmp(argv[2],"enable"))
			{
				strcpy(output_string,"enabled...\r\n");
				state = true;
			}
			else if(!strcmp(argv[2],"disable"))
			{
				strcpy(output_string,"disabled...\r\n");
				state = false;
			}
			else
			{
				strcpy(output_string, "Invalid Parameter -- options: \'enable\' or \'disable\'...\r\n");
				return;
			}

			Chip_GPIO_WritePortBit(LPC_GPIO, port, pin, state);
		}
		else {
			strcpy(output_string, NOT_ENOUGH_ARGS_STR);
		}
	}
#endif

	typedef void (*CommandFunction)(char*,uint8_t,char**);

	struct CommandDescriptor {
		const char *call_string;
		const CommandFunction function;
	};

	const CommandDescriptor command_list[] = {
			{"set_pin_dir", &set_pin_dir},
			{"write_pin", &write_pin},
			{"read_pin", &read_pin},
			{"set_led", &set_led},
			{"get_mem_info", &get_mem_info},
			{"get_task_info", &get_task_info},
			{"get_runtime_info", &get_runtime_info},
			{"start_trace", &start_trace},
#ifdef IS_FLIGHT_PCB
			{"set_load_switch", &start_load_switch},
#endif
	};

#define NUMBER_COMMANDS (sizeof(command_list) / sizeof(CommandDescriptor))
}


#endif /* UTIL_INC_CONSOLE_FUNCTIONS_HPP_ */
