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
#include "flight_controller_task.hpp"
#include "load_switch_rail.hpp"
#include "mb1240.hpp"

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


	void set_fltctl_arming(char* output_string, uint8_t argc, char** argv)
	{
		if(argc == 2) {
			if(argv[1][0] == '1') {
				flight_controller_task::arm_controller();
			} else {
				flight_controller_task::disarm_controller();
			}
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
#ifndef IS_DEBUG_BOARD
	void set_navcmp_pwr(char* output_string, uint8_t argc, char** argv)
	{
		if(argc == 2) {
			bool state;
			if(!strcmp(argv[1],"on"))
				state = true;
			else if(!strcmp(argv[1],"off"))
				state = false;
			else {
				strcpy(output_string, "Invalid Parameter -- options: \'on\' or \'off\'...\r\n");
				return;
			}
			LoadSwitch *load_switch = hal::get_driver<LoadSwitch>(hal::LOAD_SWITCH);
			load_switch->set_load_switch_navcmp(state);
		}
		else
			strcpy(output_string, NOT_ENOUGH_ARGS_STR);
	}

	void set_fltctl_pwr(char* output_string, uint8_t argc, char** argv)
	{
		if(argc == 2) {
			bool state;
			if(!strcmp(argv[1],"on"))
				state = true;
			else if(!strcmp(argv[1],"off"))
				state = false;
			else {
				strcpy(output_string, "Invalid Parameter -- options: \'on\' or \'off\'...\r\n");
				return;
			}
			LoadSwitch *load_switch = hal::get_driver<LoadSwitch>(hal::LOAD_SWITCH);
			load_switch->set_load_switch_fltctl(state);
		}
		else
			strcpy(output_string, NOT_ENOUGH_ARGS_STR);
	}

	void set_gps_pwr(char* output_string, uint8_t argc, char** argv)
	{
		if(argc == 2) {
			bool state;
			if(!strcmp(argv[1],"on"))
				state = true;
			else if(!strcmp(argv[1],"off"))
				state = false;
			else {
				strcpy(output_string, "Invalid Parameter -- options: \'on\' or \'off\'...\r\n");
				return;
			}
			LoadSwitch *load_switch = hal::get_driver<LoadSwitch>(hal::LOAD_SWITCH);
			load_switch->set_load_switch_gps(state);
		}
		else
			strcpy(output_string, NOT_ENOUGH_ARGS_STR);
	}

	void set_radio_pwr(char* output_string, uint8_t argc, char** argv)
	{
		if(argc == 2) {
			bool state;
			if(!strcmp(argv[1],"on"))
				state = true;
			else if(!strcmp(argv[1],"off"))
				state = false;
			else {
				strcpy(output_string, "Invalid Parameter -- options: \'enable\' or \'disable\'...\r\n");
				return;
			}
			LoadSwitch *load_switch = hal::get_driver<LoadSwitch>(hal::LOAD_SWITCH);
			load_switch->set_load_switch_radio(state);
		}
		else
			strcpy(output_string, NOT_ENOUGH_ARGS_STR);
	}

	void get_ultrasonic_range(char* output_string, uint8_t argc, char** argv)
	{
		Mb1240 *ultrasonic_altimeter = hal::get_driver<Mb1240>(hal::ULTRASONIC_ALTIMETER);
		sprintf(output_string, "%d\r\n", ultrasonic_altimeter->get_current_range_mm());
	}

	void set_kill(char* output_string, uint8_t argc, char** argv)
	{
		if(argc == 2) {
			bool state;
			if(!strcmp(argv[1], "1"))
				state = true;
			else if(!strcmp(argv[1], "0"))
				state = false;
			else {
				strcpy(output_string, "Invalid Parameter -- options: \'1\' or \'0\'...\r\n");
				return;
			}
			LoadSwitch *load_switch = hal::get_driver<LoadSwitch>(hal::LOAD_SWITCH);
			load_switch->set_hw_arm(state);
		}
		else
			strcpy(output_string, NOT_ENOUGH_ARGS_STR);
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
			{"set_fltctl_arming", &set_fltctl_arming},
#ifndef IS_DEBUG_BOARD
			{"set_navcmp_pwr", &set_navcmp_pwr},
			{"set_fltctl_pwr", &set_fltctl_pwr},
			{"set_gps_pwr", &set_gps_pwr},
			{"set_radio_pwr", &set_radio_pwr},
			{"get_ultrasonic_range", &get_ultrasonic_range},
#endif
	};

#define NUMBER_COMMANDS (sizeof(command_list) / sizeof(CommandDescriptor))
}


#endif /* UTIL_INC_CONSOLE_FUNCTIONS_HPP_ */
