/*
 * console_functions.hpp
 *
 *  Created on: Oct 14, 2016
 *      Author: nigil
 */

#ifndef UTIL_INC_CONSOLE_FUNCTIONS_HPP_
#define UTIL_INC_CONSOLE_FUNCTIONS_HPP_

#include <cstdio>

#include "hal.hpp"

namespace console_task {
	typedef std::unique_ptr<char*[]> CommandArguments;

	void sample_function(char* output_string, uint8_t argc, CommandArguments argv)
	{
		strcpy(output_string,"Sample function successfully called.\r\n\r\n");
	}

	void set_led(char* output_string, uint8_t argc, CommandArguments argv)
	{
		if(!strcmp(argv[0],"on"))
		{
			strcpy(output_string,"Turning LED on...\r\n");
			Chip_GPIO_WritePortBit(LPC_GPIO, 2, 10, true);
		}
		else if(!strcmp(argv[0],"off"))
		{
			strcpy(output_string,"Turning LED off...\r\n");
			Chip_GPIO_WritePortBit(LPC_GPIO, 2, 10, false);
		}
		else
		{
			strcpy(output_string, "Invalid Parameter -- options: \'on\' or \'off\'...\r\n");
		}
	}

	void activate_led(char* output_string, uint8_t argc, CommandArguments argv)
	{
		strcpy(output_string,"Turning LED on...\r\n");
		Chip_GPIO_WritePortBit(LPC_GPIO, 2, 10, true);
	}

	void deactivate_led(char* output_string, uint8_t argc, CommandArguments argv)
	{
		strcpy(output_string,"Turning LED off...\r\n");
		Chip_GPIO_WritePortBit(LPC_GPIO, 2, 10, false);
	}

	void get_mem(char* output_string, uint8_t argc, CommandArguments argv)
	{
		sprintf(output_string, "Free Memory: %d\r\n"
				"Memory Watermark: %d\r\n",
				xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
	}

	typedef void (*CommandFunction)(char*,uint8_t,CommandArguments);

	struct CommandDescriptor {
		const char *call_string;
		const CommandFunction function;
	};

	const CommandDescriptor command_list[] = {
			{"sample_function", &sample_function},
			{"activate_led", &activate_led},
			{"deactivate_led", &deactivate_led},
			{"set_led", &set_led},
			{"get_mem", &get_mem},
	};

#define NUMBER_COMMANDS (sizeof(command_list) / sizeof(CommandDescriptor))
}


#endif /* UTIL_INC_CONSOLE_FUNCTIONS_HPP_ */
