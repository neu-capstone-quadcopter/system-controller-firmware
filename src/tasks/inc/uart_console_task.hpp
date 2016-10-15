/*
 * uart_consoletask.hpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nate
 */

#ifndef TASKS_INC_UARTCONSOLETASK_HPP_
#define TASKS_INC_UARTCONSOLETASK_HPP_

#include "FreeRTOS.h"
#include "task.h"

#define UART_EVENT_DATA_MAX_LEN 255

namespace console_task {
	void start(void);

	void send_debug_message(uint8_t *data, size_t length);
	void send_debug_message(uint8_t *data, uint32_t length, int32_t *bytes_written);
}


#endif /* TASKS_INC_UARTTASK_HPP_ */
