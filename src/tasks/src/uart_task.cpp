/*
 * uart_task.cpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nate
 */

#include "uart_task.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include "hal.hpp"
#include "uart.hpp"

namespace uart_task {
	static void task_loop(void *p);

	TaskHandle_t task_handle;

	Uart *uart;

	void start(void) {
		// Retrieve driver instances from HAL
		uart = static_cast<Uart*>(hal::get_driver(hal::CONSOLE_UART));

		xTaskCreate(task_loop, "uart task", 1536, NULL, 2, &task_handle);
	}

	static void task_loop(void *p) {
		for(;;) {
			asm("NOP;");
		}
	}
}
