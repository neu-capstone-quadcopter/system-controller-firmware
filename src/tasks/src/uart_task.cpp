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

			uint8_t key;
			do
			{
				uart->read_char(&key);
				if(key != 0){
					uart->write_char(&key);
					key = 0;
				}
			}
			while(true);

			vTaskDelay(100);
			uint8_t send_char = (uint8_t)'N';
			uart->write_char(&send_char);

		}
	}
}

