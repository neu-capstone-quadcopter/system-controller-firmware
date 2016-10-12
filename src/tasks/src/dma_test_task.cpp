/*
 * dma_test_task.cpp
 *
 *  Created on: Oct 12, 2016
 *      Author: nigil
 */

#include <cstdint>

#include "FreeRTOS.h"
#include "task.h"

#include "hal.hpp"
#include "uartio.hpp"
#include "gpdma.hpp"

#include "dma_test_task.hpp"

namespace dma_test_task {
	static void task_loop(void *p);

	TaskHandle_t task_handle;
	UartIo *uart;

	void start(void) {
		uart = hal::get_driver<UartIo>(hal::CONSOLE_UART);

		xTaskCreate(task_loop, "dma task", 1536, NULL, 2, &task_handle);
	}

	static void task_loop(void *p) {

	}
}


