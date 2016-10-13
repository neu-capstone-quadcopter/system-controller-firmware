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
	GpdmaManager *dma_man;
	GpdmaChannel *test_channel_tx;
	GpdmaChannel *test_channel_rx;

	const char *str = "test\r\n";
	uint8_t str_len = 6;

	void start(void) {
		uart = hal::get_driver<UartIo>(hal::CONSOLE_UART);
		uart->allocate_buffers(100, 100);

		dma_man = hal::get_driver<GpdmaManager>(hal::GPDMA_MAN);
		test_channel_tx = dma_man->allocate_channel(0);
		test_channel_rx = dma_man->allocate_channel(1);

		uart->bind_dma_channels(test_channel_tx, test_channel_rx);
		uart->set_transfer_mode(UART_XFER_MODE_DMA);

		xTaskCreate(task_loop, "dma task", 1536, NULL, 2, &task_handle);
	}

	static void task_loop(void *p) {
		uint8_t array[4];
		while(true) {
			vTaskDelay(200);
			uart->write((uint8_t *)str, 6);
			uart->read(array, 4);
		}
	}
}


