/*
 * dma_test_task.cpp
 *
 *  Created on: Oct 12, 2016
 *      Author: nigil
 */

#include <cstdint>
#include <memory>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "hal.hpp"
#include "uartio.hpp"
#include "gpdma.hpp"

#include "dma_test_task.hpp"

namespace dma_test_task {
	static void task_loop(void *p);
	static void read_handler(UartError status, uint8_t *data, uint16_t len);
	static void write_handler(UartError status);

	TaskHandle_t task_handle;
	UartIo *uart;
	GpdmaManager *dma_man;
	GpdmaChannel *test_channel_tx;
	GpdmaChannel *test_channel_rx;

	auto read_del = dlgt::make_delegate(&read_handler);
	auto write_del = dlgt::make_delegate(&write_handler);

	void start(void) {
		uart = hal::get_driver<UartIo>(hal::CONSOLE_UART);
		uart->allocate_buffers(32, 32);

		dma_man = hal::get_driver<GpdmaManager>(hal::GPDMA_MAN);
		test_channel_tx = dma_man->allocate_channel(0);
		test_channel_rx = dma_man->allocate_channel(1);

		// Enabled DMA (Optional)
		uart->bind_dma_channels(test_channel_tx, test_channel_rx);
		uart->set_transfer_mode(UART_XFER_MODE_DMA);

		xTaskCreate(task_loop, "dma task", 1536, NULL, 2, &task_handle);
	}

	static void task_loop(void *p) {
		/*
		// Sync example
		uint8_t array[6] = {'t', 'e', 's', 't', '\r', '\n'};
		while(true) {


			uart->write(array, 6);
			uart->read(array, 6);
		}
		*/

		// Async example
		uart->read_async(5, read_del);
		while(true) {
			vTaskDelay(800); // Sleep forever!
			asm("nop;");
		}
	}

	static void read_handler(UartError status, uint8_t *data, uint16_t len) {
		uart->write_async(data, len, write_del);
	}

	static void write_handler(UartError status) {
		uart->read_async(5, read_del);
	}
}


