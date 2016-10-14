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
	static void read_handler(std::shared_ptr<UartReadData> data);

	TaskHandle_t task_handle;
	SemaphoreHandle_t semphr;
	UartIo *uart;
	GpdmaManager *dma_man;
	GpdmaChannel *test_channel_tx;
	GpdmaChannel *test_channel_rx;

	const char *str = "test\r\n";
	uint8_t str_len = 6;

	std::unique_ptr<uint8_t[]> last_read;

	void start(void) {
		uart = hal::get_driver<UartIo>(hal::CONSOLE_UART);
		uart->allocate_buffers(32, 32);

		dma_man = hal::get_driver<GpdmaManager>(hal::GPDMA_MAN);
		test_channel_tx = dma_man->allocate_channel(0);
		test_channel_rx = dma_man->allocate_channel(1);

		//uart->bind_dma_channels(test_channel_tx, test_channel_rx);
		//uart->set_transfer_mode(UART_XFER_MODE_DMA);

		semphr = xSemaphoreCreateBinary();

		xTaskCreate(task_loop, "dma task", 1536, NULL, 2, &task_handle);
	}

	static void task_loop(void *p) {
		uint8_t array[6] = {'t', 'e', 's', 't', '\r', '\n'};
		while(true) {
			//auto del = dlgt::make_delegate(&read_handler);
			//uart->read_async(5, del);
			//xSemaphoreTake(semphr, portMAX_DELAY);
			//volatile uint8_t* d = last_read.get();
			//uart->write((uint8_t*)d, 5);

			uart->write(array, 6);
			uart->read(array, 6);
		}
	}

	static void read_handler(std::shared_ptr<UartReadData> read_status) {
		last_read = std::move(read_status->data);
		xSemaphoreGive(semphr);
	}
}


