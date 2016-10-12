/*
 * uart.hpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nate
 */

#ifndef DRIVERS_INC_UARTIO_HPP_
#define DRIVERS_INC_UARTIO_HPP_

#include <cstdint>
#include "driver.hpp"
#include "chip.h"
#include "FreeRTOS.h"
#include "semphr.h"

#include "gpdma.hpp"

#define RX_BUFFER_SIZE 32
#define TX_BUFFER_SIZE 128

typedef void (*uart_char_read_callback)(uint8_t);

class UartIo : public Driver {
public:
	UartIo(LPC_USART_T *uart);
	void init_driver(void);

	inline void setBaud(uint32_t baud) {this->baud_rate = baud;
	Chip_UART_SetBaud(this->uart, baud);}
	void configData(uint32_t word_length, uint32_t parity, uint32_t stopbits);
	void uartInterruptHandler(void);

	//Read and Write
	void write(uint8_t* data, uint8_t length);
	void read(uint8_t* data, uint8_t length);
	void write_dma(uint8_t* data, uint8_t length, GpdmaChannel *dma_channel);
	inline void readChar(uint8_t* data) {read(data, 1);}
	inline void writeChar(uint8_t data) {write(&data,1);}
	void readCharAsync(uart_char_read_callback callback);

private:
	IRQn_Type getNVICIRQ(void);
	uint32_t get_tx_dmareq(void);

	SemaphoreHandle_t tx_transfer_semaphore;
	SemaphoreHandle_t rx_transfer_semaphore;

	bool write_in_progress = false;
	bool read_in_progress = false;
	bool async_read_in_progress = false;

	LPC_USART_T *uart;
	uint32_t baud_rate;
	RINGBUFF_T txring;
	RINGBUFF_T rxring;
	uint8_t rxbuff[RX_BUFFER_SIZE];
	uint8_t txbuff[TX_BUFFER_SIZE];

	uart_char_read_callback callback;
};



#endif /* DRIVERS_INC_EXAMPLELED_HPP_ */
