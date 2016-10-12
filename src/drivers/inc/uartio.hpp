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

enum UartTransferMode {POLLING, INTERRUPT, DMA};

enum UartError {
	NONE,
	NO_DMA_CHANNEL
};

class UartIo : public Driver {
public:
	UartIo(LPC_USART_T *uart);
	void init_driver(void);

	/*
	 * @brief Set the UART buad rate
	 * @param baud : The baud rate to set
	 * @return none
	 */
	void set_baud(uint32_t baud);

	/*
	 * @brief Configure the mode in switch the uart sends data
	 * @param word_length : The number of bits per word
	 * @param parity : The number of parity bits to send
	 * @param stop_bits : The number of stop bits to send
	 * @return none
	 */
	void config_data_mode(uint32_t word_length, uint32_t parity, uint32_t stop_bits);

	/*
	 * @brief Set up the internal transmit transfer mode of the driver
	 * @param mode : This is the mode of the transfer
	 * @return NONE on success
	 */
	UartError set_tx_transfer_mode(UartTransferMode mode);

	/*
	 * @brief Set up the internal receive transfer mode of the driver
	 * @param mode : This is the mode of the transfer
	 * @return NONE on success
	 */
	UartError set_rx_transfer_mode(UartTransferMode mode);

	UartError bind_tx_dma_channel(GpdmaChannel *dma_channel);
	UartError unbind_tx_dma_channel(void);
	UartError bind_rx_dma_channel(GpdmaChannel *dma_channel);
	UartError unbind_rx_dma_channel(void);

	/*
	 * @brief Write bytes via the UART
	 * @param data : Pointer to the data to be written
	 * @param length : Length of the data this is being written
	 * @return NONE on success
	 * @note
	 * The data will be send via whatever tx transfer mode you setup
	 */
	UartError write(uint8_t* data, uint8_t length);

	/*
	 * @brief Read bytes from UART
	 * @param data : Pointer to where the data is to be saved
	 * @param length : Length of the data to be read
	 * @return NONE on success
	 * @note
	 * The data will be read via whatever rx transfer mode you setup
	 */
	UartError read(uint8_t* data, uint8_t length);

	void write_dma(uint8_t* data, uint8_t length, GpdmaChannel *dma_channel);
	inline void readChar(uint8_t* data) {read(data, 1);}
	inline void writeChar(uint8_t data) {write(&data,1);}
	void readCharAsync(uart_char_read_callback callback);

	void uartInterruptHandler(void);
private:
	IRQn_Type get_nvic_irq(void);
	uint32_t get_tx_dmareq(void);

	UartTransferMode tx_transfer_mode;
	UartTransferMode rx_transfer_mode;

	SemaphoreHandle_t tx_transfer_semaphore;
	SemaphoreHandle_t rx_transfer_semaphore;

	GpdmaChannel *tx_dma_channel = NULL;
	GpdmaChannel *rx_dma_channel = NULL;

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
