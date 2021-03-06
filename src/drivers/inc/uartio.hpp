/*
 * uart.hpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nate
 */

#ifndef DRIVERS_INC_UARTIO_HPP_
#define DRIVERS_INC_UARTIO_HPP_


#include <cstdint>
#include <memory>

#include "delegate.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

#include "gpdma.hpp"
#include "driver.hpp"
#include "chip.h"

enum UartTransferMode {
	UART_XFER_MODE_POLLING,
	UART_XFER_MODE_INTERRUPT,
	UART_XFER_MODE_DMA
};

enum UartError {
	UART_ERROR_NONE,
	UART_ERROR_GENERAL,
	UART_ERROR_NO_DMA_CHANNEL,
	UART_ERROR_MEMORY_ALLOCATION,
	UART_ERROR_DMA_IN_USE,
	UART_ERROR_BUFFER_OVERFLOW,
	UART_ERROR_FRAMING_ERROR,
};

typedef dlgt::delegate<void(*)(UartError, uint8_t*, uint16_t)> UartReadDelegate;
typedef dlgt::delegate<void(*)(UartError)> UartWriteDelegate;

class UartIo : public Driver {
public:
	UartIo(LPC_USART_T *uart);
	void init_driver(void);


	void enable_interrupts();

	/*
	 * @brief Allocate the buffers needed for operations
	 * @param tx_buffer_size : Size of the tx buffer in bytes
	 * @param rx_buffer_size : Size of the rx buffer in bytes
	 * @return NONE on SUCCESS, MEMORY_ALLOCATION_ERROR if there is a problem allocating memory
	 * @notes
	 * This function must be called prior to using any of the driver functionality
	 * In the implemation of this driver using the bullshit ring buffer the size must be a power of 2...
	 */
	UartError allocate_buffers(uint16_t tx_buffer_size, uint16_t rx_buffer_size);

	/*
	 * @brief Set the UART buad rate
	 * @param baud : The baud rate to set
	 * @return none
	 */
	void set_baud(uint32_t baud);

	void set_baud_fractional(uint16_t fdr, uint16_t dll, uint16_t dlm, CHIP_SYSCTL_CLKDIV_T pclk_div);

	/*
	 * @brief Configure the mode in switch the uart sends data
	 * @param word_length : The number of bits per word
	 * @param parity : The number of parity bits to send
	 * @param stop_bits : The number of stop bits to send
	 * @return none
	 */
	void config_data_mode(uint32_t word_length, uint32_t parity, uint32_t stop_bits);

	/*
	 * @brief Set up the internal transfer mode of the driver
	 * @param mode : This is the mode of the transfer
	 * @return NONE on success
	 */
	UartError set_transfer_mode(UartTransferMode mode);

	/*
	 * @brief Add references to DMA channels to UART instances
	 * @param tx_channel: Pointer to tx dma channel
	 * @param rx_channel: Pointer to rx dma channel
	 * @return NONE on success
	 */
	UartError bind_dma_channels(GpdmaChannel *tx_channel, GpdmaChannel *rx_channel);

	/*
	 * @brief Unbind refrences to stored DMA channels to free them up for other uses
	 * @return NONE on success
	 */
	UartError unbind_dma_channels(void);

	/*
	 * @brief Write bytes via the UART
	 * @param data : Pointer to the data to be written
	 * @param length : Length of the data this is being written
	 * @return NONE on success
	 * @note
	 * The data will be send via whatever tx transfer mode you setup
	 */
	UartError write(const uint8_t* data, uint16_t length);

	UartError write_async(const uint8_t* data, uint16_t length, UartWriteDelegate& delegate);

	/*
	 * @brief Read bytes from UART
	 * @param data : Pointer to where the data is to be saved
	 * @param length : Length of the data to be read
	 * @return NONE on success
	 * @note
	 * The data will be read via whatever rx transfer mode you setup
	 */
	UartError read(uint8_t* data, uint16_t length);

	UartError read_async(uint16_t length, UartReadDelegate& delegate);

	void uartInterruptHandler(void);


private:
	void fill_tx_fifo(void);
	IRQn_Type get_nvic_irq(void);
	uint32_t get_tx_dmareq(void);
	uint32_t get_rx_dmareq(void);
	CHIP_SYSCTL_PCLK_T get_pclk(void);

	LPC_USART_T *uart;

	UartTransferMode transfer_mode;

	SemaphoreHandle_t tx_transfer_semaphore;
	SemaphoreHandle_t rx_transfer_semaphore;

	GpdmaChannel *tx_dma_channel = NULL;
	GpdmaChannel *rx_dma_channel = NULL;

	bool is_writing = false;
	bool is_reading = false;
	bool is_write_async = false;
	bool is_read_async = false;
	bool is_allocated = false;

	uint8_t volatile *tx_buffer;
	uint8_t volatile *rx_buffer;
	uint16_t tx_buffer_len;
	uint16_t rx_buffer_len;
	uint16_t tx_op_len;
	uint16_t rx_op_len;
	volatile uint16_t tx_buffer_head_pos;
	volatile uint16_t rx_buffer_head_pos;

	UartReadDelegate *rx_delegate = NULL;
	UartWriteDelegate *tx_delegate = NULL;

	UartError tx_status;
	UartError rx_status;
};



#endif /* DRIVERS_INC_EXAMPLELED_HPP_ */
