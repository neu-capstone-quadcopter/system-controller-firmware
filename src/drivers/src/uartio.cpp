/*
 * uart.cpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nate
 */

#include <cstring>

#include "FreeRTOS.h"
#include "chip.h"
#include "board.hpp"

#include "uartio.hpp"

/* Default Values */
#define DEFAULT_BAUD 115200
#define DEFAULT_WORD_LENGTH UART_LCR_WLEN8
#define DEFAULT_PARITY UART_LCR_PARITY_DIS
#define DEFAULT_STOP_BIT UART_LCR_SBS_1BIT
#define DEFAULT_TRANSFER_MODE INTERRUPT

class UartTxDmaHandlerFunctor : public DmaHandlerFunctor {
public:
	UartTxDmaHandlerFunctor(UartIo *uart) {
		this->uart = uart;
	}
	void dma_handler(DmaError error) {
		this->uart->tx_dma_handler(error);
	}
private:
	UartIo *uart;
};

UartIo::UartIo(LPC_USART_T *uart) {
	this->uart = uart;
	this->tx_transfer_mode = DEFAULT_TRANSFER_MODE;
	this->rx_transfer_mode = DEFAULT_TRANSFER_MODE;
}

void UartIo::init_driver(void)
{
	board::uart_init(this->uart);
	Chip_UART_Init(this->uart);

	this->tx_transfer_semaphore = xSemaphoreCreateBinary();
	this->rx_transfer_semaphore = xSemaphoreCreateBinary();

	set_baud(DEFAULT_BAUD);
	config_data_mode(DEFAULT_WORD_LENGTH, DEFAULT_PARITY, DEFAULT_STOP_BIT);
	Chip_UART_TXEnable(this->uart);
	Chip_UART_SetupFIFOS(this->uart, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2 |
	                       UART_FCR_RX_RS | UART_FCR_TX_RS));

	Chip_UART_IntEnable(this->uart, (UART_IER_RBRINT | UART_IER_RLSINT));
	NVIC_EnableIRQ(get_nvic_irq());
}

UartError UartIo::allocate_buffers(uint16_t tx_buffer_size, uint16_t rx_buffer_size) {
	this->tx_buffer = new uint8_t[tx_buffer_size];
	this->rx_buffer = new uint8_t[rx_buffer_size];

	if(!this->tx_buffer || !this->rx_buffer) {
		return UART_ERROR_MEMORY_ALLOCATION;
	}

	RingBuffer_Init(&this->tx_ring, this->tx_buffer, 1, tx_buffer_size);
	RingBuffer_Init(&this->rx_ring, this->rx_buffer, 1, rx_buffer_size);

	this->tx_buffer_len = tx_buffer_size;
	this->rx_buffer_len = rx_buffer_size;

	return UART_ERROR_NONE;
}

void UartIo::set_baud(uint32_t baud) {
	this->baud_rate = baud;
	Chip_UART_SetBaud(this->uart, baud);
}

void UartIo::config_data_mode(uint32_t word_length, uint32_t parity, uint32_t stop_bits){
	Chip_UART_ConfigData(this->uart, (word_length | stop_bits));
}

UartError UartIo::set_tx_transfer_mode(UartTransferMode mode) {
	if(mode == DMA && !this->tx_dma_channel) {
		return UART_ERROR_NO_DMA_CHANNEL;
	}
	this->tx_transfer_mode = mode;
	return UART_ERROR_NONE;
}

UartError UartIo::set_rx_transfer_mode(UartTransferMode mode) {
	if(mode == DMA && !this->rx_dma_channel) {
		return UART_ERROR_NO_DMA_CHANNEL;
	}
	this->rx_transfer_mode = mode;
	return UART_ERROR_NONE;
}

UartError UartIo::bind_tx_dma_channel(GpdmaChannel *dma_channel) {
	this->tx_dma_channel = dma_channel;
	return UART_ERROR_NONE;
}

UartError UartIo::unbind_tx_dma_channel(void) {
	this->tx_dma_channel = NULL;
	if(this->tx_transfer_mode == DMA) {
		this->tx_transfer_mode = INTERRUPT;
	}
	return UART_ERROR_NONE;
}

UartError UartIo::bind_rx_dma_channel(GpdmaChannel *dma_channel) {
	this->rx_dma_channel = dma_channel;
	return UART_ERROR_NONE;
}

UartError UartIo::unbind_rx_dma_channel(void) {
	this->rx_dma_channel = NULL;
	if(this->rx_transfer_mode == DMA) {
		this->rx_transfer_mode = INTERRUPT;
	}
	return UART_ERROR_NONE;
}

UartError UartIo::write(uint8_t* data, uint16_t length)
{
	switch(this->tx_transfer_mode) {
	case POLLING:
		// TODO: Maybe implement this lol
		return UART_ERROR_GENERAL;
	case INTERRUPT:
		write_in_progress = true;
		Chip_UART_SendRB(this->uart, &this->tx_ring, data, length);
		xSemaphoreTake(this->tx_transfer_semaphore, portMAX_DELAY);
		return UART_ERROR_NONE;
	case DMA:
	{
		if(this->tx_dma_channel->is_active()) {
			return UART_ERROR_DMA_IN_USE;
		}

		if(length > this->tx_buffer_len) {
			return UART_ERROR_BUFFER_OVERFLOW;
		}

		memcpy(this->tx_buffer, data, length);

		this->uart->FCR |= UART_FCR_DMAMODE_SEL;
		UartTxDmaHandlerFunctor handler_func(this);
		this->tx_dma_channel->register_callback(&handler_func);
		this->tx_dma_channel->start_transfer(
				(uint32_t)data,
				get_tx_dmareq(),
				GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA,
				length);
		xSemaphoreTake(this->tx_transfer_semaphore, portMAX_DELAY);
		return UART_ERROR_NONE;
	}
	default:
		configASSERT(0);
	}
	return UART_ERROR_GENERAL;
}

UartError UartIo::read(uint8_t* data, uint16_t length)
{
	switch(this->rx_transfer_mode) {
	case POLLING:
		// TODO: Maybe implement this lol
		return UART_ERROR_GENERAL;
	case INTERRUPT:
		this->read_in_progress = true;
		xSemaphoreTake(this->rx_transfer_semaphore, portMAX_DELAY);
		Chip_UART_ReadRB(this->uart, &this->rx_ring, data, length);
		return UART_ERROR_NONE;
	case DMA:
		return UART_ERROR_NONE;
	default:
		configASSERT(0);
	}
	return UART_ERROR_GENERAL;
}

void UartIo::readCharAsync(uart_char_read_callback callback)
{
	this->callback = callback;
	this->async_read_in_progress = true;

}

IRQn_Type UartIo::get_nvic_irq(void){

	switch((uint32_t)this->uart)
	{

	case LPC_UART0_BASE:
		return UART0_IRQn;
	case LPC_UART1_BASE:
		return UART1_IRQn;
	case LPC_UART2_BASE:
		return UART2_IRQn;
	case LPC_UART3_BASE:
		return UART3_IRQn;
	default:
		configASSERT(0);
	}
	return UART0_IRQn;
}

uint32_t UartIo::get_tx_dmareq(void) {
	switch((uint32_t)this->uart)
	{
	case LPC_UART0_BASE:
		return GPDMA_CONN_UART0_Tx;
	case LPC_UART1_BASE:
		return GPDMA_CONN_UART1_Tx;
	case LPC_UART2_BASE:
		return GPDMA_CONN_UART2_Tx;
	case LPC_UART3_BASE:
		return GPDMA_CONN_UART3_Tx;
	default:
		configASSERT(0);
	}
	return GPDMA_CONN_UART0_Tx;
}

void UartIo::tx_dma_handler(DmaError status) {
	xSemaphoreGiveFromISR(this->tx_transfer_semaphore, NULL);
}

void UartIo::rx_dma_handler(DmaError status) {
	xSemaphoreGiveFromISR(this->rx_transfer_semaphore, NULL);
}

void UartIo::uartInterruptHandler(void){
	Chip_UART_IRQRBHandler(this->uart, &this->rx_ring, &this->tx_ring);

	if(write_in_progress && RingBuffer_IsEmpty(&this->tx_ring));
	{
		xSemaphoreGiveFromISR(this->tx_transfer_semaphore, NULL);
		write_in_progress = false;
	}

	if(this->async_read_in_progress && !RingBuffer_IsEmpty(&this->rx_ring))
	{
		uint8_t data;
		Chip_UART_ReadRB(this->uart, &this->rx_ring, &data, 1);
		this->callback(data);
		this->async_read_in_progress = false;
	}
	else if(this->read_in_progress)
	{
		if(!RingBuffer_IsEmpty(&this->rx_ring))
		{
			this->read_in_progress = false;
			xSemaphoreGiveFromISR(this->rx_transfer_semaphore, NULL);
		}
	}
}
