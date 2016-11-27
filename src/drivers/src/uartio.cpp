/*
 * uart.cpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nate
 */

#include <cstring>
#include <functional>

#include "FreeRTOS.h"
#include "chip.h"
#include "board.hpp"

#include "uartio.hpp"

/* Default Values */
#define DEFAULT_BAUD 115200
#define DEFAULT_WORD_LENGTH UART_LCR_WLEN8
#define DEFAULT_PARITY UART_LCR_PARITY_DIS
#define DEFAULT_STOP_BIT UART_LCR_SBS_1BIT
#define DEFAULT_TRANSFER_MODE UART_XFER_MODE_INTERRUPT

const uint8_t UART_FIFO_DEPTH = 16;

UartIo::UartIo(LPC_USART_T *uart) {
	this->uart = uart;
	this->transfer_mode = DEFAULT_TRANSFER_MODE;
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

	//Chip_UART_IntEnable(this->uart, (UART_IER_RBRINT | UART_IER_RLSINT));
	NVIC_EnableIRQ(get_nvic_irq());
	NVIC_SetPriority(get_nvic_irq(), 7);
}

void UartIo::enable_interrupts() {
	Chip_UART_IntEnable(this->uart, (UART_IER_RBRINT | UART_IER_RLSINT));
}

UartError UartIo::allocate_buffers(uint16_t tx_buffer_size, uint16_t rx_buffer_size) {
	this->tx_buffer = new uint8_t[tx_buffer_size];
	this->rx_buffer = new uint8_t[rx_buffer_size];

	if(!this->tx_buffer || !this->rx_buffer) {
		return UART_ERROR_MEMORY_ALLOCATION;
	}

	this->tx_buffer_len = tx_buffer_size;
	this->rx_buffer_len = rx_buffer_size;

	this->is_allocated = true;

	return UART_ERROR_NONE;
}

void UartIo::set_baud(uint32_t baud) {
	Chip_UART_SetBaud(this->uart, baud);
}

void UartIo::set_baud_fractional(uint16_t fdr, uint16_t dll, uint16_t dlm, CHIP_SYSCTL_CLKDIV_T pclk_div)
{
	Chip_UART_EnableDivisorAccess(uart);
	uart->FDR = fdr;
	uart->DLL = dll;
	uart->DLM = dlm;
	Chip_UART_DisableDivisorAccess(uart);

	Chip_Clock_SetPCLKDiv(this->get_pclk(), pclk_div);
}

void UartIo::config_data_mode(uint32_t word_length, uint32_t parity, uint32_t stop_bits){

	if(parity != UART_LCR_PARITY_DIS) {
		Chip_UART_ConfigData(this->uart, (word_length | stop_bits | parity | UART_LCR_PARITY_EN));
	}
	else {
		Chip_UART_ConfigData(this->uart, (word_length | stop_bits));
	}

}

UartError UartIo::set_transfer_mode(UartTransferMode mode) {
	if(mode == UART_XFER_MODE_DMA && (!this->tx_dma_channel || !this->rx_dma_channel)) {
		return UART_ERROR_NO_DMA_CHANNEL;
	}

	if(mode == UART_XFER_MODE_DMA) {
		Chip_UART_SetupFIFOS(this->uart, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2 |
			                       UART_FCR_RX_RS | UART_FCR_TX_RS | UART_FCR_DMAMODE_SEL));
	}
	else {
		Chip_UART_SetupFIFOS(this->uart, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2 |
			                       UART_FCR_RX_RS | UART_FCR_TX_RS));
	}

	this->transfer_mode = mode;
	return UART_ERROR_NONE;
}

UartError UartIo::bind_dma_channels(GpdmaChannel *tx_channel, GpdmaChannel *rx_channel) {
	this->tx_dma_channel = tx_channel;
	this->rx_dma_channel = rx_channel;
	return UART_ERROR_NONE;
}

UartError UartIo::unbind_dma_channels(void) {
	this->tx_dma_channel = NULL;
	this->rx_dma_channel = NULL;
	if(this->transfer_mode == UART_XFER_MODE_DMA) {
		this->transfer_mode = UART_XFER_MODE_INTERRUPT;
	}
	return UART_ERROR_NONE;
}

UartError UartIo::write(const uint8_t* data, uint16_t length)
{
	this->tx_delegate = NULL;
	this->tx_op_len = length;
	this->tx_buffer_head_pos = 0;
	memcpy(const_cast<uint8_t*>(this->tx_buffer), data, length);

	switch(this->transfer_mode) {
	case UART_XFER_MODE_POLLING:
		// TODO: Maybe implement this lol
		return UART_ERROR_GENERAL;
	case UART_XFER_MODE_INTERRUPT:
		this->uart->IER |= UART_IER_THREINT;
		this->fill_tx_fifo();
		xSemaphoreTake(this->tx_transfer_semaphore, portMAX_DELAY);
		return UART_ERROR_NONE;
	case UART_XFER_MODE_DMA:
	{
		if(this->tx_dma_channel->is_active()) {
			return UART_ERROR_DMA_IN_USE;
		}

		if(length > this->tx_buffer_len) {
			return UART_ERROR_BUFFER_OVERFLOW;
		}

		this->tx_dma_channel->register_callback([this](DmaError status){
			BaseType_t task_woken = pdFALSE;
			xSemaphoreGiveFromISR(this->tx_transfer_semaphore, &task_woken);
			if(task_woken) {
				vPortYield();
			}
		});
		this->tx_dma_channel->start_transfer(
				reinterpret_cast<uint32_t>(this->tx_buffer),
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

UartError UartIo::write_async(const uint8_t* data, uint16_t length, UartWriteDelegate& delegate) {
	this->tx_delegate = &delegate;
	this->tx_op_len = length;
	this->tx_buffer_head_pos = 0;
	memcpy(const_cast<uint8_t*>(this->tx_buffer), data, length);

	switch(this->transfer_mode) {
	case UART_XFER_MODE_INTERRUPT:
		this->uart->IER |= UART_IER_THREINT;
		this->fill_tx_fifo();

		return UART_ERROR_NONE;
	case UART_XFER_MODE_DMA:
	{
		if(this->tx_dma_channel->is_active()) {
			return UART_ERROR_DMA_IN_USE;
		}

		if(length > this->tx_buffer_len) {
			return UART_ERROR_BUFFER_OVERFLOW;
		}

		this->tx_dma_channel->register_callback([this](DmaError status){
			(*this->tx_delegate)(UART_ERROR_NONE);
		});
		this->tx_dma_channel->start_transfer(
				reinterpret_cast<uint32_t>(this->tx_buffer),
				get_tx_dmareq(),
				GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA,
				length);
		return UART_ERROR_NONE;
	}
	default:
		configASSERT(0);
	}
	return UART_ERROR_GENERAL;
}

UartError UartIo::read(uint8_t* data, uint16_t length)
{
	this->rx_op_len = length;
	this->rx_buffer_head_pos = 0;
	this->rx_delegate = NULL;
	this->rx_status = UART_ERROR_NONE;

	switch(this->transfer_mode) {
	case UART_XFER_MODE_POLLING:
		// TODO: Maybe implement this lol
		return UART_ERROR_GENERAL;
	case UART_XFER_MODE_INTERRUPT:
	{
		this->uart->IER |= UART_IER_RLSINT | UART_IER_RBRINT;
		xSemaphoreTake(this->rx_transfer_semaphore, portMAX_DELAY);
		memcpy(data, const_cast<uint8_t*>(this->rx_buffer), this->rx_buffer_head_pos);
		return UART_ERROR_NONE;
	}
	case UART_XFER_MODE_DMA:
	{
		if(this->rx_dma_channel->is_active()) {
			return UART_ERROR_DMA_IN_USE;
		}

		if(length > this->rx_buffer_len) {
			return UART_ERROR_BUFFER_OVERFLOW;
		}

		this->rx_dma_channel->register_callback([this](DmaError status){
			BaseType_t task_woken = pdFALSE;
			xSemaphoreGiveFromISR(this->rx_transfer_semaphore, &task_woken);
			if(task_woken) {
				vPortYield();
			}
		});
		this->rx_dma_channel->start_transfer(
				get_rx_dmareq(),
				reinterpret_cast<uint32_t>(data),
				GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA,
				length);
		xSemaphoreTake(this->rx_transfer_semaphore, portMAX_DELAY);
		return UART_ERROR_NONE;
	}
	default:
		configASSERT(0);
	}
	return UART_ERROR_GENERAL;
}

UartError UartIo::read_async(uint16_t length, UartReadDelegate& delegate) {
	this->rx_delegate = &delegate;
	this->rx_op_len = length;
	this->rx_buffer_head_pos = 0;
	this->rx_status = UART_ERROR_NONE;

	switch(this->transfer_mode) {
	case UART_XFER_MODE_INTERRUPT:
	{
		this->uart->IER |= UART_IER_RLSINT | UART_IER_RBRINT;
		return UART_ERROR_NONE;
	}
	case UART_XFER_MODE_DMA:
	{
		if(this->rx_dma_channel->is_active()) {
			return UART_ERROR_DMA_IN_USE;
		}

		if(length > this->rx_buffer_len) {
			return UART_ERROR_BUFFER_OVERFLOW;
		}

		this->rx_dma_channel->register_callback([this](DmaError status) {
			uint8_t *data_cpy = new uint8_t[this->rx_op_len];
			memcpy(data_cpy, const_cast<uint8_t*>(this->rx_buffer), this->rx_op_len);
			(*this->rx_delegate)(UART_ERROR_NONE, const_cast<uint8_t*>(this->rx_buffer), this->rx_op_len);
		});

		//this->uart->FCR |= UART_FCR_RX_RS;

		this->rx_dma_channel->start_transfer(
				get_rx_dmareq(),
				reinterpret_cast<uint32_t>(this->rx_buffer),
				GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA,
				length);
		return UART_ERROR_NONE;
	}
	default:
		configASSERT(0);
	}
	return UART_ERROR_GENERAL;
}

void UartIo::fill_tx_fifo(void) {
	if(this->uart->LSR & UART_LSR_THRE) { // If TX Fifo is empty
		for(uint8_t i; i < UART_FIFO_DEPTH; i++) {
			if(this->tx_buffer_head_pos < this->tx_op_len) {
				this->uart->THR = this->tx_buffer[this->tx_buffer_head_pos++];
			}
			else {
				break;
			}
		}
	}
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

uint32_t UartIo::get_rx_dmareq(void) {
	switch((uint32_t)this->uart)
	{
	case LPC_UART0_BASE:
		return GPDMA_CONN_UART0_Rx;
	case LPC_UART1_BASE:
		return GPDMA_CONN_UART1_Rx;
	case LPC_UART2_BASE:
		return GPDMA_CONN_UART2_Rx;
	case LPC_UART3_BASE:
		return GPDMA_CONN_UART3_Rx;
	default:
		configASSERT(0);
	}
	return GPDMA_CONN_UART0_Rx;
}

CHIP_SYSCTL_PCLK_T UartIo::get_pclk(void) {
	switch((uint32_t)this->uart)
	{
	case LPC_UART0_BASE:
		return SYSCTL_PCLK_UART0;
	case LPC_UART1_BASE:
		return SYSCTL_PCLK_UART1;
	case LPC_UART2_BASE:
		return SYSCTL_PCLK_UART2;
	case LPC_UART3_BASE:
		return SYSCTL_PCLK_UART3;
	default:
		configASSERT(0);
	}
	return SYSCTL_PCLK_UART0;
}

void UartIo::uartInterruptHandler(void){
	BaseType_t task_woken = pdFALSE;

	switch(this->uart->IIR & UART_IIR_INTID_MASK) {
	case UART_IIR_INTID_THRE:
	{
		// Interrupt type was THRE
		this->fill_tx_fifo();

		if(this->tx_buffer_head_pos >= this->tx_op_len) {
			// Write is over
			this->uart->IER &= ~UART_IER_THREINT;

			if(this->tx_delegate) {
				// Async Write
				(*this->tx_delegate)(UART_ERROR_NONE);
			}
			else {
				// Sync write
				xSemaphoreGiveFromISR(this->tx_transfer_semaphore, &task_woken);
			}
		}
		break;
	}
	case UART_IIR_INTID_RLS:
	{
		// RLS interrupt occured, some sort of error occured
		// TODO: Handle error somehow
		if(this->uart->LSR & UART_LSR_RXFE) {
			if(this->rx_delegate) {
				// Async read
				(*this->rx_delegate)(UART_ERROR_GENERAL, const_cast<uint8_t*>(this->rx_buffer), this->rx_op_len);
			}
			else {
				// Sync read
				xSemaphoreGiveFromISR(this->rx_transfer_semaphore, &task_woken);
				this->rx_status = UART_ERROR_NONE;
			}
		}
		break;
	}
	case UART_IIR_INTID_RDA: case UART_IIR_INTID_CTI:
	{
		while(this->uart->LSR & UART_LSR_RDR) {
			if(this->rx_buffer_head_pos < this->rx_op_len) {
				this->rx_buffer[this->rx_buffer_head_pos++] = this->uart->RBR;
			}
			else {
				break;
			}

		}

		if(this->rx_buffer_head_pos >= this->rx_op_len) {
			// Read is over
			this->uart->IER &= ~(UART_IER_RBRINT | UART_IER_RLSINT);

			if(this->rx_delegate) {
				// Async read
				(*this->rx_delegate)(UART_ERROR_NONE, const_cast<uint8_t*>(this->rx_buffer), this->rx_op_len);
			}
			else {
				// Sync read
				xSemaphoreGiveFromISR(this->rx_transfer_semaphore, &task_woken);
			}
		}
		break;
	}
	}

	if(task_woken) {
		vPortYield();
	}
}
