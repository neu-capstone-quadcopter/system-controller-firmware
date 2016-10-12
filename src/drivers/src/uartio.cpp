/*
 * uart.cpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nate
 */

#include <uartio.hpp>
#include "chip.h"

#include "FreeRTOS.h"
#include "board.hpp"


#define DEFAULT_BAUD 115200

//Data Config
#define DEFAULT_WORD_LENGTH UART_LCR_WLEN8
#define DEFAULT_PARITY UART_LCR_PARITY_DIS
#define DEFAULT_STOP_BIT UART_LCR_SBS_1BIT

UartIo::UartIo(LPC_USART_T *uart) {
	this->uart = uart;
}

void UartIo::configData(uint32_t word_length, uint32_t parity, uint32_t stopbits){
	Chip_UART_ConfigData(this->uart, (word_length | stopbits));
}

IRQn_Type UartIo::getNVICIRQ(void){

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
}

void UartIo::uartInterruptHandler(void){
	Chip_UART_IRQRBHandler(this->uart, &this->rxring, &this->txring);

	if(write_in_progress && RingBuffer_IsEmpty(&this->txring));
	{
		xSemaphoreGiveFromISR(this->tx_transfer_semaphore, NULL);
		write_in_progress = false;
	}

	if(this->async_read_in_progress && !RingBuffer_IsEmpty(&this->rxring))
	{
		uint8_t data;
		Chip_UART_ReadRB(this->uart, &this->rxring, &data, 1);
		this->callback(data);
		this->async_read_in_progress = false;

	}

	else if(this->read_in_progress)
	{
		if(!RingBuffer_IsEmpty(&this->rxring))
		{
			this->read_in_progress = false;
			xSemaphoreGiveFromISR(this->rx_transfer_semaphore, NULL);
		}
	}
}

void UartIo::write(uint8_t* data, uint8_t length)
{
	//Write to the transmission buffer
	write_in_progress = true;
	Chip_UART_SendRB(this->uart, &this->txring, data, length);
	xSemaphoreTake(this->tx_transfer_semaphore, portMAX_DELAY);

}

void UartIo::write_dma(uint8_t* data, uint8_t length, GpdmaChannel *dma_channel) {
	if(dma_channel->is_active()) {
		// Error here
	}

	this->uart->FCR |= UART_FCR_DMAMODE_SEL;

	dma_channel->start_transfer(
			(uint32_t)data,
			get_tx_dmareq(),
			GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA,
			length);
}

void UartIo::read(uint8_t* data, uint8_t length){
	//Error Checking?

	//Read from read buffer
	this->read_in_progress = true;
	xSemaphoreTake(this->rx_transfer_semaphore, portMAX_DELAY);

	Chip_UART_ReadRB(this->uart, &this->rxring, data, length);
}

void UartIo::readCharAsync(uart_char_read_callback callback)
{
	this->callback = callback;
	this->async_read_in_progress = true;

}



void UartIo::init_driver(void)
{

	//UART init
	board::uart_init(this->uart);
	Chip_UART_Init(this->uart);

	//Semaphores
	this->tx_transfer_semaphore = xSemaphoreCreateBinary();
	this->rx_transfer_semaphore = xSemaphoreCreateBinary();

	//Set Baud
	setBaud(DEFAULT_BAUD);

	//Configure our data frame
	configData(DEFAULT_WORD_LENGTH, DEFAULT_PARITY, DEFAULT_STOP_BIT);

	//Enable Transmission
	Chip_UART_TXEnable(this->uart);

	//Init Rx/Tx Ring Buffers
	RingBuffer_Init(&this->rxring, this->rxbuff, 1, RX_BUFFER_SIZE);
	RingBuffer_Init(&this->txring, this->txbuff, 1, TX_BUFFER_SIZE);

	//FIFO stuff
	Chip_UART_SetupFIFOS(this->uart, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2 |
	                       UART_FCR_RX_RS | UART_FCR_TX_RS));
	//Enable Interrupts
	Chip_UART_IntEnable(this->uart, (UART_IER_RBRINT | UART_IER_RLSINT));
	NVIC_EnableIRQ(getNVICIRQ());

	//Can remove after testing...
	Chip_GPIO_WriteDirBit(LPC_GPIO,2,10,true);

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
}
