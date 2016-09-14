/*
 * uart.cpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nate
 */

#include "chip.h"

#include "uart.hpp"

#include "FreeRTOS.h"

//Baud
#define DEFAULT_BAUD 115200

//Data Config
#define DEFAULT_WORD_LENGTH UART_LCR_WLEN8
#define DEFAULT_PARITY UART_LCR_PARITY_DIS
#define DEFAULT_STOP_BIT UART_LCR_SBS_1BIT

Uart::Uart(LPC_USART_T *uart) {
	this->uart = uart;
}

void Uart::config_data(uint32_t word_length, uint32_t parity, uint32_t stopbits){
	Chip_UART_ConfigData(this->uart, (word_length | stopbits));
}

IRQn_Type Uart::get_NVIC_IRQ(void){

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

void Uart::uart_interrupt_handler(void){

}

void Uart::write(uint8_t* data, uint8_t length){
	//Error Checking?

	//Write to the transmission buffer
	Chip_UART_SendRB(this->uart, &this->txring, data, length);

	//Return some value -- bool?
}

void Uart::read(uint8_t* data, uint8_t length){
	//Error Checking?

	//Read from read buffer
	int bytes = Chip_UART_ReadRB(this->uart, this->rxring, data, length);
}



void Uart::init_driver(void) {

	//UART init
	Chip_UART_Init(this->uart);

	//Set Baud
	set_baud(DEFAULT_BAUD);

	//Config Data
	config_data(DEFAULT_WORD_LENGTH, DEFAULT_PARITY, DEFAULT_STOP_BIT);

	//Enable Transmission
	Chip_UART_TXEnable(this->uart);

	//Init Rx/Tx Ring Buffers
	RingBuffer_Init(&this->rxring, this->rxbuff, 1, RX_BUFFER_SIZE);
	RingBuffer_Init(&this->rxring, this->rxbuff, 1, RX_BUFFER_SIZE);

	//FIFO stuff
	Chip_UART_SetupFIFOS(this->uart, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2 |
	                       UART_FCR_RX_RS | UART_FCR_TX_RS));
	//Enable Interrupts
	Chip_UART_IntEnable(this->uart, (UART_IER_RBRINT | UART_IER_RLSINT));

	//Enable UART -> NVIC
		NVIC_EnableIRQ(get_NVIC_IRQ());


}

