/*
 * uart.hpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nate
 */

#ifndef DRIVERS_INC_UART_HPP_
#define DRIVERS_INC_UART_HPP_

#include <cstdint> //Defines types
#include "driver.hpp"
#include "chip.h"

#define RX_BUFFER_SIZE 32
#define TX_BUFFER_SIZE 128

class Uart : public Driver {
public:
	Uart(LPC_USART_T *uart);
	void init_driver(void);
	inline void set_baud(uint32_t baud) {this->baud_rate = baud;}
	void config_data(uint32_t word_length, uint32_t parity, uint32_t stopbits);

private:
	IRQn_Type get_NVIC_IRQ(void);

	LPC_USART_T *uart;
	uint32_t baud_rate;
	RINGBUFF_T txring;
	RINGBUFF_T rxring;
	uint8_t rxbuff[RX_BUFFER_SIZE];
	uint8_t txbuff[TX_BUFFER_SIZE];
};



#endif /* DRIVERS_INC_EXAMPLELED_HPP_ */
