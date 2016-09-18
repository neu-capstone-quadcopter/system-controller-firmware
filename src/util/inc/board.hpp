/*
 * board.hpp
 *
 *  Created on: Sep 16, 2016
 *      Author: nigil
 */

#ifndef UTIL_INC_BOARD_HPP_
#define UTIL_INC_BOARD_HPP_

#include "chip.h"

namespace board {
	void setup_clocking(void);
	void ssp_init(LPC_SSP_T *ssp);
	void uart_init(LPC_USART_T *uart);
}

#endif /* UTIL_INC_BOARD_HPP_ */
