/*
 * board.hpp
 *
 *  Created on: Sep 16, 2016
 *      Author: nigil
 */

#ifndef UTIL_INC_BOARD_HPP_
#define UTIL_INC_BOARD_HPP_

#include "chip.h"

// CC1120 Pin Defines
#define CC1120_GPIO3_PORT 2
#define CC1120_GPIO3_INTPORT GPIOINT_PORT0
#define CC1120_GPIO3_PIN 8
#define CC1120_GPIO2_PORT 2
#define CC1120_GPIO2_INTPORT GPIOINT_PORT0
#define CC1120_GPIO2_PIN 7
#define CC1120_GPIO0_PORT 2
#define CC1120_GPIO0_INTPORT GPIOINT_PORT0
#define CC1120_GPIO0_PIN 6

namespace board {
	void setup_clocking(void);
	void ssp_init(LPC_SSP_T *ssp);
	void cc1120_init();
	bool cc1120_is_gpio3_int(void);
	bool cc1120_is_gpio2_int(void);
	bool cc1120_is_gpio0_int(void);
}

#endif /* UTIL_INC_BOARD_HPP_ */
