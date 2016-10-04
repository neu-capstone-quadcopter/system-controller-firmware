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
#define CC1120_GPIO3_PORT 0
#define CC1120_GPIO3_INTPORT GPIOINT_PORT0
#define CC1120_GPIO3_PIN 0
#define CC1120_GPIO2_PORT 0
#define CC1120_GPIO2_INTPORT GPIOINT_PORT0
#define CC1120_GPIO2_PIN 0
#define CC1120_GPIO1_PORT 0
#define CC1120_GPIO1_INTPORT GPIOINT_PORT0
#define CC1120_GPIO1_PIN 0

namespace board {
	void setup_clocking(void);
	void ssp_init(LPC_SSP_T *ssp);
	void cc1120_init();
}

#endif /* UTIL_INC_BOARD_HPP_ */
