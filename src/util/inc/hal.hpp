/*
 * hal.hpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nigil
 */

#ifndef UTIL_INC_HAL_HPP_
#define UTIL_INC_HAL_HPP_

#include "driver.hpp"
#include "cd74hc4067.hpp"
#include "config.hpp"

namespace hal {
	// Add device drivers in order of dependence
	typedef enum driver_identifier{
		GPDMA_MAN = 0,
		TELEM_CC1120_SSP,
		CONSOLE_UART,
#ifdef IS_DEBUG_BOARD
		LED_0,
		LED_1,
#endif
		SENSOR_ADC,
		CD74HC4067,
		NAV_COMPUTER,
		FC_BLACKBOX_UART,
		FC_SBUS_UART,
		TELEM_CC1120,
		NUM_IDENTIFIERS = TELEM_CC1120 + 1 // Add all entries before this and update
	} driver_identifier;

	void init(void);
	template <class T>
	T *get_driver(driver_identifier id);
}

#endif /* UTIL_INC_HAL_HPP_ */
