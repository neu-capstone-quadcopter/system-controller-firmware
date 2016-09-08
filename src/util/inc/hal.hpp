/*
 * hal.hpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nigil
 */

#ifndef UTIL_INC_HAL_HPP_
#define UTIL_INC_HAL_HPP_

#include "driver.hpp"

namespace hal {
	typedef enum driver_identifier{
		LED_0,
		LED_1,
		TELEM_CC1120,
	} driver_identifier;

	void init(void);
	Driver *get_driver(driver_identifier id);
}

#endif /* UTIL_INC_HAL_HPP_ */
