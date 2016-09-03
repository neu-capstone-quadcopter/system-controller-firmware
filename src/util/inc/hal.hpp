/*
 * hal.hpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nigil
 */

#ifndef UTIL_INC_HAL_HPP_
#define UTIL_INC_HAL_HPP_

#include "driver.hpp"

typedef enum hal_driver_identifier{
	LED_0,
	LED_1,
} hal_driver_identifier;

void hal_init(void);
Driver *hal_get_driver(hal_driver_identifier id);

#endif /* UTIL_INC_HAL_HPP_ */
