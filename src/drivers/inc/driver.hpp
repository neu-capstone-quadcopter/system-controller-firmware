/*
 * driver.hpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nigil
 */

#ifndef UTIL_INC_DRIVER_HPP_
#define UTIL_INC_DRIVER_HPP_

class Driver {
public:
	Driver(void) {}
	virtual void init_driver(void) = 0;
};



#endif /* UTIL_INC_DRIVER_HPP_ */
