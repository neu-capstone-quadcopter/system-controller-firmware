/*
 * mb1240.cpp
 *
 *  Created on: Nov 22, 2016
 *      Author: nigil
 */

#ifndef DRIVERS_INC_MB1240_HPP_
#define DRIVERS_INC_MB1240_HPP_

#include "driver.hpp"

class Mb1240 : public Driver {
public:
	Mb1240();
	void init_driver();
private:
	void init_capture_timer();
};



#endif /* DRIVERS_INC_MB1240_CPP_ */
