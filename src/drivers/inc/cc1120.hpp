/*
 * cc1120.hpp
 *
 *  Created on: Sep 8, 2016
 *      Author: jknichel
 */

#ifndef DRIVERS_INC_CC1120_HPP_
#define DRIVERS_INC_CC1120_HPP_

#include "chip.h"
#include <cstdint>
#include "driver.hpp"
#include "sspio.hpp"

class Cc1120 : public Driver {
public:
	Cc1120(SspIo *ssp_device);
	void init_driver(void);
	void init_device(void);

private:
	SspIo *ssp_device;
};



#endif /* DRIVERS_INC_CC1120_HPP_ */
