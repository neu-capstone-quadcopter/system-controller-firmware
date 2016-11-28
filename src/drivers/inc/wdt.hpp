/*
 * wdt.hpp
 *
 *  Created on: Nov 27, 2016
 *      Author: bsoper
 */

#ifndef DRIVERS_INC_WDT_HPP_
#define DRIVERS_INC_WDT_HPP_

#include <cstdint>
#include "driver.hpp"

class WDT : public Driver {
public:
	WDT(uint32_t freq);
	void init_driver(void);
	void feed();
	void start();
private:
	uint32_t frequency;
};



#endif /* DRIVERS_INC_WDT_HPP_ */
