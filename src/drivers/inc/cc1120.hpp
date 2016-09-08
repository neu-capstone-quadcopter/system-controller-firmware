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

class Cc1120 : public Driver {
public:
	Cc1120(LPC_SSP_T *cc1120_ssp);
	void init_driver(void);
private:
	LPC_SSP_T *cc1120_ssp;
	SSP_ConfigFormat ssp_format;

	void ssp_write(uint8_t data);
};



#endif /* DRIVERS_INC_CC1120_HPP_ */
