/*
 * gpdma.hpp
 *
 *  Created on: Oct 12, 2016
 *      Author: nigil
 */

#ifndef DRIVERS_INC_GPDMA_HPP_
#define DRIVERS_INC_GPDMA_HPP_

#include <cstdint>
#include "driver.hpp"

class Gpdma : public Driver {
public:
	Gpdma(void);
	void init_driver(void);
private:
	LPC_GPDMA_T *gpdma_base;
};



#endif /* DRIVERS_INC_GPDMA_HPP_ */
