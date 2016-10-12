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

class GpdmaChannel {
public:
	GpdmaChannel(LPC_GPDMA_T *gpdma, uint8_t channel_num);
private:
	LPC_GPDMA_T *gpdma_base;
	uint8_t channel_num;
};

class GpdmaManager : public Driver {
public:
	GpdmaManager(LPC_GPDMA_T *gpdma);
	void init_driver(void);
	GpdmaChannel *allocate_channel(uint8_t channel_num);
	void release_channel(uint8_t channel_num);
private:
	LPC_GPDMA_T *gpdma_base;
	GpdmaChannel *channels[GPDMA_NUMBER_CHANNELS];
	uint8_t channel_usage;
};



#endif /* DRIVERS_INC_GPDMA_HPP_ */
