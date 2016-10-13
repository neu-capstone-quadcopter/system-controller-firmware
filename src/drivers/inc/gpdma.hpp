/*
 * gpdma.hpp
 *
 *  Created on: Oct 12, 2016
 *      Author: nigil
 */

#ifndef DRIVERS_INC_GPDMA_HPP_
#define DRIVERS_INC_GPDMA_HPP_

#include <cstdint>
#include <functional>
#include "driver.hpp"

enum DmaError {
	DMA_ERROR_NONE
};

typedef std::function<void (DmaError)> DmaHandlerLamda;

class GpdmaChannel {
friend class GpdmaManager;
public:
	GpdmaChannel(LPC_GPDMA_T *gpdma, uint8_t channel_num);
	bool is_active(void);
	void register_callback(DmaHandlerLamda callback);
	void start_transfer(uint32_t src, uint32_t dst, GPDMA_FLOW_CONTROL_T type, uint32_t len);
private:
	void interrupt_handler(void);
	LPC_GPDMA_T *gpdma_base;
	uint8_t channel_num;
	DmaHandlerLamda callback;
};

class GpdmaManager : public Driver {
public:
	GpdmaManager(LPC_GPDMA_T *gpdma);
	void init_driver(void);
	GpdmaChannel *allocate_channel(uint8_t channel_num);
	void release_channel(uint8_t channel_num);
	void interrupt_handler(void);
private:
	LPC_GPDMA_T *gpdma_base;
	GpdmaChannel *channels[GPDMA_NUMBER_CHANNELS];
	uint8_t channel_usage = 0;
};



#endif /* DRIVERS_INC_GPDMA_HPP_ */
