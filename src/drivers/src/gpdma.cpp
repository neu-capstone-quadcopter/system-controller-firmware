/*
 * gpdma.cpp
 *
 *  Created on: Oct 12, 2016
 *      Author: nigil
 */

#include "chip.h"
#include "gpdma.hpp"

/*
 * GpdmaChannel Impl
 */
GpdmaChannel::GpdmaChannel(LPC_GPDMA_T *gpdma, uint8_t channel_num) {
	this->gpdma_base = gpdma;
	this->channel_num = channel_num;
}


/*
 * Gpdma Impl
 */
GpdmaManager::GpdmaManager(LPC_GPDMA_T *gpdma) {
	this->gpdma_base = gpdma;

	for(uint8_t i = 0; i < GPDMA_NUMBER_CHANNELS; i++) {
		this->channels[i] = new GpdmaChannel(gpdma, i);
	}
}

void GpdmaManager::init_driver(void) {
	Chip_GPDMA_Init(this->gpdma_base);
}

GpdmaChannel *GpdmaManager::allocate_channel(uint8_t channel_num) {
	if(this->channel_usage & (1 << channel_num)) {
		// Channel already allocated
		return (GpdmaChannel*)NULL;
	}

	this->channel_usage |= (1 << channel_num);

	return this->channels[channel_num];
}

void GpdmaManager::release_channel(uint8_t channel_num) {
	this->channel_usage &= ~(1 << channel_num);
}
