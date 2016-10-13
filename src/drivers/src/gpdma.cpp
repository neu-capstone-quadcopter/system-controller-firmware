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

bool GpdmaChannel::is_active(void) {
	return this->gpdma_base->ENBLDCHNS & (1 << channel_num);
}

void GpdmaChannel::register_callback(DmaHandlerFunctor *callback) {
	this->callback = callback;
}
/*
void GpdmaChannel::program() {
	Chip_GPDMA_ChannelCmd(this->gpdma_base, this->channel_num, ENABLE);
}*/

void GpdmaChannel::start_transfer(uint32_t src, uint32_t dst, GPDMA_FLOW_CONTROL_T type, uint32_t len) {
	Chip_GPDMA_Transfer(this->gpdma_base, this->channel_num, src, dst, type, len);
}

void GpdmaChannel::interrupt_handler(void) {
	this->gpdma_base->INTTCCLEAR |= (1 << this->channel_num);
	// TODO: Probably should check status more and for errors and such and pass to callback
	if(this->callback) {
		this->callback->dma_handler(DMA_ERROR_NONE);
	}
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
	NVIC_EnableIRQ(DMA_IRQn);
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
	this->channels[channel_num]->callback = 0;
}

void GpdmaManager::interrupt_handler(void) {
	// Check which channel produced interrupt and call interrupt handler
	for(uint8_t i = 0; i < GPDMA_NUMBER_CHANNELS; i++) {
		if(this->gpdma_base->INTSTAT & (1 << i)) {
			this->channels[i]->interrupt_handler();
		}
	}
}
