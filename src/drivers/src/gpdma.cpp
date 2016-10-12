/*
 * gpdma.cpp
 *
 *  Created on: Oct 12, 2016
 *      Author: nigil
 */

#include "chip.h"
#include "gpdma.hpp"

Gpdma::Gpdma(void) {
	this->gpdma_base = LPC_GPDMA;
}

void Gpdma::init_driver(void) {
	Chip_GPDMA_Init(this->gpdma_base);
}

