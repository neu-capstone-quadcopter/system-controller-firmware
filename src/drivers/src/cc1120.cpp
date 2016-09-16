/*
 * exampleled.cpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nigil
 */

#include "chip.h"

#include "cc1120.hpp"

Cc1120::Cc1120(SspIo *ssp_device) {
	this->ssp_device = ssp_device;
}

void Cc1120::init_driver(void) {
	// SSP init is done by HAL
}

void Cc1120::init_device(void) {
	uint8_t test_data[] = {0xAA, 0xCC, 0xAA, 0xCC};
	this->ssp_device->write(test_data, 4);
}
