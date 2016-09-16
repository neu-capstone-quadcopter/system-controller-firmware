/*
 * exampleled.cpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nigil
 */

#include "chip.h"

#include "cc1120.hpp"

#define CC1120_SCLK_FREQ 5000000
#define CC1120_SSP_FORMAT SSP_FRAMEFORMAT_SPI
#define CC1120_SSP_BITS SSP_BITS_8
#define CC1120_SSP_CLOCKMODE SSP_CLOCK_CPHA1_CPOL0

Cc1120::Cc1120(SspIo *ssp_device) {
	this->ssp_device = ssp_device;
}

void Cc1120::init_driver(void) {
	// SSP init is done by HAL
	this->ssp_device->set_clk_rate(CC1120_SCLK_FREQ);

	SSP_ConfigFormat ssp_format;
	ssp_format.frameFormat = CC1120_SSP_FORMAT;
	ssp_format.bits = CC1120_SSP_BITS;
	ssp_format.clockMode = CC1120_SSP_CLOCKMODE;

	this->ssp_device->set_format(ssp_format);
}

void Cc1120::init_device(void) {
	uint8_t test_data[4] = {0xAA, 0xCC, 0xAA, 0xCC};
	this->ssp_device->write(test_data, 4);
}
