/*
 * exampleled.cpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nigil
 */

#include "chip.h"

#include "cc1120.hpp"

Cc1120::Cc1120(LPC_SSP_T *cc1120_ssp) {
	this->cc1120_ssp = cc1120_ssp;
}

void Cc1120::init_driver() {
	Chip_SSP_Init(this->cc1120_ssp);

	this->ssp_format.frameFormat = SSP_FRAMEFORMAT_SPI;
	this->ssp_format.bits = SSP_BITS_8;
	this->ssp_format.clockMode = SSP_CLOCK_CPHA0_CPOL0;
	Chip_SSP_SetFormat(this->cc1120_ssp, this->ssp_format.bits,
			this->ssp_format.frameFormat, this->ssp_format.clockMode);

	Chip_SSP_Enable(this->cc1120_ssp);

	NVIC_EnableIRQ(SSP1_IRQn);
}

void SSPIRQHANDLER(void)
{

}
