/*
 * exampleled.cpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nigil
 */

#include "chip.h"

#include "cc1120.hpp"

uint8_t RxBuf, TxBuf;

Cc1120::Cc1120(LPC_SSP_T *cc1120_ssp) {
	this->cc1120_ssp = cc1120_ssp;
}

void Cc1120::init_driver() {
	// Select port 0, pins 6-9 for SSP1. No Addition mode
	Chip_IOCON_PinMux(LPC_IOCON, 0, 6, IOCON_MODE_INACT, IOCON_FUNC2);
	Chip_IOCON_PinMux(LPC_IOCON, 0, 7, IOCON_MODE_INACT, IOCON_FUNC2);
	Chip_IOCON_PinMux(LPC_IOCON, 0, 8, IOCON_MODE_INACT, IOCON_FUNC2);
	Chip_IOCON_PinMux(LPC_IOCON, 0, 9, IOCON_MODE_INACT, IOCON_FUNC2);

	Chip_SSP_Init(this->cc1120_ssp);

	this->ssp_format.frameFormat = SSP_FRAMEFORMAT_SPI;
	this->ssp_format.bits = SSP_BITS_8;
	this->ssp_format.clockMode = SSP_CLOCK_CPHA0_CPOL0;
	Chip_SSP_SetFormat(this->cc1120_ssp, this->ssp_format.bits,
			this->ssp_format.frameFormat, this->ssp_format.clockMode);

	Chip_SSP_SetMaster(this->cc1120_ssp, 1);

	Chip_SSP_Enable(this->cc1120_ssp);

	NVIC_EnableIRQ(SSP1_IRQn);
}

void Cc1120::ssp_write(uint8_t data) {
	Chip_SSP_DATA_SETUP_T xf_setup;
	xf_setup.length = 1;
	xf_setup.tx_data = &TxBuf;
	xf_setup.rx_data = &RxBuf;

	Chip_SSP_Int_RWFrames8Bits(this->cc1120_ssp, &xf_setup);
}

void Cc1120::ssp_interrupt_handler(void) {

}
