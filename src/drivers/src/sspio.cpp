/*
 * sspio.cpp
 *
 *  Created on: Sep 13, 2016
 *      Author: nigil
 */

#include <cstring>

#include "sspio.hpp"
#include "board.hpp"

#define DEFAULT_FRAMEFORMAT SSP_FRAMEFORMAT_SPI
#define DEFAULT_BITS SSP_BITS_8
#define DEFAULT_CLOCKMODE SSP_CLOCK_CPHA0_CPOL0

SspIo::SspIo(LPC_SSP_T *ssp_base) {
	this->ssp_base = ssp_base;
}

void SspIo::init_driver() {
	board::ssp_init(this->ssp_base);
	Chip_SSP_Init(this->ssp_base);

	Chip_SSP_SetFormat(this->ssp_base, DEFAULT_FRAMEFORMAT, DEFAULT_BITS, DEFAULT_CLOCKMODE);

	Chip_SSP_SetMaster(this->ssp_base, 1);

	Chip_SSP_Enable(this->ssp_base);

	NVIC_EnableIRQ(SSP1_IRQn);
}

void SspIo::set_clk_rate(uint32_t freq) {
	Chip_SSP_SetBitRate(this->ssp_base, freq);
}

void SspIo::set_format(SSP_ConfigFormat format) {
	Chip_SSP_SetFormat(this->ssp_base, format.bits, format.frameFormat, format.clockMode);
}

void SspIo::write(uint8_t *data, size_t len) {
	memcpy(this->tx_buffer, data, sizeof(uint8_t) * len);

	this->xfer_setup.length = len;
	this->xfer_setup.tx_data = this->tx_buffer;
	this->xfer_setup.rx_data = this->rx_buffer;
	this->xfer_setup.tx_cnt = 0;
	this->xfer_setup.rx_cnt = 0;

	Chip_SSP_Int_FlushData(this->ssp_base);
	Chip_SSP_Int_RWFrames8Bits(this->ssp_base, &this->xfer_setup);
	Chip_SSP_Int_Enable(this->ssp_base);
}

void SspIo::read(uint8_t *data, size_t len) {

}

void SspIo::ssp_interrupt_handler(void) {
	Chip_SSP_Int_Disable(this->ssp_base);

	Chip_SSP_Int_RWFrames8Bits(this->ssp_base, &this->xfer_setup);

	if ((this->xfer_setup.rx_cnt != this->xfer_setup.length) ||
			(this->xfer_setup.tx_cnt != this->xfer_setup.length)) {
		Chip_SSP_Int_Enable(this->ssp_base);
	}
}

