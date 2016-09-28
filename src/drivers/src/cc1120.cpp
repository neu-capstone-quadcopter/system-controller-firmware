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

#define SPICMD_R_REGISTER(reg) (0x80 | reg)
#define SPICMD_W_REGISTER(reg) (0x00 | reg)

Cc1120::Cc1120(SspIo *ssp_device) {
	this->ssp_device = ssp_device;
}

void Cc1120::init_driver(void) {
	board::cc1120_init();

	// SSP init is done by HAL
	this->ssp_device->set_clk_rate(CC1120_SCLK_FREQ);

	SSP_ConfigFormat ssp_format;
	ssp_format.frameFormat = CC1120_SSP_FORMAT;
	ssp_format.bits = CC1120_SSP_BITS;
	ssp_format.clockMode = CC1120_SSP_CLOCKMODE;

	this->ssp_device->set_format(ssp_format);
}

void Cc1120::init_device(void) {
	reset();

	uint8_t test_data[1] = {0xA2};
	uint8_t test_read[4];
	//this->ssp_device->read_write(test_data, t, 4);
	read_register_single(0x04, test_read);
	write_register_single(0x04, test_data);
	read_register_single(0x04, test_read);
}

void Cc1120::reset(void) {
	Chip_GPIO_WritePortBit(LPC_GPIO, 2, 13, 0);
	vTaskDelay(5);
	Chip_GPIO_WritePortBit(LPC_GPIO, 2, 13, 1);
}

void Cc1120::send_command(uint8_t command, uint8_t *tx_data, uint8_t *rx_data, uint8_t data_len) {
	uint8_t op_tx_data[8];
	op_tx_data[0] = command;
	memcpy(op_tx_data + 1, tx_data, data_len);

	this->ssp_device->read_write(op_tx_data, rx_data, data_len);
}

void Cc1120::write_register(uint8_t address, uint8_t *data, uint8_t data_len) {
	send_command(SPICMD_W_REGISTER(address), data, NULL, data_len);
}

void Cc1120::write_register_single(uint8_t address, uint8_t *data) {
	write_register(address, data, 1);
}

void Cc1120::read_register(uint8_t address, uint8_t *data, uint8_t data_len) {
	uint8_t dummy_data[8];
	send_command(SPICMD_R_REGISTER(address), dummy_data ,data, data_len);
}

void Cc1120::read_register_single(uint8_t address, uint8_t *data) {
	read_register(address, data, 1);
}
