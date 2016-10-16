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
#define CC1120_SSP_CLOCKMODE SSP_CLOCK_CPHA1_CPOL1

#define SPICMD_W_REGISTER(reg) (0x00 | reg)
#define SPICMD_W_BURST_REGISTER(reg) (0x40 | reg)
#define SPICMD_W_EXTENDED_REGISTER 0x2F
#define SPICMD_W__BURST_EXTENDED_REGISTER 0x6F
#define SPICMD_R_REGISTER(reg) (0x80 | reg)
#define SPICMD_R_BURST_REGISTER(reg) (0xC0 | reg)
#define SPICMD_R_EXTENDED_REGISTER 0xAF
#define SPICMD_R_BURST_EXTENDED_REGISTER 0xEF

#define REG_IOCONFIG3 0x00
#define REG_IOCONFIG2 0x01
#define REG_IOCONFIG1 0x02
#define REG_IOCONFIG0 0x06
#define REG_SYNC_CFG1 0x08
#define REG_DEVIATION_M 0x0A
#define REG_MODCFG_DEV_E 0x0B
#define REG_DCFILT_CFG 0x0C
#define REG_IQIC 0x10
#define REG_CHAN_BW 0x11
#define REG_MDMCFG0 0x13
#define REG_AGC_CFG1 0x1C
#define REG_AGC_CFG0 0x1D
#define REG_FIFO_CFG 0x1E
#define REG_FS_CFG 0x21
#define REG_PKT_CFG0 0x28
#define REG_PKT_LEN 0x2E
#define REG_IF_MIX_CFG 0x00
#define REG_FREQOFF_CFG 0x01
#define REG_FREQ2 0x0C
#define REG_FREQ1 0x0D
#define REG_FREQ0 0x0E
#define REG_FS_DIG1 0x12
#define REG_FS_DIG0 0x13
#define REG_FS_CAL1 0x16
#define REG_FS_CAL0 0x17
#define REG_FS_DIVTWO 0x19
#define REG_FS_DSM0 0x1B
#define REG_FS_DVC0 0x1D
#define REG_FS_PFD 0x1F
#define REG_FS_PRE 0x20
#define REG_FS_REG_DIV_CML 0x21
#define REG_FS_SPARE 0x22
#define REG_FS_VCO0 0x27
#define REG_XOSC5 0x32
#define REG_XOSC1 0x36

#define INIT_IOCONFIG3 0x00
#define INIT_IOCONFIG2 0x02
#define INIT_IOCONFIG1 0xB0
#define INIT_IOCONFIG0 0x10
#define INIT_SYNC_CFG1 0x0B
#define INIT_DEVIATION_M 0x48
#define INIT_MODCFG_DEV_E 0x05
#define INIT_DCFILT_CFG 0x1C
#define INIT_IQIC 0x00
#define INIT_CHAN_BW 0x04
#define INIT_MDMCFG0 0x05
#define INIT_AGC_CFG1 0xA9
#define INIT_AGC_CFG0 0xCF
#define INIT_FIFO_CFG 0x00
#define INIT_FS_CFG 0x12
#define INIT_PKT_CFG0 0x20
#define INIT_PKT_LEN 0xFF
#define INIT_IF_MIX_CFG 0x00
#define INIT_FREQOFF_CFG 0x22
#define INIT_FREQ2 0x70
#define INIT_FREQ1 0xF3
#define INIT_FREQ0 0x33
#define INIT_FS_DIG1 0x00
#define INIT_FS_DIG0 0x5F
#define INIT_FS_CAL1 0x40
#define INIT_FS_CAL0 0x0E
#define INIT_FS_DIVTWO 0x03
#define INIT_FS_DSM0 0x33
#define INIT_FS_DVC0 0x17
#define INIT_FS_PFD 0x50
#define INIT_FS_PRE 0x6E
#define INIT_FS_REG_DIV_CML 0x14
#define INIT_FS_SPARE 0xAC
#define INIT_FS_VCO0 0xB4
#define INIT_XOSC5 0x0E
#define INIT_XOSC1 0x03

namespace driver {

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
	vTaskDelay(5);

	write_verify_register(REG_IOCONFIG3, INIT_IOCONFIG3);
	write_verify_register(REG_IOCONFIG2, INIT_IOCONFIG2);
	write_verify_register(REG_IOCONFIG1, INIT_IOCONFIG1);
	write_verify_register(REG_IOCONFIG0, INIT_IOCONFIG0);
	write_verify_register(REG_SYNC_CFG1, INIT_SYNC_CFG1);
	write_verify_register(REG_DEVIATION_M, INIT_DEVIATION_M);
	write_verify_register(REG_MODCFG_DEV_E, INIT_MODCFG_DEV_E);
	write_verify_register(REG_DCFILT_CFG, INIT_DCFILT_CFG);
	write_verify_register(REG_IQIC, INIT_IQIC);
	write_verify_register(REG_CHAN_BW, INIT_CHAN_BW);
	write_verify_register(REG_MDMCFG0, INIT_MDMCFG0);
	write_verify_register(REG_AGC_CFG1, INIT_AGC_CFG1);
	write_verify_register(REG_AGC_CFG0, INIT_AGC_CFG0);
	write_verify_register(REG_FIFO_CFG, INIT_FIFO_CFG);
	write_verify_register(REG_FS_CFG, INIT_FS_CFG);
	write_verify_register(REG_PKT_CFG0, INIT_PKT_CFG0);
	write_verify_register(REG_PKT_LEN, INIT_PKT_LEN);
	write_verify_extended_register(REG_IF_MIX_CFG, INIT_IF_MIX_CFG);
	write_verify_extended_register(REG_FREQOFF_CFG, INIT_FREQOFF_CFG);
	write_verify_extended_register(REG_FREQ2, INIT_FREQ2);
	write_verify_extended_register(REG_FREQ1, INIT_FREQ1);
	write_verify_extended_register(REG_FREQ0, INIT_FREQ0);
	write_verify_extended_register(REG_FS_DIG1, INIT_FS_DIG1);
	write_verify_extended_register(REG_FS_DIG0, INIT_FS_DIG0);
	write_verify_extended_register(REG_FS_CAL1, INIT_FS_CAL1);
	write_verify_extended_register(REG_FS_CAL0, INIT_FS_CAL0);
	write_verify_extended_register(REG_FS_DIVTWO, INIT_FS_DIVTWO);
	write_verify_extended_register(REG_FS_DSM0, INIT_FS_DSM0);
	write_verify_extended_register(REG_FS_DVC0, INIT_FS_DVC0);
	write_verify_extended_register(REG_FS_PFD, INIT_FS_PFD);
	write_verify_extended_register(REG_FS_PRE, INIT_FS_PRE);
	write_verify_extended_register(REG_FS_REG_DIV_CML, INIT_FS_REG_DIV_CML);
	write_verify_extended_register(REG_FS_SPARE, INIT_FS_SPARE);
	write_verify_extended_register(REG_FS_VCO0, INIT_FS_VCO0);
	write_verify_extended_register(REG_XOSC5, INIT_XOSC5);
	write_verify_extended_register(REG_XOSC1, INIT_XOSC1);

	if (this->error_status == WRITE_VERIFY_ERROR) {
		// do something about the error, eh!
	}

	access_command_strobe(CommandStrobeAddress::SNOP);

	/*uint8_t test_data[4] = {0x01, 0x02, 0x03, 0x04};
	uint8_t test_read[6];
	read_extended_register_burst(0x00, test_read, 4);
	write_extended_register_burst(0x00, test_data, 4);
	read_extended_register_burst(0x00, test_read, 4);*/
}

void Cc1120::reset(void) {
	Chip_GPIO_WritePortBit(LPC_GPIO, 2, 13, 0);
	vTaskDelay(5);
	Chip_GPIO_WritePortBit(LPC_GPIO, 2, 13, 1);
}

void Cc1120::gpio3_set_interrupt_pin_handler(Cc1120GpioInterruptHandler callback) {
	gpio3_interrupt_callback = callback;
}

void Cc1120::gpio2_set_interrupt_pin_handler(Cc1120GpioInterruptHandler callback) {
	gpio2_interrupt_callback = callback;
}

void Cc1120::gpio0_set_interrupt_pin_handler(Cc1120GpioInterruptHandler callback) {
	gpio0_interrupt_callback = callback;
}

void Cc1120::write_register_single(uint8_t address, uint8_t data) {
	send_command(SPICMD_W_REGISTER(address), &data, NULL, 1);
}

void Cc1120::write_register_single_async(uint8_t address, uint8_t data, CC1120CommandCallback status) {
	configASSERT(0);
}

void Cc1120::write_register_burst(uint8_t address, uint8_t *data, uint8_t data_len) {
	send_command(SPICMD_W_BURST_REGISTER(address), data, NULL, data_len);
}

void write_register_burst_async(uint8_t address, uint8_t *data, uint8_t data_len, CC1120CommandCallback callback) {
	configASSERT(0);
}

void Cc1120::write_extended_register_single(uint8_t address, uint8_t data) {
	uint8_t command [2] = {SPICMD_W_EXTENDED_REGISTER, address};
	send_command_extended(command, &data, NULL, 1);
}

void Cc1120::write_extended_register_single_async(uint8_t address, uint8_t data, CC1120CommandCallback callback) {
	configASSERT(0);
}

void Cc1120::write_extended_register_burst(uint8_t address, uint8_t *data, uint8_t data_len) {
	uint8_t command [2] = {SPICMD_W__BURST_EXTENDED_REGISTER, address};
	send_command_extended(command, data, NULL, data_len);
}

void Cc1120::write_extended_register_burst_async(uint8_t address, uint8_t *data, uint8_t data_len, CC1120CommandCallback callback) {
	configASSERT(0);
}

void Cc1120::write_verify_register(uint8_t address, uint8_t data) {
	uint8_t result[2];
	write_register_single(address, data);
	read_register_single(address, result);
	if (data != result[1]) {
		this->error_status = WRITE_VERIFY_ERROR;
	}
}

void Cc1120::write_verify_register_async(uint8_t address, uint8_t data, CC1120CommandCallback callback) {
	configASSERT(0);
}

void Cc1120::write_verify_extended_register(uint8_t address, uint8_t data) {
	uint8_t result[3];
	write_extended_register_single(address, data);
	read_extended_register_single(address, result);
	if (data != result[2]) {
		this->error_status = WRITE_VERIFY_ERROR;
	}
}

void Cc1120::write_verify_extended_register_async(uint8_t address, uint8_t data, CC1120CommandCallback callback) {
	configASSERT(0);
}

void Cc1120::read_register_single(uint8_t address, uint8_t *data) {
	uint8_t dummy_data;
	send_command(SPICMD_R_REGISTER(address), &dummy_data, data, 1);
	this->raw_status = data[0];
}

void Cc1120::read_register_single_async(uint8_t address, uint8_t *data, CC1120CommandDataCallback callback) {
	configASSERT(0);
}

void Cc1120::read_register_burst(uint8_t address, uint8_t *data, uint8_t data_len) {
	uint8_t dummy_data[data_len];
	send_command(SPICMD_R_BURST_REGISTER(address), dummy_data, data, data_len);
	this->raw_status = data[0];
}

void Cc1120::read_register_burst_async(uint8_t address, uint8_t *data, uint8_t data_len, CC1120CommandDataCallback callback) {
	configASSERT(0);
}

void Cc1120::read_extended_register_single(uint8_t address, uint8_t *data) {
	uint8_t dummy_data;
	uint8_t command [2] = {SPICMD_R_EXTENDED_REGISTER, address};
	send_command_extended(command, &dummy_data, data, 1);
	this->raw_status = data[0];
}

void Cc1120::read_extended_register_single_async(uint8_t address, uint8_t *data, CC1120CommandDataCallback callback) {
	configASSERT(0);
}

void Cc1120::read_extended_register_burst(uint8_t address, uint8_t *data, uint8_t data_len) {
	uint8_t dummy_data[data_len];
	uint8_t command [2] = {SPICMD_R_BURST_EXTENDED_REGISTER, address};
	send_command_extended(command, dummy_data, data, data_len);
	this->raw_status = data[0];
}

void Cc1120::read_extended_register_burst_async(uint8_t address, uint8_t *data, uint8_t data_len, CC1120CommandDataCallback callback) {
	configASSERT(0);
}

void Cc1120::access_command_strobe(CommandStrobeAddress address) {
	send_command(static_cast<uint8_t>(address), NULL, NULL, 0);
}

void Cc1120::access_command_strobe_async(CommandStrobeAddress address, CC1120CommandCallback callback) {
	configASSERT(0);
}

void Cc1120::send_command(uint8_t command, uint8_t *tx_data, uint8_t *rx_data, uint8_t data_len) {
	uint8_t op_tx_data[data_len + 1];
	op_tx_data[0] = command;
	if (tx_data) {
		memcpy(op_tx_data + 1, tx_data, data_len);
	}

	this->ssp_device->read_write(op_tx_data, rx_data, data_len + 1);
}

void Cc1120::send_command_extended(uint8_t *command, uint8_t *tx_data, uint8_t *rx_data, uint8_t data_len) {
	uint8_t op_tx_data[data_len + 2];
	op_tx_data[0] = command[0];
	op_tx_data[1] = command[1];
	memcpy(op_tx_data + 2, tx_data, data_len);

	this->ssp_device->read_write(op_tx_data, rx_data, data_len + 2);
}

bool Cc1120::pinint_handler(void) {
	if (board::cc1120_is_gpio3_int()) {
		if (gpio3_interrupt_callback) {
			gpio3_interrupt_callback();
		}
		return true;
	}
	if (board::cc1120_is_gpio2_int()) {
		if (gpio2_interrupt_callback) {
			gpio2_interrupt_callback();
		}
		return true;
	}
	if (board::cc1120_is_gpio0_int()) {
		if (gpio0_interrupt_callback) {
			gpio0_interrupt_callback();
		}
		return true;
	}
	return false;
}
}
