/*
 * cc1120.hpp
 *
 *  Created on: Sep 8, 2016
 *      Author: jknichel
 */

#ifndef DRIVERS_INC_CC1120_HPP_
#define DRIVERS_INC_CC1120_HPP_

#include "board.hpp"
#include "chip.h"
#include <cstdint>
#include <cstring>
#include "driver.hpp"
#include "sspio.hpp"

#include "FreeRTOS.h"
#include "task.h"

typedef void (*cc1120_callback_t)(void);

typedef enum {
	SRES = 0x30,
	SFSTXON = 0x31,
	SXOFF = 0x32,
	SCAL = 0x33,
	SRX = 0x34,
	STX = 0x35,
	SIDLE = 0x36,
	SAFC = 0x37,
	SWOR = 0x38,
	SPWD = 0x39,
	SFRX = 0x3A,
	SFTX = 0x3B,
	SWORRST = 0x3C,
	SNOP = 0x3D
} command_strobe_address;

class Cc1120 : public Driver {
public:
	Cc1120(SspIo *ssp_device);
	void init_driver(void);
	void init_device(void);
	void reset();
	void gpio3_set_interrupt_pin_handler(cc1120_callback_t callback);
	void gpio2_set_interrupt_pin_handler(cc1120_callback_t callback);
	void gpio0_set_interrupt_pin_handler(cc1120_callback_t callback);
	void write_register_burst(uint8_t address, uint8_t *data, uint8_t data_len);
	void write_extended_register_single(uint8_t address, uint8_t data);
	void write_register_single(uint8_t address, uint8_t data);
	void write_extended_register_burst(uint8_t address, uint8_t *data, uint8_t data_len);
	void write_verify_register(uint8_t address, uint8_t data);
	void write_verify_extended_register(uint8_t address, uint8_t data);
	void read_register_burst(uint8_t address, uint8_t *data, uint8_t data_len);
	void read_extended_register_burst(uint8_t address, uint8_t *data, uint8_t data_len);
	void read_register_single(uint8_t address, uint8_t *data);
	void read_extended_register_single(uint8_t address, uint8_t *data);
	void access_command_strobe(command_strobe_address address);
	void send_command(uint8_t command, uint8_t *tx_data, uint8_t *rx_data, uint8_t data_len);
	void send_command_extended(uint8_t *command, uint8_t *tx_data, uint8_t *rx_data, uint8_t data_len);
	bool pinint_handler(void);

private:
	SspIo *ssp_device;
	uint8_t raw_status;
	enum errors { NONE, WRITE_VERIFY_ERROR };
	errors error_status = NONE;
	cc1120_callback_t gpio3_interrupt_callback = NULL;
	cc1120_callback_t gpio2_interrupt_callback = NULL;
	cc1120_callback_t gpio0_interrupt_callback = NULL;
};



#endif /* DRIVERS_INC_CC1120_HPP_ */
