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

class Cc1120 : public Driver {
public:
	Cc1120(SspIo *ssp_device);
	void init_driver(void);
	void init_device(void);
	void reset();
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
	void send_command(uint8_t command, uint8_t *tx_data, uint8_t *rx_data, uint8_t data_len);
	void send_command_extended(uint8_t *command, uint8_t *tx_data, uint8_t *rx_data, uint8_t data_len);

private:
	SspIo *ssp_device;
	uint8_t raw_status;
	enum errors { NONE, WRITE_VERIFY_ERROR };
	errors error_status = NONE;
};



#endif /* DRIVERS_INC_CC1120_HPP_ */
