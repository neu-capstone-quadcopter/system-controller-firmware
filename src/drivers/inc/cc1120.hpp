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
	void send_command(uint8_t command, uint8_t *tx_data, uint8_t *rx_data, uint8_t data_len);
	void write_register(uint8_t address, uint8_t *data, uint8_t data_len);
	void write_register_single(uint8_t address, uint8_t *data);
	void read_register(uint8_t address, uint8_t *data, uint8_t data_len);
	void read_register_single(uint8_t address, uint8_t *data);

private:
	SspIo *ssp_device;
};



#endif /* DRIVERS_INC_CC1120_HPP_ */
