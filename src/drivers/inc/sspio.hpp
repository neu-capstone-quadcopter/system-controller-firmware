/*
 * sspio.hpp
 *
 *  Created on: Sep 13, 2016
 *      Author: nigil
 */

#ifndef DRIVERS_INC_SSPIO_HPP_
#define DRIVERS_INC_SSPIO_HPP_

#include <cstddef>
#include <cstdint>

#include "chip.h"
#include "driver.hpp"

#define BUFFER_SIZE 200

class SspIo : public Driver {
public:
	SspIo(LPC_SSP_T *ssp_base);
	void init_driver(void);
	void write(uint8_t *data, size_t len);
	void read(uint8_t *data, size_t len);
	void ssp_interrupt_handler(void);
private:
	LPC_SSP_T *ssp_base;
	SSP_ConfigFormat ssp_format;
	uint8_t tx_buffer[BUFFER_SIZE];
	uint8_t rx_buffer[BUFFER_SIZE];
};



#endif /* DRIVERS_INC_SSPIO_HPP_ */
