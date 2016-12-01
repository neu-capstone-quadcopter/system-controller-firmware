/*
 * mb1240.cpp
 *
 *  Created on: Nov 22, 2016
 *      Author: nigil
 */

#ifndef DRIVERS_INC_MB1240_HPP_
#define DRIVERS_INC_MB1240_HPP_

#include "chip.h"

#include "driver.hpp"

class Mb1240 : public Driver {
public:
	Mb1240(LPC_TIMER_T *ultrasonic_timer, uint8_t timer_cap_ch);
	void init_driver();
	bool get_current_range_mm(uint16_t *range);
	void timer_interrupt_handler();
private:
	LPC_TIMER_T *timer;
	uint8_t timer_cap_ch;
	uint32_t last_rising_cap;
	uint16_t current_range_mm;
	bool is_awaiting_rising_edge = true;
	bool has_read_last_value = true;
	void init_capture_timer();
};



#endif /* DRIVERS_INC_MB1240_CPP_ */
