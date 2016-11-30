/*
 * load_switch_rail.hpp
 *
 *  Created on: Nov 17, 2016
 *      Author: bsoper
 */

#ifndef DRIVERS_INC_GPIO_HPP_
#define DRIVERS_INC_GPIO_HPP_

#include "driver.hpp"

class GpioManager : public Driver {
public:
	GpioManager() {};
	void init_driver(void);
	void set_load_switch_navcmp(bool state);
	void set_load_switch_fltctl(bool state);
	void set_load_switch_gps(bool state);
	void set_load_switch_radio(bool state);
	void set_pwm_output_en(bool state);
};



#endif /* DRIVERS_INC_GPIO_HPP_ */
