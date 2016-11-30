/*
 * load_switch_rail.hpp
 *
 *  Created on: Nov 17, 2016
 *      Author: bsoper
 */

#ifndef DRIVERS_INC_LOAD_SWITCH_RAIL_HPP_
#define DRIVERS_INC_LOAD_SWITCH_RAIL_HPP_

#include "driver.hpp"

class LoadSwitch : public Driver {
public:
	LoadSwitch() {};
	void init_driver(void);
	void set_load_switch_navcmp(bool state);
	void set_load_switch_fltctl(bool state);
	void set_load_switch_gps(bool state);
	void set_load_switch_radio(bool state);
	void set_hw_arm(bool state);
};



#endif /* DRIVERS_INC_LOAD_SWITCH_RAIL_HPP_ */
