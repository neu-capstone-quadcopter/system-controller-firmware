/*
 * load_switch_rail.hpp
 *
 *  Created on: Nov 17, 2016
 *      Author: bsoper
 */

#ifndef DRIVERS_INC_LOAD_SWITCH_RAIL_HPP_
#define DRIVERS_INC_LOAD_SWITCH_RAIL_HPP_

class LoadSwitch : Driver {
public:
	static void set_load_switch_navcmp(bool state);
	static void set_load_switch_fltctl(bool state);
	static void set_load_switch_gps(bool state);
	static void set_load_switch_radio(bool state);
};



#endif /* DRIVERS_INC_LOAD_SWITCH_RAIL_HPP_ */
