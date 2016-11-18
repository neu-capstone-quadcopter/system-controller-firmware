/*
 * load_switch_rail.cpp
 *
 *  Created on: Nov 17, 2016
 *      Author: bsoper
 */

#include "board.hpp"
#include "load_switch_rail.hpp"

#ifndef IS_DEBUG_BOARD
void LoadSwitch::set_load_switch_navcmp(bool state) {
	Chip_GPIO_WritePortBit(LPC_GPIO, NAVCMP_EN_PORT, NAVCMP_EN_PIN, state);
}

void LoadSwitch::set_load_switch_fltctl(bool state) {
	Chip_GPIO_WritePortBit(LPC_GPIO, FLTCTL_EN_PORT, FLTCTL_EN_PIN, state);
}

void LoadSwitch::set_load_switch_gps(bool state) {
	Chip_GPIO_WritePortBit(LPC_GPIO, GPS_EN_PORT, GPS_EN_PIN, state);
}

void LoadSwitch::set_load_switch_radio(bool state) {
	Chip_GPIO_WritePortBit(LPC_GPIO, RADIO_EN_PORT, RADIO_EN_PIN, state);
}
#endif

