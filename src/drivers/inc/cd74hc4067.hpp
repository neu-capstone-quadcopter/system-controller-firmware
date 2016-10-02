/*
 * mux.hpp
 *
 *  Created on: Sep 13, 2016
 *      Author: bsoper
 */

#ifndef DRIVERS_INC_CD74HC4067_HPP_
#define DRIVERS_INC_CD74HC4067_HPP_

#include <cstdint>
#include "driver.hpp"

typedef struct Cd74hc4067_gpio_map {
	uint8_t s0_port;
	uint8_t s0_pin;
	uint8_t s1_port;
	uint8_t s1_pin;
	uint8_t s2_port;
	uint8_t s2_pin;
	uint8_t s3_port;
	uint8_t s3_pin;
	uint8_t en_port;
	uint8_t en_pin;
} Cd74hc4067_gpio_map;

class Cd74hc4067 : public Driver {
public:
	Cd74hc4067(Cd74hc4067_gpio_map gpio_map);
	void init_driver(void);
	void enable();
	void disable();
	void select_channel(uint8_t channel);
	uint8_t get_channel();
private:
	Cd74hc4067_gpio_map gpio_map;
	uint8_t channel;
};



#endif /* DRIVERS_INC_CD74HC4067_HPP_ */
