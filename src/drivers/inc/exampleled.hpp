/*
 * exampleled.hpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nigil
 */

#ifndef DRIVERS_INC_EXAMPLELED_HPP_
#define DRIVERS_INC_EXAMPLELED_HPP_

#include <cstdint>
#include "driver.hpp"

class ExampleLed : public Driver {
public:
	ExampleLed(uint8_t port, uint8_t pin);
	void init_driver(void);
	void set_led(bool state);
private:
	uint8_t port;
	uint8_t pin;
};



#endif /* DRIVERS_INC_EXAMPLELED_HPP_ */
