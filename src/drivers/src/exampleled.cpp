/*
 * exampleled.cpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nigil
 */

#include "chip.h"

#include "exampleled.hpp"
#include "board.hpp"

ExampleLed::ExampleLed(uint8_t port, uint8_t pin) {
	this->port = port;
	this->pin = pin;
}

void ExampleLed::init_driver(void) {
	Chip_GPIO_WriteDirBit(LPC_GPIO, this->port, this->pin, true);
}

void ExampleLed::set_led(bool state) {
	Chip_GPIO_WritePortBit(LPC_GPIO, this->port, this->pin, state);
}

