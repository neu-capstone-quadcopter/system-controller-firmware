/*
 * cd74hc4067.cpp
 *
 *  Created on: Sep 13, 2016
 *      Author: bsoper
 */

#include "cd74hc4067.hpp"
#include "chip.h"

Cd74hc4067::Cd74hc4067(Cd74hc4067_gpio_map gpio_map){
	this->gpio_map = gpio_map;
}

void Cd74hc4067::init_driver(void) {
	Chip_GPIO_WriteDirBit(LPC_GPIO, this->gpio_map.s0_port, this->gpio_map.s0_pin, true);
	Chip_GPIO_WriteDirBit(LPC_GPIO, this->gpio_map.s1_port, this->gpio_map.s1_pin, true);
	Chip_GPIO_WriteDirBit(LPC_GPIO, this->gpio_map.s2_port, this->gpio_map.s2_pin, true);
	Chip_GPIO_WriteDirBit(LPC_GPIO, this->gpio_map.s3_port, this->gpio_map.s3_pin, true);
	Chip_GPIO_WriteDirBit(LPC_GPIO, this->gpio_map.en_port, this->gpio_map.en_pin, true);
}

void Cd74hc4067::enable() {
	Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.en_port, this->gpio_map.en_pin, false);
}

void Cd74hc4067::disable() {
	Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.en_port, this->gpio_map.en_pin, true);
}

void Cd74hc4067::select_channel(uint8_t channel) {
	this->channel = channel;
	switch(channel) {
	case 0:
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s0_port, this->gpio_map.s0_pin, false);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s1_port, this->gpio_map.s1_pin, false);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s2_port, this->gpio_map.s2_pin, false);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s3_port, this->gpio_map.s3_pin, false);
		break;
	case 1:
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s0_port, this->gpio_map.s0_pin, true);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s1_port, this->gpio_map.s1_pin, false);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s2_port, this->gpio_map.s2_pin, false);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s3_port, this->gpio_map.s3_pin, false);
		break;
	case 2:
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s0_port, this->gpio_map.s0_pin, false);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s1_port, this->gpio_map.s1_pin, true);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s2_port, this->gpio_map.s2_pin, false);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s3_port, this->gpio_map.s3_pin, false);
		break;
	case 3:
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s0_port, this->gpio_map.s0_pin, true);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s1_port, this->gpio_map.s1_pin, true);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s2_port, this->gpio_map.s2_pin, false);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s3_port, this->gpio_map.s3_pin, false);
		break;
	case 4:
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s0_port, this->gpio_map.s0_pin, false);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s1_port, this->gpio_map.s1_pin, false);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s2_port, this->gpio_map.s2_pin, true);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s3_port, this->gpio_map.s3_pin, false);
		break;
	case 5:
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s0_port, this->gpio_map.s0_pin, true);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s1_port, this->gpio_map.s1_pin, false);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s2_port, this->gpio_map.s2_pin, true);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s3_port, this->gpio_map.s3_pin, false);
		break;
	case 6:
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s0_port, this->gpio_map.s0_pin, false);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s1_port, this->gpio_map.s1_pin, true);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s2_port, this->gpio_map.s2_pin, true);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s3_port, this->gpio_map.s3_pin, false);
		break;
	case 7:
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s0_port, this->gpio_map.s0_pin, true);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s1_port, this->gpio_map.s1_pin, true);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s2_port, this->gpio_map.s2_pin, true);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s3_port, this->gpio_map.s3_pin, false);
		break;
	case 8:
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s0_port, this->gpio_map.s0_pin, false);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s1_port, this->gpio_map.s1_pin, false);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s2_port, this->gpio_map.s2_pin, false);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s3_port, this->gpio_map.s3_pin, true);
		break;
	case 9:
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s0_port, this->gpio_map.s0_pin, true);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s1_port, this->gpio_map.s1_pin, false);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s2_port, this->gpio_map.s2_pin, false);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s3_port, this->gpio_map.s3_pin, true);
		break;
	case 10:
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s0_port, this->gpio_map.s0_pin, false);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s1_port, this->gpio_map.s1_pin, true);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s2_port, this->gpio_map.s2_pin, false);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s3_port, this->gpio_map.s3_pin, true);
		break;
	case 11:
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s0_port, this->gpio_map.s0_pin, true);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s1_port, this->gpio_map.s1_pin, true);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s2_port, this->gpio_map.s2_pin, false);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s3_port, this->gpio_map.s3_pin, true);
		break;
	case 12:
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s0_port, this->gpio_map.s0_pin, false);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s1_port, this->gpio_map.s1_pin, false);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s2_port, this->gpio_map.s2_pin, true);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s3_port, this->gpio_map.s3_pin, true);
		break;
	case 13:
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s0_port, this->gpio_map.s0_pin, true);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s1_port, this->gpio_map.s1_pin, false);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s2_port, this->gpio_map.s2_pin, true);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s3_port, this->gpio_map.s3_pin, true);
		break;
	case 14:
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s0_port, this->gpio_map.s0_pin, false);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s1_port, this->gpio_map.s1_pin, true);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s2_port, this->gpio_map.s2_pin, true);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s3_port, this->gpio_map.s3_pin, true);
		break;
	case 15:
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s0_port, this->gpio_map.s0_pin, true);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s1_port, this->gpio_map.s1_pin, true);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s2_port, this->gpio_map.s2_pin, true);
		Chip_GPIO_WritePortBit(LPC_GPIO, this->gpio_map.s3_port, this->gpio_map.s3_pin, true);
		break;
	default:
		break;
	}
}

uint8_t Cd74hc4067::get_channel() {
	return this->channel;
}

