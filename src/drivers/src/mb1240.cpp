/*
 * mb1240.cpp
 *
 *  Created on: Nov 22, 2016
 *      Author: nigil
 */

#include "chip.h"

#include "mb1240.hpp"
#include "board.hpp"
#include "util.hpp"

//static const uint32_t SENSOR_US_PER_CM = 58;
static const float SENSOR_US_PER_MM = 5.8f;
static const uint32_t TIMER_CNT_PER_US = 24; // 1e-6 / (4 / CCLK))

Mb1240::Mb1240(LPC_TIMER_T *ultrasonic_timer, uint8_t timer_cap_ch) {
	this->timer = ultrasonic_timer;
	this->timer_cap_ch = timer_cap_ch;
}

void Mb1240::init_driver() {
	board::mb1240_init();
	this->init_capture_timer();
}

bool Mb1240::get_current_range_mm(uint16_t *range) {
	*range = this->current_range_mm;
	if(this->has_read_last_value) {
		return false;
	}
	else {
		this->has_read_last_value = true;
		return true;
	}
}

void Mb1240::timer_interrupt_handler() {
	if(this->is_awaiting_rising_edge) {
		// Record current capture value
		this->last_rising_cap = this->timer->CR[this->timer_cap_ch];

		Chip_TIMER_CaptureRisingEdgeDisable(this->timer, this->timer_cap_ch);
		Chip_TIMER_CaptureFallingEdgeEnable(this->timer, this->timer_cap_ch);


		this->is_awaiting_rising_edge = false;
	}
	else {
		uint32_t falling_edge_capture = this->timer->CR[this->timer_cap_ch];


		uint32_t pulse_cnt;
		if(falling_edge_capture > this->last_rising_cap) {
			pulse_cnt = falling_edge_capture - this->last_rising_cap;
		}
		else {
			pulse_cnt = UINT32_MAX - this->last_rising_cap + falling_edge_capture;
		}

		uint32_t pulse_time_us = pulse_cnt / TIMER_CNT_PER_US;

		this->current_range_mm = pulse_time_us / SENSOR_US_PER_MM;


		Chip_TIMER_CaptureFallingEdgeDisable(this->timer, this->timer_cap_ch);
		Chip_TIMER_CaptureRisingEdgeEnable(this->timer, this->timer_cap_ch);

		this->has_read_last_value = false;
		this->is_awaiting_rising_edge = true;
	}
	Chip_TIMER_ClearCapture(this->timer, this->timer_cap_ch);
}

void Mb1240::init_capture_timer() {
	Chip_TIMER_Init(this->timer);
	Chip_TIMER_Reset(this->timer);

	Chip_TIMER_CaptureRisingEdgeEnable(this->timer, this->timer_cap_ch);
	Chip_TIMER_CaptureEnableInt(this->timer, this->timer_cap_ch);

	Chip_TIMER_Enable(this->timer);
	NVIC_ClearPendingIRQ(get_timer_nvic_irq(this->timer));
	NVIC_EnableIRQ(get_timer_nvic_irq(this->timer));
}
