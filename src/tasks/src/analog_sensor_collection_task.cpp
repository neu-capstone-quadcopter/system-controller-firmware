/*
 * analog_sensor_collection_task.cpp
 *
 *  Created on: Sep 13, 2016
 *      Author: bsoper
 */

#include "analog_sensor_collection_task.hpp"
#include "cd74hc4067.hpp"
#include "hal.hpp"
#include "adc.hpp"
#include "FreeRTOS.h"
#include "task.h"

namespace adc_task {
	static void task_loop(void *p);

	Adc* adc;
	Cd74hc4067 *mux;

	TaskHandle_t task_handle;

	void start() {
		adc = static_cast<Adc*>(hal::get_driver(hal::SENSOR_ADC));
		mux = static_cast<Cd74hc4067*>(hal::get_driver(hal::CD74HC4067));
		xTaskCreate(task_loop, "led task", 1536, NULL, 2, &task_handle);
	}

	static void task_loop(void *p) {
		uint16_t data;

		Chip_TIMER_Init(LPC_TIMER0);
		Chip_TIMER_Reset(LPC_TIMER0);
		Chip_TIMER_MatchEnableInt(LPC_TIMER0, 1);
		Chip_TIMER_SetMatch(LPC_TIMER0, 1, 3125);
		Chip_TIMER_ResetOnMatchEnable(LPC_TIMER0, 1);
		Chip_TIMER_Enable(LPC_TIMER0);

		NVIC_ClearPendingIRQ(TIMER0_IRQn);
		NVIC_EnableIRQ(TIMER0_IRQn);

		adc->enable_channel(ADC_CH0);
		mux->enable();
		for(;;) {
			for (int i = 0; i < 16; ++i) {
				mux->select_channel(i);
				adc->read_value(ADC_CH0, &data);
			}
		}
	}
}

void TIMER0_IRQHandler(void) {
	if(Chip_TIMER_MatchPending(LPC_TIMER0, 1)) {
		Chip_TIMER_ClearMatch(LPC_TIMER0, 1);
		Chip_GPIO_WriteDirBit(LPC_GPIO, 2, 11, true);
		Chip_GPIO_WritePortBit(LPC_GPIO, 2, 11, true);
	}
}


