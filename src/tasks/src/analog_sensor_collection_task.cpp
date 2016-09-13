/*
 * analog_sensor_collection_task.cpp
 *
 *  Created on: Sep 13, 2016
 *      Author: bsoper
 */

#include "analog_sensor_collection_task.hpp"
#include "hal.hpp"
#include "adc.hpp"
#include "FreeRTOS.h"
#include "task.h"

namespace adc_task {
	static void task_loop(void *p);

	Adc* adc;

	TaskHandle_t task_handle;

	void start() {
		adc = static_cast<Adc*>(hal::get_driver(hal::SENSOR_ADC));
		xTaskCreate(task_loop, "led task", 1536, NULL, 2, &task_handle);
	}

	static void task_loop(void *p) {
		uint16_t data;
		adc->enable_channel(ADC_CH0);
		adc->read_value(ADC_CH0, &data);
		for(;;) {
			asm("nop:");
		}
	}
}


