/*
 * analog_sensor_collection_task.hpp
 *
 *  Created on: Sep 13, 2016
 *      Author: bsoper
 */

#ifndef TASKS_INC_ANALOG_SENSOR_COLLECTION_TASK_HPP_
#define TASKS_INC_ANALOG_SENSOR_COLLECTION_TASK_HPP_

#include "FreeRTOS.h"
#include "task.h"

namespace sensor_task {
	void start(void);

	enum adc_event_type {
		ADC_SCAN = 0
	};

	typedef struct {
	  enum adc_event_type type;
	} adc_event_t;

	typedef struct {
		uint16_t val0;
		uint16_t val1;
		uint16_t val2;
		uint16_t val3;
		uint16_t val4;
		uint16_t val5;
		uint16_t val6;
		uint16_t val7;
		uint16_t val8;
		uint16_t val9;
		uint16_t val10;
		uint16_t val11;
		uint16_t val12;
		uint16_t val13;
		uint16_t val14;
		uint16_t val15;
	} adc_values_t;
}

#endif /* TASKS_INC_ANALOG_SENSOR_COLLECTION_TASK_HPP_ */
