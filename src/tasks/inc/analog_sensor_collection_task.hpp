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

namespace adc_task {
	void start(void);

	enum adc_event_type {
		ADC_SCAN = 0
	};

	typedef struct {
	  enum adc_event_type type;
	} adc_event_t;
}

#endif /* TASKS_INC_ANALOG_SENSOR_COLLECTION_TASK_HPP_ */
