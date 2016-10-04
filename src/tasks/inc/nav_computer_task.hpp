/*
 * nav_computer_task.hpp
 *
 *  Created on: Sep 27, 2016
 *      Author: bsoper
 */

#ifndef TASKS_INC_NAV_COMPUTER_TASK_HPP_
#define TASKS_INC_NAV_COMPUTER_TASK_HPP_

#include "analog_sensor_collection_task.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

namespace nav_computer_task {

	static QueueHandle_t nav_event_queue;
	void start(void);

	enum nav_event_type {
		ADC_SCAN = 0
	};

	typedef struct {
	  enum nav_event_type type;
	  sensor_task::adc_values_t data;
	} nav_event_t;

	void add_event_to_queue(nav_event_t event);

} // End nav_computer_task namespace.



#endif /* TASKS_INC_NAV_COMPUTER_TASK_HPP_ */
