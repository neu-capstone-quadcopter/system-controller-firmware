/*
 * nav_computer_task.hpp
 *
 *  Created on: Sep 27, 2016
 *      Author: bsoper
 */

#ifndef TASKS_INC_NAV_COMPUTER_TASK_HPP_
#define TASKS_INC_NAV_COMPUTER_TASK_HPP_

#include "analog_sensor_collection_task.hpp"
#include "message_base_types.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

namespace nav_computer_task {
	void start(void);

	enum class NavComputerCmdStatus {
		NO_ERROR
	};

	enum nav_event_type {
		ADC_SCAN = 0
	};

	typedef struct {
	  enum nav_event_type type;
	} nav_event_t;

	void add_event_to_queue(nav_event_t event);
	void add_message_to_outgoing_frame(OutgoingNavComputerMessage &msg);

} // End nav_computer_task namespace.



#endif /* TASKS_INC_NAV_COMPUTER_TASK_HPP_ */
