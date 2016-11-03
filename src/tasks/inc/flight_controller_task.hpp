/*
 * flight_controller_task.hpp
 *
 *  Created on: Oct 14, 2016
 *      Author: ncasale
 */

#ifndef TASKS_INC_FLIGHT_CONTROLLER_TASK_HPP_
#define TASKS_INC_FLIGHT_CONTROLLER_TASK_HPP_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

namespace flight_controller_task {

	enum flight_cont_event_type {
		BLACKBOX_READ = 0,
		FLIGHT_COMMAND = 1
	};

	typedef struct {
	  enum flight_cont_event_type type;
	  uint8_t* data;
	  uint16_t length;
	} flight_cont_event_t;

	void start(void);
	void add_event_to_queue(flight_cont_event_t event);
	void set_frame_channel_cmd(uint8_t channel, uint16_t value);

} // End flight_controller_task namespace.



#endif /* TASKS_INC_FLIGHT_CONTROLLER_TASK_HPP_ */
