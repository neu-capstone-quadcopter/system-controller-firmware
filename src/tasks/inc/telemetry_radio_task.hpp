/*
 * telemetry_radio_task.hpp
 *
 *  Created on: Sep 12, 2016
 *      Author: justin
 */

#ifndef TASKS_INC_TELEMETRY_RADIO_TASK_HPP_
#define TASKS_INC_TELEMETRY_RADIO_TASK_HPP_

#include "FreeRTOS.h"
#include "task.h"
#include "message_base_types.hpp"

namespace telemetry_radio_task {
	void start(void);
	void queue_outgoing_message(Message* message);
}


#endif /* TASKS_INC_TELEMETRY_RADIO_TASK_HPP_ */
