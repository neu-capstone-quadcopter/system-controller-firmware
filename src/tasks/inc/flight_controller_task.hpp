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
	void start(void);
	void set_frame_channel_cmd(uint8_t channel, uint16_t value);

} // End flight_controller_task namespace.



#endif /* TASKS_INC_FLIGHT_CONTROLLER_TASK_HPP_ */
