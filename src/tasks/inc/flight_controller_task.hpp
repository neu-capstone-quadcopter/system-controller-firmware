/*
 * flight_controller_task.hpp
 *
 *  Created on: Oct 14, 2016
 *      Author: ncasale
 */

#ifndef TASKS_INC_FLIGHT_CONTROLLER_TASK_HPP_
#define TASKS_INC_FLIGHT_CONTROLLER_TASK_HPP_

#include <cstdint>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

namespace flight_controller_task {
	enum class Status {
		SUCCESS = 0,
		COMMAND_ALREADY_INQUEUE = 1
	};

	struct RcValue {
		uint16_t pitch;
		uint16_t roll;
		uint16_t yaw;
		uint16_t throttle;
	};

	static bool soft_kill_state = false;

	void start(void);
	Status pass_rc(RcValue new_value);
	void arm_controller(void);
	void disarm_controller(void);
	void kill_controller(void);
	void soft_kill_controller(void);
	bool is_controller_armed(void);

} // End flight_controller_task namespace.



#endif /* TASKS_INC_FLIGHT_CONTROLLER_TASK_HPP_ */
