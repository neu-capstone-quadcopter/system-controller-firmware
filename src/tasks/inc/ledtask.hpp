/*
 * ledtask.hpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nigil
 */

#ifndef TASKS_INC_LEDTASK_HPP_
#define TASKS_INC_LEDTASK_HPP_

#include "FreeRTOS.h"
#include "task.h"

/*class LedTask {
public:
	LedTask(void);
	void start_task(void);
private:
	void task_loop(void *p);
	TaskHandle_t task_handle;
};
*/
namespace LedTask {
	void start_task(void);
}


#endif /* TASKS_INC_LEDTASK_HPP_ */
