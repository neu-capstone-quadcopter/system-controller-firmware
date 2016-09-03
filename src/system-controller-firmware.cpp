/*
 * driver.hpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nigil
 */

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include <cr_section_macros.h>

#include "FreeRTOS.h"
#include "task.h"

#include "hal.hpp"
#include "ledtask.hpp"

inline void* operator new (size_t size) { return pvPortMalloc(size); }
inline void* operator new[] (size_t size) { return pvPortMalloc(size); }

int main(void) {
	hal_init();

	LedTask::start_task();

	vTaskStartScheduler();

    return 0;
}
