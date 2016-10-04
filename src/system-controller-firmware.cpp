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
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"

#include "board.hpp"
#include "hal.hpp"
#include "ledtask.hpp"
#include "telemetry_radio_task.hpp"
#include <uart_console_task.hpp>

inline void* operator new (size_t size) { return pvPortMalloc(size); }
inline void* operator new[] (size_t size) { return pvPortMalloc(size); }

int main(void) {
	board::setup_clocking();
	hal::init();
	led_task::start();
	//telemetry_radio_task::start();
	console_task::start();
	vTaskStartScheduler();

    return 0;
}
