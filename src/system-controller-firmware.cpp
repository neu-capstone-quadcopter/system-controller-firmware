/*
 * system-controller-firmware.cpp
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
#include "analog_sensor_collection_task.hpp"
#include "uart_console_task.hpp"
#include "dma_test_task.hpp"
#include "nav_computer_task.hpp"
#include "flight_controller_task.hpp"
#include "config.hpp"

inline void* operator new (size_t size) { return pvPortMalloc(size); }
inline void* operator new[] (size_t size) { return pvPortMalloc(size); }
inline void operator delete (void *p) { vPortFree(p); }
inline void operator delete[] (void *p) { vPortFree(p); }



int main(void) {
	board::setup_clocking();
	vTraceInitTraceData();
	hal::init();

	// Enable PWM Output
	Chip_GPIO_SetDir(LPC_GPIO, FLTCTL_PWM_EN_PORT, FLTCTL_PWM_EN_PIN, true);
	Chip_GPIO_SetPinState(LPC_GPIO, FLTCTL_PWM_EN_PORT, FLTCTL_PWM_EN_PIN, false);

#ifdef IS_DEBUG_BOARD
	led_task::start();
#endif
	//telemetry_radio_task::start();
	//sensor_task::start();
	console_task::start();
	nav_computer_task::start();
	flight_controller_task::start();

	uiTraceStart();

	vTaskStartScheduler();

    return 0;
}
