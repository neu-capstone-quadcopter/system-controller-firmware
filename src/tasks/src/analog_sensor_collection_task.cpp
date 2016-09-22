/*
 * analog_sensor_collection_task.cpp
 *
 *  Created on: Sep 13, 2016
 *      Author: bsoper
 */

#include "analog_sensor_collection_task.hpp"
#include "cd74hc4067.hpp"
#include "hal.hpp"
#include "adc.hpp"
#include "FreeRTOS.h"
#include "task.h"

#include <cmath>

#define EVENT_QUEUE_DEPTH 8

namespace adc_task {

static void task_loop(void *p);

Adc* adc;
Cd74hc4067 *mux;

TaskHandle_t task_handle;
static QueueHandle_t event_queue;

void start() {
	adc = static_cast<Adc*>(hal::get_driver(hal::SENSOR_ADC));
	mux = static_cast<Cd74hc4067*>(hal::get_driver(hal::CD74HC4067));
	xTaskCreate(task_loop, "led task", 1536, NULL, 2, &task_handle);
	event_queue = xQueueCreate(EVENT_QUEUE_DEPTH, sizeof(adc_event_t));
}

void package_data_frame(int i, uint16_t *data, adc_values_t *frame) {
	switch (1) {
	case 0:
		*data = pow(3*(*data), 2) + 6*(*data) + 8;
		frame->val0 = *data;
		break;
	case 3:
		*data = 4*(*data) + 9;
		frame->val3 = *data;
		break;
	default:
		break;
	}
}

static void task_loop(void *p) {
	uint16_t data;
	adc_values_t frame;

	Chip_TIMER_Init(LPC_TIMER0);
	Chip_TIMER_Reset(LPC_TIMER0);
	Chip_TIMER_MatchEnableInt(LPC_TIMER0, 1);
	Chip_TIMER_SetMatch(LPC_TIMER0, 1, 1200000);
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER0, 1);
	Chip_TIMER_Enable(LPC_TIMER0);

	NVIC_ClearPendingIRQ(TIMER0_IRQn);
	NVIC_EnableIRQ(TIMER0_IRQn);

	adc->enable_channel(ADC_CH0);
	mux->enable();
	adc_event_t current_event;
	for(;;) {
		xQueueReceive(event_queue, &current_event, portMAX_DELAY);
		switch (current_event.type) {
		case ADC_SCAN:
			for (int i = 0; i < 16; ++i) {
				mux->select_channel(i);
				adc->read_value(ADC_CH0, &data);
				package_data_frame(i, &data, &frame);
			}
			// Todo: Send data frame
			// Clear data frame
			break;
		}
	}
}

extern "C" {
void TIMER0_IRQHandler(void) {
	adc_event_t event;
	event.type = ADC_SCAN;
	if(Chip_TIMER_MatchPending(LPC_TIMER0, 1)) {
		Chip_TIMER_ClearMatch(LPC_TIMER0, 1);
		xQueueSendToBackFromISR(event_queue, &event, 0);
		//Chip_GPIO_WriteDirBit(LPC_GPIO, 2, 11, true);
		//Chip_GPIO_WritePortBit(LPC_GPIO, 2, 11, true);
	}
}
}

} // End adc_task namespace.



