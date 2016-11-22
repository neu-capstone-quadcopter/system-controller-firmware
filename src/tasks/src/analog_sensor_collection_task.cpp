/*
 * analog_sensor_collection_task.cpp
 *
 *  Created on: Sep 13, 2016
 *      Author: bsoper
 */

// Standard
#include <cmath>

// Freertos
#include "FreeRTOS.h"
#include "task.h"

// Tasks
#include "analog_sensor_collection_task.hpp"
#include "nav_computer_task.hpp"

// Drivers
#include "adc.hpp"
#include "cd74hc4067.hpp"
#include "hal.hpp"

// Other
#include "analog_sensor_message.hpp"

#define EVENT_QUEUE_DEPTH 8

namespace sensor_task {
	enum class AdcEvent {
		ADC_SCAN = 0
	};

	static void initialize_timer();
	static void task_loop(void *p);

	const uint8_t MUX_INPUT_POLYNOMIAL_A_VALUES[16] = {
			1, //SYS_3V3_ISENSE
			1, //SYS_5V_ISENSE
			1, //VGPS_ISENSE
			1, //VUSB_ISENSE
			1, //VFLTCTL_ISENSE
			1, //NAVCMP_ISENSE
			1, //VRADIO_ISENSE
			1, //SPARE
			1, //VRADIO_VSENSE
			1, //NAVCMP_VSENSE
			1, //VUSB_VSENSE
			1, //VFLTCTL_VSENSE
			1, //VGPS_VSENSE
			1, //SYS_5V_VSENSE
			1, //VSYS_VSENSE
			1, //SYS_3V3_VSENSE
	};

	const uint8_t MUX_INPUT_POLYNOMIAL_B_VALUES[16] = {
			1, //SYS_3V3_ISENSE
			1, //SYS_5V_ISENSE
			1, //VGPS_ISENSE
			1, //VUSB_ISENSE
			1, //VFLTCTL_ISENSE
			1, //NAVCMP_ISENSE
			1, //VRADIO_ISENSE
			1, //SPARE
			1, //VRADIO_VSENSE
			1, //NAVCMP_VSENSE
			1, //VUSB_VSENSE
			1, //VFLTCTL_VSENSE
			1, //VGPS_VSENSE
			1, //SYS_5V_VSENSE
			1, //VSYS_VSENSE
			1, //SYS_3V3_VSENSE
	};

	Adc *adc;
	Cd74hc4067 *mux;

	static TaskHandle_t task_handle;
	static QueueHandle_t event_queue;

	void start() {
		adc = hal::get_driver<Adc>(hal::SENSOR_ADC);
		mux = hal::get_driver<Cd74hc4067>(hal::CD74HC4067);
		xTaskCreate(task_loop, "sensor task", 400, NULL, 2, &task_handle);
		event_queue = xQueueCreate(EVENT_QUEUE_DEPTH, sizeof(AdcEvent));
	}

	void initialize_timer() {
		Chip_TIMER_Init(LPC_TIMER1);
		Chip_TIMER_Reset(LPC_TIMER1);
		Chip_TIMER_MatchEnableInt(LPC_TIMER1, 1);
		Chip_TIMER_SetMatch(LPC_TIMER1, 1, 240000);//1200000
		Chip_TIMER_ResetOnMatchEnable(LPC_TIMER1, 1);
		Chip_TIMER_Enable(LPC_TIMER1);
		NVIC_ClearPendingIRQ(TIMER1_IRQn);
		NVIC_EnableIRQ(TIMER1_IRQn);
	}

	void task_loop(void *p) {
		initialize_timer();
		adc->enable_channel(ADC_CH0);
		mux->enable();
		AdcEvent current_event;
		for(;;) {
			xQueueReceive(event_queue, &current_event, portMAX_DELAY);
			switch (current_event) {
			case AdcEvent::ADC_SCAN:
			{
				AnalogSensorMessage msg_to_send;
				for (int i = 0; i < 16; ++i) {
					uint16_t adc_data;
					mux->select_channel(i);
					adc->read_value(ADC_CH0, &adc_data);
					AdcSensorValue val = MUX_INPUT_POLYNOMIAL_A_VALUES[i] * adc_data + MUX_INPUT_POLYNOMIAL_B_VALUES[i];
					msg_to_send.adc_values[i] = val;
				}
				nav_computer_task::add_message_to_outgoing_frame(msg_to_send);
				break;
			}
			}
		}
	}

	extern "C" {
		void TIMER1_IRQHandler(void) {
			if(Chip_TIMER_MatchPending(LPC_TIMER1, 1)) {
				AdcEvent event = AdcEvent::ADC_SCAN;
				Chip_TIMER_ClearMatch(LPC_TIMER1, 1);
				xQueueSendToBackFromISR(event_queue, &event, 0);
			}
		}
	}

}



