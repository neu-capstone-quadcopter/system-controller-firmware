/*
 * telemetry_radio_task.cpp
 *
 *  Created on: Sep 12, 2016
 *      Author: justin
 */

#include "telemetry_radio_task.hpp"

#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"

#include "hal.hpp"
#include "cc1120.hpp"

#define RX_STATE_TIME 500
#define TX_STATE_TIME 500
#define TX_QUEUE_DEPTH 10
#define EVENT_QUEUE_DEPTH 10

namespace telemetry_radio_task {
	typedef enum {
		IDLE,
		TRANSMIT,
		RECEIVE
	} TaskState;

	typedef enum {
		TRANSMIT_SLOT_START,
	} Event;


	static void task_loop(void *p);
	static void enter_rx_state();
	static void enter_tx_state();
	static void state_timer_handler(TimerHandle_t timer);
	static void cc1120_gpio3_int_handler(void);
	static void cc1120_gpio2_int_handler(void);
	static void cc1120_gpio0_int_handler(void);


	static TaskHandle_t task_handle;
	static QueueHandle_t event_queue;
	static QueueHandle_t tx_queue_0;
	static QueueHandle_t tx_queue_1;
	static QueueHandle_t* current_tx_queue;
	static TimerHandle_t state_timer;
	static TaskState state;

	Cc1120 *telem_cc1120;

	void start(void) {
		// Retrieve driver instances from HAL
		telem_cc1120 = static_cast<Cc1120*>(hal::get_driver(hal::TELEM_CC1120));
		xTaskCreate(task_loop, "telem radio task", 1536, NULL, 2, &task_handle);

		tx_queue_0 = xQueueCreate(TX_QUEUE_DEPTH, sizeof(Message*));
		tx_queue_1 = xQueueCreate(TX_QUEUE_DEPTH, sizeof(Message*));
		event_queue = xQueueCreateCountingSemaphore(EVENT_QUEUE_DEPTH, sizeof(Event));
	}

	void queue_outgoing_message(Message* message) {
		xQueueSendToBack(*current_tx_queue, message, 0);
	}

	static void task_loop(void *p) {
		state_timer = xTimerCreate("State Timer", 500, pdFALSE, NULL, state_timer_handler);
		telem_cc1120->init_device();
		telem_cc1120->gpio3_set_interrupt_pin_handler(cc1120_gpio3_int_handler);
		telem_cc1120->gpio2_set_interrupt_pin_handler(cc1120_gpio2_int_handler);
		telem_cc1120->gpio0_set_interrupt_pin_handler(cc1120_gpio0_int_handler);
		telem_cc1120->access_command_strobe(SIDLE);

		state = IDLE;

		Event current_event;

		for (;;) {
			xQueueReceive(event_queue, &current_event, portMAX_DELAY);
			switch (current_event) {
			case TRANSMIT_SLOT_START:
				// extract message contents
				// serialize data
				// trigger async send
				break;
			default:
				configASSERT(0);
				break;
			}
		}
	}

	static void enter_rx_state() {
		state = RECEIVE;
		telem_cc1120->access_command_strobe_async(SRX, [](){});

		BaseType_t pass;
		pass = xTimerChangePeriod(state_timer, RX_STATE_TIME, 0);
		pass &= xTimerStart(state_timer, 0);

		if (!pass) {
			configASSERT(0);
		}
	}

	static void enter_tx_state() {
		state = TRANSMIT;
		//telem_cc1120->access_command_strobe_async(STX, [](){});

		BaseType_t pass;
		pass = xTimerChangePeriod(state_timer, TX_STATE_TIME, 0);
		pass &= xTimerStart(state_timer, 0);

		if (!pass) {
			configASSERT(0);
		}

		Event e = TRANSMIT_SLOT_START;
		xQueueSendToBack(event_queue, &e, 0);

		taskENTER_CRITICAL();
		if (current_tx_queue == &tx_queue_0) {
			current_tx_queue = &tx_queue_1;
		} else {
			current_tx_queue = &tx_queue_0;
		}
		taskEXIT_CRITICAL();
	}

	static void state_timer_handler(TimerHandle_t timer) {
		switch (state) {
		case RECEIVE:
			enter_tx_state();
			break;
		case TRANSMIT:
			enter_rx_state();
			break;
		default:
			configASSERT(0);
			break;
		}
	}

	static void cc1120_gpio3_int_handler(void) {
		switch (state) {
		case RECEIVE:
			// code
			break;
		case TRANSMIT:
			// code
			break;
		default:
			configASSERT(0);
			break;
		}
	}

	static void cc1120_gpio2_int_handler(void) {
		switch (state) {
		case RECEIVE:
			// code
			break;
		case TRANSMIT:
			// code
			break;
		default:
			configASSERT(0);
			break;
		}
	}

	static void cc1120_gpio0_int_handler(void) {
		switch (state) {
		case RECEIVE:
			// code
			break;
		case TRANSMIT:
			// code
			break;
		default:
			configASSERT(0);
			break;
		}
	}
}
