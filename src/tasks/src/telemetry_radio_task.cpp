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

using namespace driver;
namespace telemetry_radio_task {
	typedef enum {
		IDLE,
		TRANSMIT,
		TRANSMIT_TX_FIFO_ABOVE_THR,
		RECEIVE,
		RECEIVING_PACKET
	} TaskState;

	typedef enum {
		TRANSMIT_SLOT_START,
		TX_FIFO_FILL,
		MESSAGE_RECEIVED,
	} Event;

	const uint8_t TX_FIFO_THRESHOLD = 50;
	const uint8_t RX_FIFO_THRESHOLD = 50;


	static void task_loop(void *p);
	static void enter_rx_state();
	static void enter_tx_state();
	static void end_tx_state();
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
	static int16_t pkt_count = 0;
	static uint8_t current_tx_pkt_length = 0;
	static uint8_t current_tx_pkt_sent;
	static uint8_t* tx_message_serialized;
	static uint8_t* rx_message_data;
	static uint8_t message_length_received = 0;

	driver::Cc1120 *telem_cc1120;

	void start(void) {
		// Retrieve driver instances from HAL
		telem_cc1120 = hal::get_driver<Cc1120>(hal::TELEM_CC1120);
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
		telem_cc1120->access_command_strobe(CommandStrobeAddress::SIDLE);

		state = IDLE;

		Event current_event;
		Message tx_message;

		for (;;) {
			xQueueReceive(event_queue, &current_event, portMAX_DELAY);
			switch (current_event) {
			case TRANSMIT_SLOT_START:
				current_tx_pkt_sent = 0;
				telem_cc1120->access_command_strobe(driver::CommandStrobeAddress::SFTX);
				// Transmit slot is starting.

				// We've received a message to trasmit
				// Serialize Protobuf
				// serialize_message(&tx_message, tx_message_serialized);
				if (current_tx_pkt_length > 128) {
					telem_cc1120->write_tx_fifo_async(tx_message_serialized, 128,
						[](Cc1120CommandStatus status){
							asm("nop");
						});
					current_tx_pkt_sent = 128;
				} else {
					telem_cc1120->write_tx_fifo_async(tx_message_serialized, current_tx_pkt_length,
						[](Cc1120CommandStatus status){
							asm("nop");
						});
					end_tx_state();
				}
				break;
			default:
				configASSERT(0);
				break;
			}
		}
	}

	static void enter_rx_state() {
		state = RECEIVE;
		telem_cc1120->access_command_strobe_async(CommandStrobeAddress::SRX, [](Cc1120CommandStatus status){
			asm("nop");
		});

		BaseType_t pass;
		pass = xTimerChangePeriod(state_timer, RX_STATE_TIME, 0);
		pass &= xTimerStart(state_timer, 0);

		if (!pass) {
			configASSERT(0);
		}

		board::cc1120_gpio0_set_rising_edge_int(true);
		board::cc1120_gpio0_set_falling_edge_int(false);
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

		board::cc1120_gpio2_set_rising_edge_int(true);
		board::cc1120_gpio2_set_falling_edge_int(false);

		Event e = TRANSMIT_SLOT_START;
		xQueueSendToBack(event_queue, &e, 0);

		/*taskENTER_CRITICAL();
		if (current_tx_queue == &tx_queue_0) {
			current_tx_queue = &tx_queue_1;
		} else {
			current_tx_queue = &tx_queue_0;
		}
		taskEXIT_CRITICAL();*/
	}

	static void end_tx_state() {
		xTimerStop(state_timer, 0);
		enter_rx_state();
	}

	static void state_timer_handler(TimerHandle_t timer) {
		switch (state) {
		case RECEIVE:
			enter_tx_state();
			break;
		case TRANSMIT:
			// TODO: Clean up transmission
			enter_rx_state();
			break;
		default:
			configASSERT(0);
			break;
		}
	}

	static void cc1120_gpio3_int_handler(void) {

	}

	static void cc1120_gpio2_int_handler(void) {
		switch (state) {
		case RECEIVE:
			// code
			break;
		case TRANSMIT:
			state = TRANSMIT_TX_FIFO_ABOVE_THR;
			board::cc1120_gpio2_set_falling_edge_int(true);
			board::cc1120_gpio2_set_rising_edge_int(false);
			break;
		case TRANSMIT_TX_FIFO_ABOVE_THR:
			state = TRANSMIT;
			board::cc1120_gpio2_set_falling_edge_int(false);
			board::cc1120_gpio2_set_rising_edge_int(true);
			if (128 - TX_FIFO_THRESHOLD > current_tx_pkt_length - current_tx_pkt_sent) {
				telem_cc1120->write_tx_fifo_async(tx_message_serialized + current_tx_pkt_sent, current_tx_pkt_length - current_tx_pkt_sent,
						[](Cc1120CommandStatus status){
							asm("nop");
						});
				end_tx_state();
			}
			else {
				telem_cc1120->write_tx_fifo_async(tx_message_serialized + current_tx_pkt_sent, 128 - TX_FIFO_THRESHOLD, [](Cc1120CommandStatus status){
							asm("nop");
						});
				current_tx_pkt_sent += 128 - TX_FIFO_THRESHOLD;
			}
			break;
		default:
			configASSERT(0);
			break;
		}
	}

	static void cc1120_gpio0_int_handler(void) {
		switch (state) {
		case RECEIVE:
			telem_cc1120->read_rx_fifo_async(1, [](Cc1120CommandStatus status, uint8_t* message_length, uint8_t data_len) {
				rx_message_data = new uint8_t [*message_length];
				if (*message_length < 128 - RX_FIFO_THRESHOLD && message_length_received == 0) {
					telem_cc1120->read_rx_fifo_async(*message_length, [](Cc1120CommandStatus status, uint8_t* message_data, uint8_t message_length) {
							memcpy(rx_message_data, message_data, message_length);
							message_length_received = 0;
							Event e = MESSAGE_RECEIVED;
							xQueueSendToBack(event_queue, &e, 0);
						});
				}
				else if (*message_length < 128 - RX_FIFO_THRESHOLD && message_length_received > 0) {
					telem_cc1120->read_rx_fifo_async(*message_length, [](Cc1120CommandStatus status, uint8_t* message_data, uint8_t message_length) {
							memcpy(rx_message_data + message_length_received, message_data, message_length);
							message_length_received = 0;
							Event e = MESSAGE_RECEIVED;
							xQueueSendToBack(event_queue, &e, 0);
						});
				}
				else {
					board::cc1120_gpio0_set_falling_edge_int(true);
					board::cc1120_gpio0_set_rising_edge_int(false);
					telem_cc1120->read_rx_fifo_async(128 - RX_FIFO_THRESHOLD, [](Cc1120CommandStatus stats, uint8_t* message_data, uint8_t message_length) {
						memcpy(rx_message_data + message_length_received, message_data, message_length);
						message_length_received += message_length;
						board::cc1120_gpio0_set_rising_edge_int(true);
						board::cc1120_gpio0_set_falling_edge_int(false);
					});
				}
			});
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
