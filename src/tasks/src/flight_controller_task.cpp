/*
 * flight_controller_task.cpp
 *
 *  Created on: Oct 14, 2016
 *      Author: ncasale
 */

#include <cstdlib>
#include <cstring>
#include <cr_section_macros.h>
#include <gpio.hpp>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "hal.hpp"
#include "gpdma.hpp"
#include "uartio.hpp"
#include "telemetry_parser.hpp"

#include "flight_controller_task.hpp"

#define EVENT_QUEUE_DEPTH 8
#define MAX_BUFFER_SIZE 255
#define ERROR_VAL 999999

namespace flight_controller_task {

enum SBusChannelMapping {
	PITCH_CH = 1,
	ROLL_CH = 0,
	YAW_CH = 3,
	THROTTLE_CH = 2,
	ARMING_CH = 6
};

static const uint16_t SBUS_CHANNEL_MASK = 0x07ff;
static const uint8_t SBUS_CHANNEL_BIT_LEN = 11;
static const uint8_t SBUS_CHANNEL_17 = 16;
static const uint8_t SBUS_CHANNEL_18 = 17;
static const uint8_t SBUS_INTERVAL_MS = 14;
static const uint8_t SBUS_FRAME_LEN = 25;

static const uint8_t STREAM_READ_LEN = 20;

static const uint8_t RC_VALUE_QUEUE_DEPTH = 1;
static const uint16_t HIGH_SWITCH_RC_VALUE = 200;
static const uint16_t LOW_SWITCH_RC_VALUE = 1800;
static const uint16_t RC_VALUE_TIMEOUT_MS = 5000;

struct SBusFrame{
	uint16_t channels[18];
	bool frame_lost;
	bool fail_safe_activated;

	/*
	 * Make a raw data copy of the frame suitable for sending to cleanflight
	 * This function encodes the data in a format ready for sending via S.BUS
	 * S.BUS signal has 16, 11 bit channels along with 2 digital channels. The
	 * data is to be sent in a big endian format while the LPC1759 is little endian
	 * as such after building a byte we need to reverse the bits contained.
	 */
	void serialize(uint8_t* raw_frame) {
		memset(raw_frame, 0, 25);

		raw_frame[0] = 0x0F; // Reversed start byte
		uint8_t byte_idx = 1;
		int8_t start_pos = 0;

		// TODO: Could have just used a packed struct...
		for(uint8_t i = 0; i < 16; i++) {
			uint16_t channel = this->channels[i] & SBUS_CHANNEL_MASK;

			int8_t bits_to_write = 11;
			raw_frame[byte_idx++] |= channel << (start_pos);
			bits_to_write -= 8 - start_pos;

			start_pos = 11 - bits_to_write;
			if(bits_to_write <= 8) {
				raw_frame[byte_idx] |= channel >> (start_pos);
				if(bits_to_write == 8) {
					byte_idx++;
					start_pos = 0;
				}
				else {
					start_pos = bits_to_write;
				}
			}
			else {
				raw_frame[byte_idx++] |= channel >> (start_pos);
				bits_to_write -= 8;
				start_pos = bits_to_write;
				raw_frame[byte_idx] |= channel >> (11 - start_pos);
			}
		}

		if(this->channels[SBUS_CHANNEL_17]) {
			raw_frame[23] |= (1 << 0); // Bit 0 due to reversed nature
		}

		if(this->channels[SBUS_CHANNEL_18]) {
			raw_frame[23] |= (1 << 1); // Bit 1 due to reversed nature
		}
	}
};

static void task_loop(void *p);
static void init_telem_uart();
static void init_sbus_uart();
static void init_sbus_timer(void);
static void telem_uart_read_handler(UartError status, uint8_t *data, uint16_t len);
static void sbus_frame_written_handler(UartError status);

static TaskHandle_t task_handle;
static QueueHandle_t rc_value_queue;
static TimerHandle_t arming_timeout_timer;

static UartIo* telem_uart;
static UartIo* sbus_uart;
static GpdmaManager *dma_man;
static GpioManager *gpio_man;

static SBusFrame last_sbus_frame;
__BSS(RAM2)
static Stream blackbox_stream;

static bool arming_channel_state = false;
static bool kill_state = false;

auto fc_bb_read_del = dlgt::make_delegate(&telem_uart_read_handler);
auto sbus_written_del = dlgt::make_delegate(&sbus_frame_written_handler);

void start() {
	dma_man = hal::get_driver<GpdmaManager>(hal::GPDMA_MAN);
	telem_uart = hal::get_driver<UartIo>(hal::FC_TELEM_UART);
	sbus_uart = hal::get_driver<UartIo>(hal::FC_SBUS_UART);
	gpio_man = hal::get_driver<GpioManager>(hal::GPIOS);

	rc_value_queue = xQueueCreate(EVENT_QUEUE_DEPTH, sizeof(RcValue));
	// Creating timer for disarming if we don't get new attitude updates
	arming_timeout_timer = xTimerCreate("NavTimer", RC_VALUE_TIMEOUT_MS, pdFALSE, NULL, [](TimerHandle_t timer) {
		disarm_controller();
	});

	xTaskCreate(task_loop, "flight controller", 400, NULL, 2, &task_handle);
}

Status pass_rc(RcValue new_value) {
	arm_controller();

	xTimerStart(arming_timeout_timer, 0);

	if(xQueueSendToBack(rc_value_queue, &new_value, 0) != pdPASS) {
		return Status::COMMAND_ALREADY_INQUEUE;
	}
	return Status::SUCCESS;
}

void arm_controller(void) {
	if(!kill_state) {
		arming_channel_state = true;
		gpio_man->set_pwm_output_en(true);
	}
}

void disarm_controller(void) {
	arming_channel_state = false;
}

void kill_controller(void) {
	kill_state = true;
	arming_channel_state = false;
	gpio_man->set_pwm_output_en(false);
}

bool is_controller_armed(void) {
	return arming_channel_state;
}

static void task_loop(void *p) {
	blackbox_stream.allocate();
	init_telem_uart();
	init_sbus_uart();

	// Init SBUS Frame
	for(uint8_t i = 0; i< 18; i++) {
		last_sbus_frame.channels[i] = 1500;
	}

	init_sbus_timer();

	// Kickoff async reads
	telem_uart->read_async(STREAM_READ_LEN, fc_bb_read_del);

	TelemetryParser telem_parser;
	for(;;) {
		telem_parser.parse_stream(blackbox_stream);
	}
}

static void init_telem_uart() {
	telem_uart->allocate_buffers(0,150);
	telem_uart->config_data_mode(UART_LCR_WLEN8, UART_LCR_PARITY_DIS, UART_LCR_SBS_1BIT);
	telem_uart->set_baud_fractional(0xED, 0x1B, 0x0, SYSCTL_CLKDIV_1); // Set baud rate to 115200
}

static void init_sbus_uart() {
	sbus_uart->allocate_buffers(100, 0);
	sbus_uart->set_baud_fractional(0x41, 0xC, 0x0, SYSCTL_CLKDIV_4); // Set baud rate to 100000
	sbus_uart->config_data_mode(UART_LCR_WLEN8, UART_LCR_PARITY_EVEN, UART_LCR_SBS_2BIT);

	GpdmaChannel *sbus_dma_channel_tx = dma_man->allocate_channel(0);
	GpdmaChannel *sbus_dma_channel_rx = dma_man->allocate_channel(1);

	sbus_uart->bind_dma_channels(sbus_dma_channel_tx, sbus_dma_channel_rx);
	sbus_uart->set_transfer_mode(UART_XFER_MODE_DMA);
}

static void init_sbus_timer(void) {
	Chip_RIT_Init(LPC_RITIMER);
	Chip_RIT_TimerDebugEnable(LPC_RITIMER);
	Chip_RIT_SetTimerInterval(LPC_RITIMER, SBUS_INTERVAL_MS);
	Chip_RIT_Enable(LPC_RITIMER);
	NVIC_EnableIRQ(RITIMER_IRQn); // TODO: Make this a very high priority interrupt
}

static void telem_uart_read_handler(UartError status, uint8_t *data, uint16_t len)
{
	BaseType_t task_woken = pdFALSE;

	blackbox_stream.push(data,len, &task_woken);

	telem_uart->read_async(STREAM_READ_LEN, fc_bb_read_del);

	if(task_woken) {
		vPortYield();
	}
}

static void sbus_frame_written_handler(UartError status) {
	// TODO: Check status
}
} // End flight_controller_task namespace.

extern "C" {
	using namespace flight_controller_task;
	void RIT_IRQHandler(void) {
		uint8_t raw_frame[SBUS_FRAME_LEN];
		SBusFrame new_frame = last_sbus_frame;;
		RcValue new_values;
		// Get RC values
		if(xQueueReceiveFromISR(rc_value_queue, &new_values, NULL) == pdTRUE) {
			new_frame.channels[PITCH_CH] = new_values.pitch;
			new_frame.channels[ROLL_CH] = new_values.roll;
			new_frame.channels[YAW_CH] = new_values.yaw;
			new_frame.channels[THROTTLE_CH] = new_values.throttle;
		}

		// Arming
		if(arming_channel_state) {
			new_frame.channels[ARMING_CH] = HIGH_SWITCH_RC_VALUE;
		}
		else {
			new_frame.channels[ARMING_CH] = LOW_SWITCH_RC_VALUE;
		}

		memcpy(&last_sbus_frame, &new_frame, sizeof(SBusFrame));
		new_frame.serialize(raw_frame);
		sbus_uart->write_async(raw_frame, SBUS_FRAME_LEN, sbus_written_del);
		//last_sbus_frame = new_frame;
		Chip_RIT_ClearInt(LPC_RITIMER);
	}
}

