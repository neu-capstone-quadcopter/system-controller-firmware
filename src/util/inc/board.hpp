/*
 * board.hpp
 *
 *  Created on: Sep 16, 2016
 *      Author: nigil
 */

#ifndef UTIL_INC_BOARD_HPP_
#define UTIL_INC_BOARD_HPP_

#include "chip.h"
#include "cd74hc4067.hpp"
#include "config.hpp"

// Hardware Abstraction

#if defined(IS_FLIGHT_PCB)

#define NAV_UART 			LPC_UART1
#define CONSOLE_TASK_UART 	LPC_UART3
#define SSP 				LPC_SSP1
#define ADC 				LPC_ADC
#define GPDMA 				LPC_GPDMA
#define ADC_MUX_PORT		1
#define ADC_MUX_PIN			30

static Cd74hc4067_gpio_map MUX_GPIO_MAP = {
				.s0_port = 0,
				.s0_pin = 29,
				.s1_port = 0,
				.s1_pin = 30,
				.s2_port = 1,
				.s2_pin = 20,
				.s3_port = 1,
				.s3_pin = 22,
				.en_port = 1,
				.en_pin = 23
};

#else

#define NAV_UART 			LPC_UART1
#define CONSOLE_TASK_UART 	LPC_UART3
#define SSP 				LPC_SSP1
#define LED0_PORT 			2
#define LED1_PORT 			2
#define LED0_PIN 			11
#define LED1_PIN 			12
#define DEBUG_LED_PORT 		2
#define DEBUG_LED_PIN		10
#define ADC 				LPC_ADC
#define GPDMA 				LPC_GPDMA
#define ADC_MUX_PORT		0
#define ADC_MUX_PIN			23

static Cd74hc4067_gpio_map MUX_GPIO_MAP = {
				.s0_port = 0,
				.s0_pin = 2,
				.s1_port = 0,
				.s1_pin = 3,
				.s2_port = 0,
				.s2_pin = 21,
				.s3_port = 0,
				.s3_pin = 22,
				.en_port = 0,
				.en_pin = 27
};

#endif

namespace board {
	void setup_clocking(void);
	void ssp_init(LPC_SSP_T *ssp);
	void uart_init(LPC_USART_T *uart);
}


#endif /* UTIL_INC_BOARD_HPP_ */
