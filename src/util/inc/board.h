/*
 * board.h
 *
 *  Created on: Oct 15, 2016
 *      Author: nigil
 */

#ifndef UTIL_INC_BOARD_H_
#define UTIL_INC_BOARD_H_

// Hardware Abstraction

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
#define GPIO				LPC_GPIO
#define IOCON				LPC_IOCON

Cd74hc4067_gpio_map MUX_GPIO_MAP = {
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


#endif /* UTIL_INC_BOARD_H_ */
