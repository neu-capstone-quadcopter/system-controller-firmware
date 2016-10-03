/*
 * hal.cpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nigil
 */

#include <uartio.hpp>
#include "chip.h"

#include "hal.hpp"
#include "gpdma.hpp"
#include "sspio.hpp"
#include "cc1120.hpp"
#include "driver.hpp"
#include "exampleled.hpp"
#include "adc.hpp"
//#include "nav_computer.hpp"
#include "cd74hc4067.hpp"

namespace hal {
	void add_drivers(void);

	static Driver *drivers[NUM_IDENTIFIERS];

	Cd74hc4067_gpio_map gpio_map = {
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

	void init(void) {
		SystemCoreClockUpdate();
		Chip_IOCON_Init(LPC_IOCON);
		Chip_GPIO_Init(LPC_GPIO);
		Chip_IOCON_Init(LPC_IOCON);

		Chip_GPIO_WriteDirBit(LPC_GPIO, 2, 10, true); // Random Debug LED...

		add_drivers();

		// Initialize all drivers
		for(int i = 0; i < NUM_IDENTIFIERS; i++) {
			drivers[i]->init_driver();
		}
	}

	void add_drivers(void) {
		// Instantiate drivers
		GpdmaManager *gpdma_man = new GpdmaManager(LPC_GPDMA);
		SspIo *telem_cc1120_ssp = new SspIo(LPC_SSP1);
		UartIo *console_uart = new UartIo(LPC_UART3);
		ExampleLed *led_0 = new ExampleLed(2, 11);
		ExampleLed *led_1 = new ExampleLed(2, 12);
		Adc *adc = new Adc(LPC_ADC);
		Cd74hc4067 *adc_mux = new Cd74hc4067(gpio_map);
		//NavComputer *nav_computer = new NavComputer();
		Cc1120 *telem_cc1120 = new Cc1120(telem_cc1120_ssp);

		// Add drivers to driver array
		drivers[GPDMA_MAN] = gpdma_man;
		drivers[TELEM_CC1120_SSP] = telem_cc1120_ssp;
		drivers[LED_0] = led_0;
		drivers[LED_1] = led_1;
		drivers[SENSOR_ADC] = adc;
		drivers[CD74HC4067] = adc_mux;
		//drivers[NAV_COMPUTER] = nav_computer;
		drivers[TELEM_CC1120] = telem_cc1120;
		drivers[CONSOLE_UART] = console_uart;
	}

	template <class T>
	T *get_driver(driver_identifier id) {
		return static_cast<T *>(drivers[id]);
	}

	template class GpdmaManager *get_driver(driver_identifier);
	template class SspIo *get_driver(driver_identifier);
	template class UartIo *get_driver(driver_identifier);
	template class Adc *get_driver(driver_identifier);
	template class Cd74hc4067 *get_driver(driver_identifier);
	template class Cc1120 *get_driver(driver_identifier);
	template class ExampleLed *get_driver(driver_identifier);
}

extern "C" {
	using namespace hal;
	void DMA_IRQHandler() {
		static_cast<GpdmaManager*>(drivers[GPDMA_MAN])->interrupt_handler();
	}

	void SSP1_IRQHandler() {
		static_cast<SspIo*>(drivers[TELEM_CC1120_SSP])->ssp_interrupt_handler();
	}

	void UART3_IRQHandler(void){
		static_cast<UartIo*>(drivers[CONSOLE_UART])->uartInterruptHandler();
	}
}
