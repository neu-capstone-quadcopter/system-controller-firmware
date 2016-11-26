/*
 * hal.cpp
 *
 *  Created on: Sep 2, 2016
 *      Author: nigil
 */

#include "chip.h"
#include "board.hpp"

#include "hal.hpp"
#include "gpdma.hpp"
#include "uartio.hpp"
#include "sspio.hpp"
#include "cc1120.hpp"
#include "driver.hpp"
#include "exampleled.hpp"
#include "adc.hpp"
#include "cd74hc4067.hpp"
#include "mb1240.hpp"
#include "config.hpp"
#include "load_switch_rail.hpp"

namespace hal {
	void add_drivers(void);

	static Driver *drivers[NUM_IDENTIFIERS];

	void init(void) {
		SystemCoreClockUpdate();
		Chip_IOCON_Init(LPC_IOCON);
		Chip_GPIO_Init(LPC_GPIO);
		add_drivers();

		// Initialize all drivers
		for(int i = 0; i < NUM_IDENTIFIERS; i++) {
			if(i == ULTRASONIC_ALTIMETER)
				continue;
			drivers[i]->init_driver();
		}
	}

	void add_drivers(void) {
		// Instantiate drivers
		GpdmaManager *gpdma_man = new GpdmaManager(GPDMA);
		SspIo *telem_cc1120_ssp = new SspIo(SSP);
		UartIo *console_uart = new UartIo(CONSOLE_TASK_UART);
#ifdef IS_DEBUG_BOARD
		ExampleLed *led_0 = new ExampleLed(LED0_PORT, LED0_PIN);
		ExampleLed *led_1 = new ExampleLed(LED1_PORT, LED1_PIN);
#else
		LoadSwitch *load_switch = new LoadSwitch();
		Mb1240 *ultrasonic_altimeter = new Mb1240(ULTRASONIC_TIMER, ULTRASONIC_TIMER_CAP_CH);
#endif
		Adc *adc = new Adc(ADC);
		Cd74hc4067 *adc_mux = new Cd74hc4067(MUX_GPIO_MAP);
		UartIo *nav_computer = new UartIo(NAV_UART);
		UartIo *fc_blackbox_uart = new UartIo(BLACKBOX_UART);
		UartIo *fc_sbus_uart = new UartIo(SBUS_UART);
		Cc1120 *telem_cc1120 = new Cc1120(telem_cc1120_ssp);

		// Add drivers to driver array
		drivers[GPDMA_MAN] = gpdma_man;
		drivers[TELEM_CC1120_SSP] = telem_cc1120_ssp;
#ifdef IS_DEBUG_BOARD
		drivers[LED_0] = led_0;
		drivers[LED_1] = led_1;
#else
		drivers[LOAD_SWITCH] = load_switch;
		//drivers[ULTRASONIC_ALTIMETER] = ultrasonic_altimeter;
#endif
		drivers[SENSOR_ADC] = adc;
		drivers[CD74HC4067] = adc_mux;
		drivers[NAV_COMPUTER] = nav_computer;
		drivers[FC_TELEM_UART] = fc_blackbox_uart;
		drivers[FC_SBUS_UART] = fc_sbus_uart;
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
	template class Mb1240 *get_driver(driver_identifier);
	template class LoadSwitch *get_driver(driver_identifier);
}

extern "C" {
	using namespace hal;
	void DMA_IRQHandler() {
		static_cast<GpdmaManager*>(drivers[GPDMA_MAN])->interrupt_handler();
	}
#ifdef IS_DEBUG_BOARD
	void SSP1_IRQHandler() {
		static_cast<SspIo*>(drivers[TELEM_CC1120_SSP])->ssp_interrupt_handler();
	}

	void UART3_IRQHandler(void){
		static_cast<UartIo*>(drivers[CONSOLE_UART])->uartInterruptHandler();
	}

	void UART1_IRQHandler(void){
		static_cast<UartIo*>(drivers[NAV_COMPUTER])->uartInterruptHandler();
	}
	void UART2_IRQHandler(void){
		static_cast<UartIo*>(drivers[FC_SBUS_UART])->uartInterruptHandler();
	}

	void UART0_IRQHandler(void){
		static_cast<UartIo*>(drivers[FC_TELEM_UART])->uartInterruptHandler();
	}
#else
	void SSP1_IRQHandler() {
		static_cast<SspIo*>(drivers[TELEM_CC1120_SSP])->ssp_interrupt_handler();
	}

	void UART2_IRQHandler(void){
		static_cast<UartIo*>(drivers[CONSOLE_UART])->uartInterruptHandler();
	}

	void UART1_IRQHandler(void){
		static_cast<UartIo*>(drivers[NAV_COMPUTER])->uartInterruptHandler();
	}

	void UART0_IRQHandler(void){
		static_cast<UartIo*>(drivers[FC_SBUS_UART])->uartInterruptHandler();
	}

	void UART3_IRQHandler(void){
		static_cast<UartIo*>(drivers[FC_TELEM_UART])->uartInterruptHandler();
	}
	void TIMER0_IRQHandler(void) {
		static_cast<Mb1240*>(drivers[ULTRASONIC_ALTIMETER])->timer_interrupt_handler();
	}
#endif
}
