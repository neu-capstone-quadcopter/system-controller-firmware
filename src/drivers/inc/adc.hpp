/*
 * adc.hpp
 *
 *  Created on: Sep 13, 2016
 *      Author: bsoper
 */

#ifndef DRIVERS_INC_ADC_HPP_
#define DRIVERS_INC_ADC_HPP_

//#include "lpc_types.h"
#include "chip.h"
#include "driver.hpp"
#include "FreeRTOS.h"
#include "semphr.h"

class Adc : public Driver {
public:
	Adc(LPC_ADC_T *adc_base);
	void init_driver();
	int read_value(ADC_CHANNEL_T channel, uint16_t *data);
	void enable_channel(ADC_CHANNEL_T channel);
private:
	LPC_ADC_T *adc_base;
	ADC_CLOCK_SETUP_T adc_clock;
	SemaphoreHandle_t command_running_semaphore;
};



#endif /* DRIVERS_INC_ADC_HPP_ */
