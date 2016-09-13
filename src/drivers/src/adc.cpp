/*
 * adc.cpp
 *
 *  Created on: Sep 13, 2016
 *      Author: bsoper
 */

#include "adc.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include "chip.h"

Adc::Adc(LPC_ADC_T *adc_base) {
	this->adc_base = adc_base;
}

void Adc::init_driver() {
	Chip_ADC_Init(this->adc_base, &this->adc_clock);
	this->command_running_semaphore = xSemaphoreCreateBinary();
	if(this->command_running_semaphore) {
		xSemaphoreGive(this->command_running_semaphore);
	}
	// Write 1 to pinsel register bits 21:14
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 23, FUNC1);
}

int Adc::read_value(ADC_CHANNEL_T channel, uint16_t *data) {
	if(xSemaphoreTakeFromISR(this->command_running_semaphore, NULL) == pdTRUE) {
		Chip_ADC_SetBurstCmd(this->adc_base, DISABLE);
		/* Start A/D conversion if not using burst mode */
		Chip_ADC_SetStartMode(this->adc_base, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
		/* Waiting for A/D conversion complete */
		while (Chip_ADC_ReadStatus(this->adc_base, channel, ADC_DR_DONE_STAT) != SET) {}

		int status = 0;
		// Get value.
		status = Chip_ADC_ReadValue(this->adc_base, channel, data);
		if (status == 0)
			return 1;
		// Do data manipulation.
		//data = ConvertData(data, i);
		// Send value to correct place.
		//SendData(data, i);
		xSemaphoreGive(this->command_running_semaphore);
		return 0;
	  }
	  else
	    return 1;
}

void Adc::enable_channel(ADC_CHANNEL_T channel) {
	Chip_ADC_EnableChannel(this->adc_base, channel, ENABLE);
}


