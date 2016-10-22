/*
 * analog_sensor_message.cpp
 *
 *  Created on: Oct 21, 2016
 *      Author: nigil
 */

#include "analog_sensor_message.hpp"

void AnalogSensorMessage::serialize_protobuf(monarcpb_SysCtrlToNavCPU &protobuf) {
	protobuf.has_analog_sensors = true;
	for(uint8_t i = 0; i < 16; i++) { // TODO: I dont like this, Use an array to map ADC to value it reps
		switch (i) {
		case 0:
			protobuf.analog_sensors.has_sys_3v3_isense = true;
			protobuf.analog_sensors.sys_3v3_isense = this->adc_values[i];
			break;
		case 1:
			protobuf.analog_sensors.has_sys_3v3_isense = true;
			protobuf.analog_sensors.sys_5v_isense = this->adc_values[i];
			break;
		case 2:
			protobuf.analog_sensors.has_vgps_isense = true;
			protobuf.analog_sensors.vgps_isense = this->adc_values[i];
			break;
		case 3:
			protobuf.analog_sensors.has_vusb_isense = true;
			protobuf.analog_sensors.vusb_isense = this->adc_values[i];
			break;
		case 4:
			protobuf.analog_sensors.has_vfltctl_isense = true;
			protobuf.analog_sensors.vfltctl_isense = this->adc_values[i];
			break;
		case 5:
			protobuf.analog_sensors.has_navcmp_isense = true;
			protobuf.analog_sensors.navcmp_isense = this->adc_values[i];
			break;
		case 6:
			protobuf.analog_sensors.has_vradio_isense = true;
			protobuf.analog_sensors.vradio_isense = this->adc_values[i];
			break;
		case 7:
			protobuf.analog_sensors.has_tp27 = true;
			protobuf.analog_sensors.tp27 = this->adc_values[i];
			break;
		case 8:
			protobuf.analog_sensors.has_vradio_vsense = true;
			protobuf.analog_sensors.vradio_vsense = this->adc_values[i];
			break;
		case 9:
			protobuf.analog_sensors.has_navcmp_vsense = true;
			protobuf.analog_sensors.navcmp_vsense = this->adc_values[i];
			break;
		case 10:
			protobuf.analog_sensors.has_vusb_vsense = true;
			protobuf.analog_sensors.vusb_vsense = this->adc_values[i];
			break;
		case 11:
			protobuf.analog_sensors.has_vfltctl_vsense = true;
			protobuf.analog_sensors.vfltctl_vsense = this->adc_values[i];
			break;
		case 12:
			protobuf.analog_sensors.has_vgps_vsense = true;
			protobuf.analog_sensors.vgps_vsense = this->adc_values[i];
			break;
		case 13:
			protobuf.analog_sensors.has_sys_5v_vsense = true;
			protobuf.analog_sensors.sys_5v_vsense = this->adc_values[i];
			break;
		case 14:
			protobuf.analog_sensors.has_vsys_vsense = true;
			protobuf.analog_sensors.vsys_vsense = this->adc_values[i];
			break;
		case 15:
			protobuf.analog_sensors.has_sys_3v3_vsense = true;
			protobuf.analog_sensors.sys_3v3_vsense = this->adc_values[i];
			break;
		}
	}
}
