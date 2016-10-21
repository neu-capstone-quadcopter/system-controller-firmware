/*
 * analog_sensor_message.cpp
 *
 *  Created on: Oct 21, 2016
 *      Author: nigil
 */

#include "analog_sensor_message.hpp"

AnalogSensorMessage::serialize_protobuf(monarcpb_SysCtrlToNavCPU &protobuf) {
	message.has_analog_sensors = true;
	for(uint8_t i; i < 16; i++) { // TODO: I dont like this, Use an array to map ADC to value it reps
		switch (i) {
		case 0:
			message.analog_sensors.has_sys_3v3_isense = true;
			message.analog_sensors.sys_3v3_isense = data.sensor_values[i];
			break;
		case 1:
			message.analog_sensors.has_sys_3v3_isense = true;
			message.analog_sensors.sys_5v_isense = data.sensor_values[i];
			break;
		case 2:
			message.analog_sensors.has_vgps_isense = true;
			message.analog_sensors.vgps_isense = data.sensor_values[i];
			break;
		case 3:
			message.analog_sensors.has_vusb_isense = true;
			message.analog_sensors.vusb_isense = data.sensor_values[i];
			break;
		case 4:
			message.analog_sensors.has_vfltctl_isense = true;
			message.analog_sensors.vfltctl_isense = data.sensor_values[i];
			break;
		case 5:
			message.analog_sensors.has_navcmp_isense = true;
			message.analog_sensors.navcmp_isense = data.sensor_values[i];
			break;
		case 6:
			message.analog_sensors.has_vradio_isense = true;
			message.analog_sensors.vradio_isense = data.sensor_values[i];
			break;
		case 7:
			message.analog_sensors.has_tp27 = true;
			message.analog_sensors.tp27 = data.sensor_values[i];
			break;
		case 8:
			message.analog_sensors.has_vradio_vsense = true;
			message.analog_sensors.vradio_vsense = data.sensor_values[i];
			break;
		case 9:
			message.analog_sensors.has_navcmp_vsense = true;
			message.analog_sensors.navcmp_vsense = data.sensor_values[i];
			break;
		case 10:
			message.analog_sensors.has_vusb_vsense = true;
			message.analog_sensors.vusb_vsense = data.sensor_values[i];
			break;
		case 11:
			message.analog_sensors.has_vfltctl_vsense = true;
			message.analog_sensors.vfltctl_vsense = data.sensor_values[i];
			break;
		case 12:
			message.analog_sensors.has_vgps_vsense = true;
			message.analog_sensors.vgps_vsense = data.sensor_values[i];
			break;
		case 13:
			message.analog_sensors.has_sys_5v_vsense = true;
			message.analog_sensors.sys_5v_vsense = data.sensor_values[i];
			break;
		case 14:
			message.analog_sensors.has_vsys_vsense = true;
			message.analog_sensors.vsys_vsense = data.sensor_values[i];
			break;
		case 15:
			message.analog_sensors.has_sys_3v3_vsense = true;
			message.analog_sensors.sys_3v3_vsense = data.sensor_values[i];
			break;
		}
	}
}
