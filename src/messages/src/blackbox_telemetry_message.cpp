/*
 * analog_sensor_message.cpp
 *
 *  Created on: Oct 21, 2016
 *      Author: nigil
 */

#include "blackbox_telemetry_message.hpp".hpp"

void BlackboxTelemetryMessage::serialize_protobuf(monarcpb_SysCtrlToNavCPU &protobuf) {
	protobuf.has_telemetry = true;
	protobuf.telemetry.has_gyroscope = true;
	protobuf.telemetry.has_magnetometer = true;
	protobuf.telemetry.has_accelerometer = true;
	for(uint8_t i = 0; i < 9; i++) {
		switch (i) {
		case 0: //Gyro 0
			protobuf.telemetry.gyroscope.has_x = true;
			protobuf.telemetry.gyroscope.x = this->bb_telem_values[i];
			break;
		case 1: //Gyro1
			protobuf.telemetry.gyroscope.has_y = true;
			protobuf.telemetry.gyroscope.y = this->bb_telem_values[i];
			break;
		case 2: //Gyro2
			protobuf.telemetry.gyroscope.has_z = true;
			protobuf.telemetry.gyroscope.z = this->bb_telem_values[i];
			break;
		case 3: //Mag0
			protobuf.telemetry.magnetometer.has_x = true;
			protobuf.telemetry.magnetometer.x = this->bb_telem_values[i];
			break;
		case 4: //Mag1
			protobuf.telemetry.magnetometer.has_y = true;
			protobuf.telemetry.magnetometer.y = this->bb_telem_values[i];
			break;
		case 5: //Mag2
			protobuf.telemetry.magnetometer.has_z = true;
			protobuf.telemetry.magnetometer.z = this->bb_telem_values[i];
			break;
		case 6: //Acc0
			protobuf.telemetry.accelerometer.has_x = true;
			protobuf.telemetry.accelerometer.x = this->bb_telem_values[i];
			break;
		case 7: //Acc1
			protobuf.telemetry.accelerometer.has_y = true;
			protobuf.telemetry.accelerometer.y = this->bb_telem_values[i];
			break;
		case 8: //Acc2
			protobuf.telemetry.accelerometer.has_z = true;
			protobuf.telemetry.accelerometer.z = this->bb_telem_values[i];
			break;
		}
	}
}
