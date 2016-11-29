/*
 * analog_sensor_message.cpp
 *
 *  Created on: Oct 21, 2016
 *      Author: nigil
 */

#include <telemetry_message.hpp>

void TelemetryMessage::serialize_protobuf(monarcpb_SysCtrlToNavCPU &protobuf) {
	protobuf.has_telemetry = true;
	protobuf.telemetry.has_attitude = true;
	protobuf.telemetry.has_gyroscope = true;
	protobuf.telemetry.has_magnetometer = true;
	protobuf.telemetry.has_accelerometer = true;
	protobuf.telemetry.has_atmospheric_pressure = true;
	for(uint8_t i = 0; i < 17; i++) {
		switch (i) {
		case 0: //Acc x
			protobuf.telemetry.accelerometer.has_x = true;
			protobuf.telemetry.accelerometer.x = this->telem_values[i];
			break;
		case 1: //Acc y
			protobuf.telemetry.accelerometer.has_y = true;
			protobuf.telemetry.accelerometer.y = this->telem_values[i];
			break;
		case 2: //Acc z
			protobuf.telemetry.accelerometer.has_z = true;
			protobuf.telemetry.accelerometer.z = this->telem_values[i];
			break;
		case 3: //Mag x
			protobuf.telemetry.magnetometer.has_x = true;
			protobuf.telemetry.magnetometer.x = this->telem_values[i];
			break;
		case 4: //Mag y
			protobuf.telemetry.magnetometer.has_y = true;
			protobuf.telemetry.magnetometer.y = this->telem_values[i];
			break;
		case 5: //Mag z
			protobuf.telemetry.magnetometer.has_z = true;
			protobuf.telemetry.magnetometer.z = this->telem_values[i];
			break;
		case 6: //Gyr x
			protobuf.telemetry.gyroscope.has_x = true;
			protobuf.telemetry.gyroscope.x = this->telem_values[i];
			break;
		case 7: //Gyr y
			protobuf.telemetry.gyroscope.has_y = true;
			protobuf.telemetry.gyroscope.y = this->telem_values[i];
			break;
		case 8: //Gyr z
			protobuf.telemetry.gyroscope.has_z = true;
			protobuf.telemetry.gyroscope.z = this->telem_values[i];
			break;
		case 9: //Baro
			protobuf.telemetry.has_atmospheric_pressure = true;
			protobuf.telemetry.atmospheric_pressure = this->telem_values[i];
			break;
		case 10: // Pitch
			protobuf.telemetry.attitude.has_pitch = true;
			protobuf.telemetry.attitude.pitch = this->telem_values[i];
			break;
		case 11: // Roll
			protobuf.telemetry.attitude.has_roll = true;
			protobuf.telemetry.attitude.roll = this->telem_values[i];
			break;
		case 12: // Yaw
			protobuf.telemetry.attitude.has_yaw = true;
			protobuf.telemetry.attitude.yaw = this->telem_values[i];
			break;
		case 13: //Motor0 Thrust
			protobuf.telemetry.has_motor0_thrust = true;
			protobuf.telemetry.motor0_thrust = this->telem_values[i];
			break;
		case 14: //Motor1 Thrust
			protobuf.telemetry.has_motor1_thrust = true;
			protobuf.telemetry.motor1_thrust = this->telem_values[i];
			break;
		case 15: //Motor2 Thrust
			protobuf.telemetry.has_motor2_thrust = true;
			protobuf.telemetry.motor2_thrust = this->telem_values[i];
			break;
		case 16: //Motor3 Thrust
			protobuf.telemetry.has_motor3_thrust = true;
			protobuf.telemetry.motor3_thrust = this->telem_values[i];
			break;
		}
	}
}
