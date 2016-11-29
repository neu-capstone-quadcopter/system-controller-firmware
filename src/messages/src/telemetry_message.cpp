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
	protobuf.telemetry.has_motor_thrust = true;
	protobuf.telemetry.has_latest_control = true;
	for(uint8_t i = 0; i < 22; i++) {
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
		case 10: //Baro AP
			protobuf.telemetry.has_atmospheric_pressure = true;
			protobuf.telemetry.atmospheric_pressure = this->telem_values[i] + this->telem_values[i-1] * 100;
			break;
		case 11: // Pitch
			protobuf.telemetry.attitude.has_pitch = true;
			protobuf.telemetry.attitude.pitch = this->telem_values[i];
			break;
		case 12: // Roll
			protobuf.telemetry.attitude.has_roll = true;
			protobuf.telemetry.attitude.roll = this->telem_values[i];
			break;
		case 13: // Yaw
			protobuf.telemetry.attitude.has_yaw = true;
			protobuf.telemetry.attitude.yaw = this->telem_values[i];
			break;
		case 14: // CMD Pitch
			protobuf.telemetry.latest_control.has_pitch = true;
			protobuf.telemetry.latest_control.pitch = this->telem_values[i];
			break;
		case 15: // CMD Roll
			protobuf.telemetry.latest_control.has_roll = true;
			protobuf.telemetry.latest_control.roll = this->telem_values[i];
			break;
		case 16: // CMD Yaw
			protobuf.telemetry.latest_control.has_yaw = true;
			protobuf.telemetry.latest_control.yaw = this->telem_values[i];
			break;
		case 17: // CMD Throttle
			protobuf.telemetry.latest_control.has_throttle = true;
			protobuf.telemetry.latest_control.throttle = this->telem_values[i];
			break;
		case 18: //Motor0 Thrust
			protobuf.telemetry.motor_thrust.has_motor0 = true;
			protobuf.telemetry.motor_thrust.motor0 = this->telem_values[i];
			break;
		case 19: //Motor1 Thrust
			protobuf.telemetry.motor_thrust.has_motor1 = true;
			protobuf.telemetry.motor_thrust.motor1 = this->telem_values[i];
			break;
		case 20: //Motor2 Thrust
			protobuf.telemetry.motor_thrust.has_motor2 = true;
			protobuf.telemetry.motor_thrust.motor2 = this->telem_values[i];
			break;
		case 21: //Motor3 Thrust
			protobuf.telemetry.motor_thrust.has_motor3 = true;
			protobuf.telemetry.motor_thrust.motor3 = this->telem_values[i];
			break;
		default:
			break;
		}
	}
}
