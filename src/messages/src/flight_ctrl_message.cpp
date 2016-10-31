/*
 * flight_ctrl_message.cpp
 *
 *  Created on: Oct 31, 2016
 *      Author: ben
 */

#include <flight_ctrl_message.hpp>

void FlightCtrlMessage::serialize_protobuf(monarcpb_SysCtrlToNavCPU &protobuf) {
	protobuf.control.has_roll = true;
	protobuf.control.has_pitch = true;
	protobuf.control.has_yaw = true;
	protobuf.control.has_elevation = true;

	protobuf.control.roll = roll;
	protobuf.control.pitch = pitch;
	protobuf.control.yaw = yaw;
	protobuf.control.elevation = elevation;
}


