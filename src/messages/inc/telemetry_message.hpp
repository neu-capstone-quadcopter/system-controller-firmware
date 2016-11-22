/*
 * blackbox_sensor_message.hpp
 *
 *  Created on: Nov 17, 2016
 *      Author: Nate
 */

#ifndef MESSAGES_INC_TELEMETRY_MESSAGE_HPP_
#define MESSAGES_INC_TELEMETRY_MESSAGE_HPP_

#include "api.pb.h"

#include "message_base_types.hpp"

typedef int32_t TelemValue;

struct TelemetryMessage : OutgoingNavComputerMessage {
	TelemValue telem_values[14];

	void serialize_protobuf(monarcpb_SysCtrlToNavCPU &protobuf);
};

#endif /* MESSAGES_INC_TELEMETRY_MESSAGE_HPP_ */
