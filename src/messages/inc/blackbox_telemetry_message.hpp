/*
 * blackbox_sensor_message.hpp
 *
 *  Created on: Nov 17, 2016
 *      Author: Nate
 */

#ifndef MESSAGES_INC_BLACKBOX_TELEMETRY_MESSAGE_HPP_
#define MESSAGES_INC_BLACKBOX_TELEMETRY_MESSAGE_HPP_

#include "api.pb.h"

#include "message_base_types.hpp"

typedef int32_t BBTelemValue;

struct BlackboxTelemetryMessage : OutgoingNavComputerMessage {
	BBTelemValue bb_telem_values[9];

	void serialize_protobuf(monarcpb_SysCtrlToNavCPU &protobuf);
};

#endif /* MESSAGES_INC_BLACKBOX_TELEMETRY_MESSAGE_HPP_ */
