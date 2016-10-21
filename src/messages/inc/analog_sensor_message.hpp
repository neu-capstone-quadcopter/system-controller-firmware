/*
 * AnalogSensorMessage.hpp
 *
 *  Created on: Oct 21, 2016
 *      Author: nigil
 */

#ifndef MESSAGES_INC_ANALOG_SENSOR_MESSAGE_HPP_
#define MESSAGES_INC_ANALOG_SENSOR_MESSAGE_HPP_

#include "api.pb.h"

#include "message_base_types.hpp"

typedef uint16_t AdcSensorValue;

struct AnalogSensorMessage : OutgoingNavComputerMessage {
	AdcSensorValue adc_values[16];

	void serialize_protobuf(monarcpb_SysCtrlToNavCPU &protobuf);
};

#endif /* MESSAGES_INC_ANALOG_SENSOR_MESSAGE_HPP_ */
