/*
 * internal_flight_ctrl_message.hpp
 *
 *  Created on: Oct 30, 2016
 *      Author: bsoper
 */

#ifndef MESSAGES_INC_INTERNAL_FLIGHT_CTRL_MESSAGE_HPP_
#define MESSAGES_INC_INTERNAL_FLIGHT_CTRL_MESSAGE_HPP_

#include "message_base_types.hpp"

struct InternalFlightCtrlMessage : Message {
	uint16_t roll;
	uint16_t pitch;
	uint16_t yaw;
	uint16_t elevation;
};


#endif /* MESSAGES_INC_INTERNAL_FLIGHT_CTRL_MESSAGE_HPP_ */
