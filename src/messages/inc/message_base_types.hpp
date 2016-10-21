/*
 * message_types.hpp
 *
 *  Created on: Oct 20, 2016
 *      Author: nigil
 */

#ifndef UTIL_INC_MESSAGE_TYPES_HPP_
#define UTIL_INC_MESSAGE_TYPES_HPP_

struct Message {

};

struct OutgoingNavComputerMessage : Message {
	virtual void serialize_protobuf(monarcpb_SysCtrlToNavCPU protobuf) = 0;
};


#endif /* UTIL_INC_MESSAGE_TYPES_HPP_ */
