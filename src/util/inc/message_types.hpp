/*
 * message_types.hpp
 *
 *  Created on: Oct 15, 2016
 *      Author: justin
 */

#ifndef UTIL_INC_MESSAGE_TYPES_HPP_
#define UTIL_INC_MESSAGE_TYPES_HPP_

struct Message {

};

struct ExampleMessage : Message  {
	uint32_t x;
	const char* y;
};



#endif /* UTIL_INC_MESSAGE_TYPES_HPP_ */
