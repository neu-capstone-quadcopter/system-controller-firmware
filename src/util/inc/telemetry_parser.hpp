#ifndef UTIL_TELEMETRY_PARSER_HPP_
#define UTIL_TELEMETRY_PARSER_HPP_

#include "blackbox_stream.hpp"
#include "FreeRTOS.h"
#include "telemetry_message.hpp"

enum frame_fields
{
	ACC_X,
	ACC_Y,
	ACC_Z,
	MAG_X,
	MAG_Y,
	MAG_Z,
	GYR_X,
	GYR_Y,
	GYR_Z
};

struct telemetry_frame
{
	uint16_t telem_fields[GYR_Z+1] = {0};

};

class TelemetryParser
{
public:
	//Constructor
	TelemetryParser();

	//Parser Functions
	void parseForData(Stream&);

	//Identifies Data Field
	void getData(Stream&, uint8_t);

	//Sends a frame if it is complete, else doesn't and continues parse
	void sendFrame();

	//Use to reset our member variables
	void resetVariables();


private:
	bool have_data_id = false;
	int curr_field = 0;
	int data_byte = 0; //Which data byte are we looking at?
	bool have_values[GYR_Z+1] = {false};

	telemetry_frame curr_frame;
	TelemetryMessage msg_to_send;

};

#endif //UTIL_TELEMETRY_PARSER_HPP_


//Checklist:
//1. Outline Basic Class Structure
//2. Create a function that will parse a frame
//3. Create a Frame Structure to hold the data
//4. Create Nav Computer Message Type
//5. Test Messages with Nav Computer
