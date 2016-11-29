#ifndef UTIL_TELEMETRY_PARSER_HPP_
#define UTIL_TELEMETRY_PARSER_HPP_

#include <stream.hpp>
#include "FreeRTOS.h"
#include "telemetry_message.hpp"
#include "nav_computer_task.hpp"

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
	GYR_Z,
	BARO_BP,
	BARO_AP,
	PITCH,
	ROLL,
	YAW,
	CMD_PITCH,
	CMD_ROLL,
	CMD_YAW,
	CMD_THROTTLE,
	MOTOR0_THRUST,
	MOTOR1_THRUST,
	MOTOR2_THRUST,
	MOTOR3_THRUST,
	FRAME_FIELDS
};

struct telemetry_frame
{
	int16_t telem_fields[FRAME_FIELDS] = {0};

};

class TelemetryParser
{
public:
	//Constructor
	TelemetryParser();

	//Parser Functions
	void parse_stream(Stream&);

	//Identifies Data Field
	void get_data(Stream&, uint8_t);

	//Sends a frame if it is complete, else doesn't and continues parse
	void send_frame();

	//Use to reset our member variables
	void reset_variables();


private:
	bool have_data_id = false;
	int curr_field = -1;
	int data_byte = 0; //Which data byte are we looking at?
	bool xor_next_byte = false;
	bool have_values[FRAME_FIELDS] = {false};

	telemetry_frame curr_frame;
	TelemetryMessage msg_to_send;

};

#endif //UTIL_TELEMETRY_PARSER_HPP_
