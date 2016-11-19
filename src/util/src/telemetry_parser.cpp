#include "telemetry_parser.hpp"

//Define our Data IDs
#define GPS_ALTITUDE_ID 0x01
#define TEMPERATURE1_ID 0x02
#define RPM_ID 0x03
#define FUEL_LEVEL_ID 0x04
#define TEMPERATURE2_ID 0x05
#define VOLT_ID 0x06
#define ALTITUDE_ID 0x10
#define GPS_SPEED_ID 0x11
#define LONGITUDE_ID 0x12
#define COURSE_ID 0x14

#define ACC_X_ID 0x24
#define ACC_Y_ID 0x25
#define ACC_Z_ID 0x26

#define NUM_MSG_FIELDS 9

TelemetryParser::TelemetryParser()
{
	//Just for testing, let's ensure that Mag/Gyro fields
	//are populated
	have_values[MAG_X] = true;
	have_values[MAG_Y] = true;
	have_values[MAG_Z] = true;
	have_values[GYR_X] = true;
	have_values[GYR_Y] = true;
	have_values[GYR_Z] = true;
}

void TelemetryParser::parseForData(Stream &stream)
{

	//Start by popping a byte from the stream
	uint8_t curr_byte = stream.popFromStream();

	//Compare this byte to our data IDs
	getData(stream, curr_byte);

}

void TelemetryParser::getData(Stream &stream, uint8_t curr_byte)
{
	//If byte is 0x5E, this is field separator and we should
	//reset flag and return
	if(curr_byte == 0x5E)
	{
		have_data_id = false;
		data_byte = 0;
		return;
	}
	//Have we figured out which field to populate
	if(!have_data_id)
	{
		//If not, let's figure out which field
		switch(curr_byte)
		{
		case ACC_X_ID:
			curr_field = ACC_X;
			have_data_id = true;
			break;
		case ACC_Y_ID:
			curr_field = ACC_Y;
			have_data_id = true;
			break;
		case ACC_Z_ID:
			curr_field = ACC_Z;
			have_data_id = true;
			break;
		default:
			//configASSERT(0);
			break;
		}

		return;
	}
	else
	{
		//If we already know which field we are populating let's
		//populate it
		uint16_t curr_byte16 = (uint16_t) curr_byte;
		curr_frame.telem_fields[curr_field] = curr_frame.telem_fields[curr_field] +
				(curr_byte16 << (data_byte * 8));
		data_byte++;
	}
}

void TelemetryParser::sendFrame()
{
	//Iterate through our haveValues vector. If all are true, frame is populated
	//and we can send off to Nav Computer. Else just return.
	for(int i = 0; i < GYR_Z; i++)
	{
		if(!have_values[i])
		{
			return;
		}
	}

	//If we make it through loop - frame is ready to send so

	//TODO: LOGIC TO SEND FRAME
	 //Let's take this completed frame and send it off to the Nav Computer
	TelemValue msg_val;
	for(int i = 0; i < NUM_MSG_FIELDS; i++)
	{
		switch(i)
		{
		case 0:
			msg_val = curr_frame.telem_fields[i];
			msg_to_send.telem_values[i] = msg_val;
			break;
		case 1:
			msg_val = curr_frame.telem_fields[i];
			msg_to_send.telem_values[i] = msg_val;
			break;
		case 2:
			msg_val = curr_frame.telem_fields[i];
			msg_to_send.telem_values[i] = msg_val;
			break;
		case 3:
			msg_val = curr_frame.telem_fields[i];
			msg_to_send.telem_values[i] = msg_val;
			break;
		case 4:
			msg_val = curr_frame.telem_fields[i];
			msg_to_send.telem_values[i] = msg_val;
			break;
		case 5:
			msg_val = curr_frame.telem_fields[i];
			msg_to_send.telem_values[i] = msg_val;
			break;
		case 6:
			msg_val = curr_frame.telem_fields[i];
			msg_to_send.telem_values[i] = msg_val;
			break;
		case 7:
			msg_val = curr_frame.telem_fields[i];
			msg_to_send.telem_values[i] = msg_val;
			break;
		case 8:
			msg_val = curr_frame.telem_fields[i];
			msg_to_send.telem_values[i] = msg_val;
			break;

		}

	}

	//Reset member variables for new frame
	resetVariables();

}

void TelemetryParser::resetVariables()
{
	have_data_id = false;
	curr_field = 0;
	data_byte = 0;

	//Reset haveValues and frame data
	for(int i = 0; i < GYR_Z; i++)
	{
		//TODO: Remove condition after testing
		if(i == ACC_X || i == ACC_Y || i == ACC_Z)
		{
			have_values[i] = false;
			curr_frame.telem_fields[i] = 0;
		}

	}
}
