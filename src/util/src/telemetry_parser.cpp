#include "telemetry_parser.hpp"

//Define our Data IDs
#define ACC_X_ID 0x24
#define ACC_Y_ID 0x25
#define ACC_Z_ID 0x26
#define GYR_X_ID 0x40
#define GYR_Y_ID 0x41
#define GYR_Z_ID 0x42
#define MAG_X_ID 0x6A
#define MAG_Y_ID 0x6B
#define MAG_Z_ID 0x6C
#define BARO_BP_ID 0x10
#define BARO_AP_ID 0x21
#define MOTOR0_THRUST_ID 0x60
#define MOTOR1_THRUST_ID 0x61
#define MOTOR2_THRUST_ID 0x62
#define MOTOR3_THRUST_ID 0x63
#define PITCH_ID 0x7A
#define ROLL_ID 0x7B
#define YAW_ID 0x7C
#define CMD_PITCH_ID 0x8A
#define CMD_ROLL_ID 0x8B
#define CMD_YAW_ID 0x8C
#define CMD_THROTTLE_ID 0x8D

#define NUM_MSG_FIELDS 22

TelemetryParser::TelemetryParser()
{
	//Baro is sent at a lower rate than rest of data so we will
	//assume that it is always ready to send so we don't block other data
	//have_values[BARO] = true;

	/*//JUST FOR TESTING -- For now we will always have the Motor Thrust values be true
	have_values[MOTOR0_THRUST] = true;
	have_values[MOTOR1_THRUST] = true;
	have_values[MOTOR2_THRUST] = true;
	have_values[MOTOR3_THRUST] = true;
*/

}

void TelemetryParser::parse_stream(Stream &stream)
{
	//Start by popping a byte from the stream
	uint8_t curr_byte = stream.pop();

	//Compare this byte to our data IDs
	get_data(stream, curr_byte);

	//Send of the Frame
	send_frame();

}

void TelemetryParser::get_data(Stream &stream, uint8_t curr_byte)
{
	//If byte is 0x5E, this is field separator and we should
	//reset flag and return
	if(curr_byte == 0x5E)
	{
		have_data_id = false;
		data_byte = 0;
		if(curr_field >= 0)
			have_values[curr_field] = true;
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
		case MAG_X_ID:
			curr_field = MAG_X;
			have_data_id = true;
			break;
		case MAG_Y_ID:
			curr_field = MAG_Y;
			have_data_id = true;
			break;
		case MAG_Z_ID:
			curr_field = MAG_Z;
			have_data_id = true;
			break;
		case GYR_X_ID:
			curr_field = GYR_X;
			have_data_id = true;
			break;
		case GYR_Y_ID:
			curr_field = GYR_Y;
			have_data_id = true;
			break;
		case GYR_Z_ID:
			curr_field = GYR_Z;
			have_data_id = true;
			break;
		case BARO_BP_ID:
			curr_field = BARO_BP;
			have_data_id = true;
			break;
		case BARO_AP_ID:
			curr_field = BARO_AP;
			have_data_id = true;
			break;
		case PITCH_ID:
			curr_field = PITCH;
			have_data_id = true;
			break;
		case ROLL_ID:
			curr_field = ROLL;
			have_data_id = true;
			break;
		case YAW_ID:
			curr_field = YAW;
			have_data_id = true;
			break;
		case CMD_PITCH_ID:
			curr_field = CMD_PITCH;
			have_data_id = true;
			break;
		case CMD_ROLL_ID:
			curr_field = CMD_ROLL;
			have_data_id = true;
			break;
		case CMD_YAW_ID:
			curr_field = CMD_YAW;
			have_data_id = true;
			break;
		case CMD_THROTTLE_ID:
			curr_field = CMD_THROTTLE;
			have_data_id = true;
			break;
		case MOTOR0_THRUST_ID:
			curr_field = MOTOR0_THRUST;
			have_data_id = true;
			break;
		case MOTOR1_THRUST_ID:
			curr_field = MOTOR1_THRUST;
			have_data_id = true;
			break;
		case MOTOR2_THRUST_ID:
			curr_field = MOTOR2_THRUST;
			have_data_id = true;
			break;
		case MOTOR3_THRUST_ID:
			curr_field = MOTOR3_THRUST;
			have_data_id = true;
			break;

		default:
			break;
		}

		return;
	}
	else
	{
		//If we receive 0x5D as byte, disregard and XOR next byte with
		//0x60
		if(curr_byte == 0x5D)
		{
			xor_next_byte = true;
			return;
		}
		else{
			//If we already know which field we are populating let's
			//populate it
			uint16_t curr_byte16 = (uint16_t) curr_byte;

			//Should this byte be XOR'd with 0x60?
			if(xor_next_byte)
			{
				curr_byte16 = curr_byte16 ^ 0x60;
				xor_next_byte = false;
			}

			//If the data byte is 0, we will zero out this field to ensure data is correct even
			//if we parse this field multiple times before a frame is sent
			if(data_byte == 0)
				curr_frame.telem_fields[curr_field] = 0;

			curr_frame.telem_fields[curr_field] |= (int16_t)(curr_byte16 << (data_byte * 8));
			data_byte++;
		}
	}
}

void TelemetryParser::send_frame()
{
	//Iterate through our haveValues vector. If all are true, frame is populated
	//and we can send off to Nav Computer. Else just return.
	for(int i = 0; i < FRAME_FIELDS; i++)
	{
		if(!have_values[i])
		{
			return;
		}
	}

	//If we make it through loop - frame is ready to send so

	//Let's take this completed frame and send it off to the Nav Computer
	TelemValue msg_val;
	for(int i = 0; i < NUM_MSG_FIELDS; i++)
	{
		msg_val = curr_frame.telem_fields[i];
		msg_to_send.telem_values[i] = msg_val;

	}

	//Send the frame
	nav_computer_task::add_message_to_outgoing_frame(msg_to_send);

	//Reset member variables for new frame
	reset_variables();

}

void TelemetryParser::reset_variables()
{
	have_data_id = false;
	curr_field = -1;
	data_byte = 0;

	//Reset haveValues and frame data
	for(int i = 0; i < FRAME_FIELDS; i++)
	{
		have_values[i] = false;
		curr_frame.telem_fields[i] = 0;
	}
}
