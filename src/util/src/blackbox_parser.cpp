#include "blackbox_parser.hpp"

/*
 * This function will parse our stream until it finds either an
 * \nI or a \nP and pick the correct decoding scheme
 */
void BlackboxParser::decodeFrameType(Stream &bb_stream){

	//Skip this logic if we're in the middle of decoding a frame
	if(decoding_i_frame)
	{
		decodeFrame(bb_stream, 'I');
	}
	else if(decoding_p_frame)
	{
		decodeFrame(bb_stream, 'P');
	}
	else
	{
		//Let's figure out what type of frame we're looking at. The below logic
		//assumes that we won't get a sequence of two newline characters in a row
		//before our frame specifier (e.g. "\n\nI"
		bool found_frame_id = false;
		while(!found_frame_id && !bb_stream.streamIsEmpty())
		{
			uint8_t curr_byte = bb_stream.popFromStream();
			if(curr_byte == '\n')
			{
				uint8_t next_byte = bb_stream.popFromStream();
				if(next_byte == 'I')
				{
					decodeFrame(bb_stream, 'I');
					found_frame_id = true;
				}
				else if(next_byte == 'P')
				{
					decodeFrame(bb_stream, 'I');
					found_frame_id = true;
				}
			}
		}
	}
}

/*
 * This function will decode any I Frames that are contained within our stream
 */
void BlackboxParser::decodeFrame(Stream &bb_stream, char frame_type){
	if(frame_type == 'I')
	{
		//We are now decoding I Frame
		decoding_i_frame = true;
	}
	else if(frame_type == 'P')
	{
		//We are now decoding P Frame
		decoding_p_frame = true;
	}
	else
	{
		//Invalid Frame Type
		configASSERT(0);
	}

	uint8_t curr_predictor;
	uint8_t curr_encoding;

	//Loop through everything in stream
	while(!frame_complete && !bb_stream.streamIsEmpty())
	{
		if(decoding_i_frame)
		{
			curr_predictor = i_field_predictors[curr_field];
			curr_encoding = i_field_encodings[curr_field];
		}
		else if(decoding_p_frame)
		{
			curr_predictor = p_field_predictors[curr_field];
			curr_encoding = p_field_encodings[curr_field];
		}

		uint8_t curr_byte = bb_stream.popFromStream();

		//Let's decode this byte
		switch(curr_encoding)
		{
		case 0:
			decodeUVB(curr_byte, bb_stream);
			break;
		case 1:
			decodeSVB(curr_byte, 1);
			break;
		case 6:
			if(!header_read) //Set header if not already done so
			{
				header_bytes = curr_byte;
				header_read = true;
				if(!bb_stream.streamIsEmpty())
				{
					curr_byte = bb_stream.popFromStream();
				}
				else
				{
					break;
				}
			}
			decodeTAG8_8SVB(curr_byte);
			break;
		case 7:
			if(!header_read)
			{
				header_bytes = curr_byte;
				header_read = true;
				if(!bb_stream.streamIsEmpty())
				{
					curr_byte = bb_stream.popFromStream();
				}
				else
				{
					break;
				}
			}
			decodeTAG2_2S32(curr_byte);
			break;
		case 8:
			decodeTAG8_4S16(curr_byte);
			break;
		case 9:
			decodeNULL(curr_byte);
			break;
		default:
			configASSERT(0);
		}

		//How do we interpret this data now?
		switch(curr_predictor)
		{
		case 0:
			predictZero(curr_byte);
			break;
		case 1:
			predictLastValue(curr_byte);
			break;
		case 2:
			predictStraightLine(curr_byte);
			break;
		case 3:
			predictAverage2(curr_byte);
			break;
		case 4:
			predictMinThrottle(curr_byte);
			break;
		case 5:
			predictMotor0(curr_byte);
			break;
		case 6:
			predictIncrement(curr_byte);
			break;
		default:
			configASSERT(0);
		}

		//Is this the final value we will be using?
		if(final_value)
		{
			//Add current value to particular field
			curr_frame.addField(curr_field,curr_value);

			//Increment current field
			if(curr_field < (NUM_FIELDS - 1))
				curr_field++;
			else
			{
				frame_complete = true;
				curr_field = 0;
				curr_value = 0;
				decoding_i_frame = false;
				decoding_p_frame = false;
				//Add logic here to do something with completed frame
			}

		}
	}

}
/*
 * HELPER FUNCTIONS
 */



/*
 * BEGIN DECODER FUNCTIONS
 */


void BlackboxParser::decodeUVB(uint8_t &byte, Stream &bb_stream)
{
	//7 lower bits are used to encode data
	//MSB is used to signify if another byte is needed
	//to fully represent data
	if((byte & (1<<8)) == 0)
	{
		curr_value = byte;
		return;
	}
	while(byte & (1<<8) && !bb_stream.streamIsEmpty())
	{
		curr_value += (byte & 0x7F) << (num_bytes_used * 8);
		num_bytes_used++;
		byte = bb_stream.popFromStream();
	}

	//If this is last byte of data, set curr value and return
	if((byte & (1<<8)) == 0)
	{
		curr_value += (byte & 0x7F) << (num_bytes_used * 8);
		final_value = true;
		return;
	}

}

void BlackboxParser::decodeSVB(uint8_t byte, uint8_t num_bytes)
{
	//This logic may change if it ends up being used with 32 bit numbers

	//Start by taking byte and left shifting lsb by num_bytes * 8 - 1
	curr_value = byte << (num_bytes * 8 - 1);

	//Right shift byte by 1
	byte = byte >> 1;

	//Xor curr_value with byte
	curr_value = curr_value ^ byte;

}

void BlackboxParser::decodeTAG2_2S32(uint8_t byte)
{
	//Check header to see how many bytes required to encode current field
	if(tag32_bytes_required == 0)
	{
		uint8_t header_mask = 0x03;
		tag32_bytes_required = header_bytes & header_mask;
		header_bytes = header_bytes >> 2;
	}

	if(tag32_bytes_decoded < tag32_bytes_required)
	{
		curr_value += byte;
		curr_value << 8;
		tag32_bytes_decoded++;
	}
	else
	{
		final_value = true;
		tag32_bytes_decoded = 0;
		tag32_bytes_required = 0;
	}



}

void BlackboxParser::decodeTAG8_4S16(uint8_t byte)
{
	//Add code
}

void BlackboxParser::decodeTAG8_8SVB(uint8_t byte)
{
	//First an 8 bit header is written. The bits in this field all correspond
	//to a field to be written (e.g. if LSB is 1 then first field will be non-zero).

	//First - check LSB. If 1, then decode this byte and set to current field
	if((header_bytes & (1 << 0)) == 1)
	{
		decodeSVB(byte, 1);
		header_bytes << 1;
		final_value = true;
	}
	else
	{
		curr_value = 0;
		final_value = true;
	}


}

void BlackboxParser::decodeNULL(uint8_t byte)
{
	curr_value = byte;
	final_value = true;
}


/*
 * BEGIN PREDICTOR FUNCTIONS
 */

void BlackboxParser::predictLastValue(uint8_t byte)
{
	//Add code
}

void BlackboxParser::predictZero(uint8_t byte)
{
	//Add code
}

void BlackboxParser::predictStraightLine(uint8_t byte)
{
	//Add code
}

void BlackboxParser::predictAverage2(uint8_t byte)
{
	//Add code
}

void BlackboxParser::predictMinThrottle(uint8_t byte)
{
	//Add code
}

void BlackboxParser::predictMotor0(uint8_t byte)
{
	//Add code
}

void BlackboxParser::predictIncrement(uint8_t byte)
{
	//Add code
}
