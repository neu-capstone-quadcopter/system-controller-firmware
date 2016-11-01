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
					decodeFrame(bb_stream, 'P');
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
	bool curr_sign;

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

		curr_sign = field_signs[curr_field];
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
			predictZero(curr_sign);
			break;
		case 1:
			predictLastValue(curr_sign);
			break;
		case 2:
			predictStraightLine(curr_sign);
			break;
		case 3:
			predictAverage2(curr_sign);
			break;
		case 4:
			predictMinThrottle(curr_sign);
			break;
		case 5:
			predictMotor0(curr_sign);
			break;
		case 6:
			predictIncrement(curr_sign);
			break;
		default:
			configASSERT(0);
		}

		//Is this the final value we will be using?
		if(final_value)
		{
			//Add current value to particular field
			if(curr_sign == 0)
				curr_frame.addUnsignedField(curr_unsigned_field,curr_value);
			else
				curr_frame.addSignedField(curr_signed_field,scurr_value);

			//Increment current field
			if((curr_signed_field + curr_unsigned_field) < (NUM_FIELDS - 1))
				if(curr_sign == 0)
					curr_unsigned_field++;
				else
					curr_signed_field++;
			else
			{
				frame_complete = true;
				curr_unsigned_field = 0;
				curr_signed_field = 0;
				curr_value = 0;
				scurr_value = 0;
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
		scurr_value = byte;
		curr_value = byte;
		return;
	}
	while(byte & (1<<8) && !bb_stream.streamIsEmpty())
	{
		if(field_signs[curr_field] == 0)
			curr_value += (byte & 0x7F) << (num_bytes_used * 8);
		else
			scurr_value += (byte * 0x7F) << (num_bytes_used * 8);
		num_bytes_used++;
		byte = bb_stream.popFromStream();
	}

	//If this is last byte of data, set curr value and return
	if((byte & (1<<8)) == 0)
	{
		if(field_signs[curr_field] == 0)
			curr_value += (byte & 0x7F) << (num_bytes_used * 8);
		else
			scurr_value += (byte * 0x7F) << (num_bytes_used * 8);
		final_value = true;
		return;
	}

}

void BlackboxParser::decodeSVB(uint8_t byte, uint8_t num_bytes)
{
	//This function works on signed values

	//Start by taking byte and left shifting lsb by num_bytes * 8 - 1
	scurr_value = byte << (num_bytes * 8 - 1);

	//Right shift byte by 1
	byte = byte >> 1;

	//Xor curr_value with byte
	scurr_value = scurr_value ^ byte;

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
		scurr_value += byte;
		scurr_value << 8;
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
	if(tag16_bytes_required == 0)
	{
		uint8_t header_mask = 0x03;
		tag16_bytes_required = header_bytes & header_mask;

		if(tag16_bytes_required == 1 || tag16_bytes_required == 2)
		{
			tag16_bytes_required = 1;
		}
		else if(tag16_bytes_required == 3)
		{
			tag16_bytes_required = 2;
		}

		header_bytes = header_bytes >> 2;
	}

	if(tag16_bytes_decoded < tag16_bytes_required)
	{
		scurr_value += byte;
		tag16_bytes_decoded++;
		if(tag16_bytes_decoded < tag16_bytes_required)
		{
			scurr_value << 8;
		}
		else
		{
			final_value = true;
			tag16_bytes_decoded = 0;
			tag16_bytes_required = 0;
		}
	}

}

void BlackboxParser::decodeTAG8_8SVB(uint8_t byte)
{
	//First an 8 bit header is written. The bits in this field all correspond
	//to a field to be written (e.g. if LSB is 1 then first field will be non-zero).

	//First - check LSB. If 1, then decode this byte and set to current field
	if((header_bytes & (1 << 0)) == 1)
	{
		decodeSVB(byte, 1);
		header_bytes = header_bytes << 1;
		final_value = true;
	}
	else
	{
		scurr_value = 0;
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

/*
 * predictLastValue subtracts the previous value of the field from
 * the raw field value.
 */
void BlackboxParser::predictLastValue(bool sign)
{
	if(sign == 0)
		curr_value += prev_frame.unsigned_field_values[curr_unsigned_field];
	else
		scurr_value += prev_frame.signed_field_values[curr_signed_field];
}

/*
 * predictZero does not modify the field value
 */
void BlackboxParser::predictZero(bool sign)
{
	return;
}

/*
 * predictStraightLine assumes the slope between the current measurement (cur_frame)
 * and the previous measurement(prev_frame) will be similar to the slope between the
 * previous measurement and the measurement before that (prev_frame2)
 */
void BlackboxParser::predictStraightLine(bool sign)
{
	//Signed Field values
	int32_t sprev_val;
	int32_t sprev_val2;

	//Unsigned Field values
	uint32_t uprev_val2;
	uint32_t uprev_val;

	if(prev_frame2.isFrameEmpty())
	{
		if(sign == 0)
			uprev_val2 = 0;
		else
			sprev_val2 = 0;
	}
	else
	{
		if(sign == 0)
			uprev_val2 = prev_frame2.unsigned_field_values[curr_unsigned_field];
		else
			sprev_val2 = prev_frame2.signed_field_values[curr_signed_field];
	}

	if(prev_frame.isFrameEmpty())
	{
		if(sign == 0)
			uprev_val = 0;
		else
			sprev_val = 0;
	}
	else
	{
		if(sign == 0)
			uprev_val = prev_frame.unsigned_field_values[curr_unsigned_field];
		else
			sprev_val = prev_frame.signed_field_values[curr_signed_field];
	}

	//Compute slope and add to curr_value -- check if we are working
	//with signed or unsigned field
	if(sign == 0)
		curr_value += (uprev_val2 - (2 * uprev_val));
	else
		scurr_value += (sprev_val2 - (2 * sprev_val));

}

/*
 * Uses the average of the two previously computed field vaues
 * as the predictor
 */
void BlackboxParser::predictAverage2(bool sign)
{
	//Unsigned field
	uint32_t uprev_val2;
	uint32_t uprev_val;

	//Signed field
	int32_t sprev_val2;
	int32_t sprev_val;

	if(prev_frame2.isFrameEmpty())
	{
		if(sign == 0)
			uprev_val2 = 0;
		else
			sprev_val2 = 0;
	}
	else
	{
		if(sign = 0)
			uprev_val2 = prev_frame2.unsigned_field_values[curr_unsigned_field];
		else
			sprev_val2 = prev_frame2.signed_field_values[curr_signed_field];
	}

	if(prev_frame.isFrameEmpty())
	{
		if(sign == 0)
			uprev_val = 0;
		else
			sprev_val = 0;
	}

	else
	{
		if(sign == 0)
			uprev_val = prev_frame.unsigned_field_values[curr_unsigned_field];
		else
			sprev_val = prev_frame.signed_field_values[curr_signed_field];
	}

	//Compute slope and add to curr_value
	if(sign == 0)
		curr_value += (uprev_val2 - (2 * uprev_val));
	else
		scurr_value += (sprev_val2 - (2 * sprev_val));
}

/*
 * Uses min_throttle value from header as predictor. This value is 1150
 */
void BlackboxParser::predictMinThrottle(bool sign)
{
	if(sign == 0)
		curr_value += min_throttle;
	else
		scurr_value += min_throttle;
}

/*
 * Uses motor0 value which was predicted earlier in decoding as the predictor.
 */
void BlackboxParser::predictMotor0(bool sign)
{
	//This function should never be used with a signed value...
	curr_value += curr_frame.unsigned_field_values[MOTOR0];
	if(sign == 1)
		configASSERT(0);
}

/*
 * Assumes field will be incremented by one.
 */
void BlackboxParser::predictIncrement(bool sign)
{
	if(sign == 0)
		curr_value ++;
	else
		scurr_value++;
}
