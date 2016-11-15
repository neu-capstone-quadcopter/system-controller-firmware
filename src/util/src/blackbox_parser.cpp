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
		while(!frame_id_found)
		{
			uint8_t curr_byte = bb_stream.popFromStream();
			if(blackbox_values_index < 300)
			{
				blackbox_values[blackbox_values_index] = curr_byte;
				blackbox_values_index++;

			}
			else
			{
				blackbox_values[blackbox_values_index-1] = 128;
			}


			//uint8_t next_byte = bb_stream.popFromStream();
			if(curr_byte == 'I')
			{
				frame_id_found = true;
				decodeFrame(bb_stream, 'I');

			}

			else if(curr_byte == 'P' && i_frame_decoded)
			{
				frame_id_found = true;
				decodeFrame(bb_stream, 'P');

			}
		}

		return;
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
		i_frame_decoded = false;
        frame_complete = false;
	}
	else if(frame_type == 'P')
	{
		//We are now decoding P Frame
		decoding_p_frame = true;
        frame_complete = false;
	}
	else
	{
		//Invalid Frame Type
		//configASSERT(0);
	}

	uint8_t curr_predictor;
	uint8_t curr_encoding;
	bool curr_sign;

	//Loop through everything in stream
	while(!frame_complete)
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

		//Let's start decoding
		switch(curr_encoding)
		{
		case 0:
            decodeUVB(bb_stream, curr_sign);
			break;
		case 1:
			decodeSVB(bb_stream,curr_sign);
			break;
		case 6:
			if(!header_read) //Set header if not already done so
			{
				header_bytes = bb_stream.popFromStream();
				header_read = true;
			}

			decodeTAG8_8SVB(bb_stream, curr_sign);
			//decoding_tag8_8svb = true;
			break;
		case 7:
			decodeTAG2_3S32(bb_stream, curr_sign);
            decoding_tag2_3s32 = true;
			break;
		case 8:
			decodeTAG8_4S16(bb_stream, curr_sign);
            decoding_tag8_4s16 = true;
			break;
		case 9:
			decodeNULL(bb_stream, curr_sign);
			break;
		default:
			configASSERT(0);
			break;
		}

        //Now that we have decoded value - let's put it into correct field
		if(decoding_tag8_8svb && final_value)
		{
			for(int i = 0; i<tag8_8svb_fields; i++)
			{
				curr_sign = field_signs[curr_field];
				if(curr_sign == 0)
				{
					curr_frame.addUnsignedField(curr_unsigned_field, (uint32_t)tag8_8svb_values[i]);
					curr_unsigned_field++;
				}
				else{
					curr_frame.addSignedField(curr_signed_field, tag8_8svb_values[i]);
					curr_signed_field++;
				}

				curr_field++;
			}

			final_value = false;
			decoding_tag8_8svb = false;
		}
        if(decoding_tag2_3s32 && final_value){
            //If we are doing tag2_3s32 and have decoded all field values, want to place
            //those into frame
            for(int i = 0; i < tag2_3s32_fields; i++){
                curr_sign = field_signs[curr_field];
                if(curr_sign == 0)
                {
                    curr_frame.addUnsignedField(curr_unsigned_field, (uint32_t)tag2_3s32_values[i]);
                    curr_unsigned_field++;
                }
                else{
                    curr_frame.addSignedField(curr_signed_field, tag2_3s32_values[i]);
                    curr_signed_field++;
                }

                curr_field++;
            }

            final_value = false;
            decoding_tag2_3s32 = false;

        }
        else if(decoding_tag8_4s16 && final_value)
        {
            //If we are doing tag2_3s32 and have decoded all field values, want to place
            //those into frame
            for(int i = 0; i < tag8_4s16_fields; i++){
                curr_sign = field_signs[curr_field];
                if(curr_sign == 0)
                {
                    curr_frame.addUnsignedField(curr_unsigned_field, (uint32_t)tag2_3s32_values[i]);
                    curr_unsigned_field++;
                }
                else{
                    curr_frame.addSignedField(curr_signed_field, tag2_3s32_values[i]);
                    curr_signed_field++;
                }

                curr_field++;
            }

            final_value = false;
            decoding_tag8_4s16 = false;
        }
        else if(!decoding_tag2_3s32 && !decoding_tag8_4s16 && !decoding_tag8_8svb && final_value)
        {
			if(curr_sign == 0)
			{
				curr_frame.addUnsignedField(curr_unsigned_field,curr_value);
				curr_unsigned_field++;
			}
			else
			{
				curr_frame.addSignedField(curr_signed_field,scurr_value);
				curr_signed_field++;
			}

			//Increment decoding count
			curr_field++;
			final_value = false;
        }

		//Apply decompression once all fields are decoded
		if(curr_field >= (NUM_FIELDS - 1))
		{
            curr_predictor = 0;
            curr_signed_field = 0;
            curr_unsigned_field = 0;

            for(int i = 0; i < NUM_FIELDS; i++)
            {
                if(decoding_i_frame)
                    curr_predictor = i_field_predictors[i];
                if(decoding_p_frame)
                    curr_predictor = p_field_predictors[i];

                curr_sign = field_signs[i];

                //Get the current field value we want to decompress
                int32_t field_val;
                if(curr_sign == 0)
                {
                    field_val = (int32_t) curr_frame.getUnsignedField(curr_unsigned_field);
                }
                else
                {
                    field_val = curr_frame.getSignedField(curr_signed_field);
                }


                //How do we decompress this field?
                switch(curr_predictor)
                {
                    case 0:
                        field_val = predictZero(curr_sign, field_val);
                        break;
                    case 1:
                        field_val = predictLastValue(curr_sign, field_val);
                        break;
                    case 2:
                        field_val = predictStraightLine(curr_sign, field_val);
                        break;
                    case 3:
                        field_val = predictAverage2(curr_sign, field_val);
                        break;
                    case 4:
                        field_val = predictMinThrottle(curr_sign, field_val);
                        break;
                    case 5:
                        field_val = predictMotor0(curr_sign, field_val);
                        break;
                    case 6:
                        field_val = predictIncrement(curr_sign, field_val);
                        break;
                    default:
                        configASSERT(0);
                        break;
                }

                //Assign field_val back to corresponding field in data frame
                if(curr_sign == 0)
                {
                    curr_frame.addUnsignedField(curr_unsigned_field, (uint32_t) field_val);
                    curr_unsigned_field++;
                }
                else
                {
                    curr_frame.addSignedField(curr_signed_field, field_val);
                    curr_signed_field++;
                }

            }

            //When done -- reset various state variables and indicate frame is done
            if(decoding_i_frame)
                i_frame_decoded = true;
            final_value = false;
            frame_complete = true;
            curr_unsigned_field = 0;
            curr_signed_field = 0;
            header_read = false;
            curr_value = 0;
            scurr_value = 0;
            decoding_i_frame = false;
            decoding_p_frame = false;
            fields_decoded = 0;
            curr_field = 0;
            frame_id_found = false;

            //Set this frame to previous frame for next run
            prev_frame2 = prev_frame;
            prev_frame = curr_frame;
            //Add logic here to do something with completed frame

        }
	}

}
/*
 * HELPER FUNCTIONS
 */



/*
 * BEGIN DECODER FUNCTIONS
 */

void BlackboxParser::decodeUVB(Stream &bb_stream, bool sign)
{
	int i, c, shift = 0;
	uint32_t result;

	for(i=0; i<5; i++)
	{
		c = bb_stream.popFromStream();
		result = result |((c & ~0x80) << shift);

		//Final Byte?
		if(c<128){
			if(sign == 0)
				curr_value = result;
			else
				scurr_value = (int32_t) result;
			final_value = true;
			return;
		}

		shift += 7;
	}

	//If more than 4 bytes, we will set value equal to 0
	if(sign == 0)
		curr_value = 0;
	else
		scurr_value = 0;

	final_value = true;
}

void BlackboxParser::decodeSVB(Stream &bb_stream, bool sign)
{
	//This function works on signed values
	//Decode UVB to start
	decodeUVB(bb_stream, sign);

    if(sign == 0)
    {
        curr_value = (uint32_t)zigZagDecode(curr_value);
    }
    else
    {
    	scurr_value = zigZagDecode(scurr_value);
    }

    final_value = true;

}

int32_t BlackboxParser::decodeSVB_tag8(Stream &bb_stream, bool sign)
{
	decodeUVB(bb_stream, sign);

	if(sign == 0)
	    {
	        curr_value = (uint32_t)zigZagDecode(curr_value);
	        return (int32_t) curr_value;
	    }
	    else
	    {
	    	scurr_value = zigZagDecode(scurr_value);
	    	return scurr_value;
	    }

}

void BlackboxParser::decodeTAG2_3S32(Stream &bb_stream, bool sign)
{
	uint8_t leadByte;
	uint8_t byte1, byte2, byte3, byte4;
	int i;

	leadByte = bb_stream.popFromStream();

	// Check the selector in the top two bits to determine the field layout
	switch (leadByte >> 6) {
		case 0:
			// 2-bit fields
			tag2_3s32_values[0] = signExtend2Bit((leadByte >> 4) & 0x03);
			tag2_3s32_values[1] = signExtend2Bit((leadByte >> 2) & 0x03);
			tag2_3s32_values[2] = signExtend2Bit(leadByte & 0x03);
			break;
		case 1:
			// 4-bit fields
			tag2_3s32_values[0] = signExtend4Bit(leadByte & 0x0F);

			leadByte = bb_stream.popFromStream();

			tag2_3s32_values[1] = signExtend4Bit(leadByte >> 4);
			tag2_3s32_values[2] = signExtend4Bit(leadByte & 0x0F);
			break;
		case 2:
			// 6-bit fields
			tag2_3s32_values[0] = signExtend6Bit(leadByte & 0x3F);

			leadByte = bb_stream.popFromStream();
			tag2_3s32_values[1] = signExtend6Bit(leadByte & 0x3F);

			leadByte = bb_stream.popFromStream();
			tag2_3s32_values[2] = signExtend6Bit(leadByte & 0x3F);
			break;
		case 3:
			// Fields are 8, 16 or 24 bits, read selector to figure out which field is which size

			for (i = 0; i < 3; i++) {
				switch (leadByte & 0x03) {
					case 0: // 8-bit
						byte1 = bb_stream.popFromStream();

						// Sign extend to 32 bits
						tag2_3s32_values[i] = (int32_t) (int8_t) (byte1);
						break;
					case 1: // 16-bit
						byte1 = bb_stream.popFromStream();
						byte2 = bb_stream.popFromStream();

						// Sign extend to 32 bits
						tag2_3s32_values[i] = (int32_t) (int16_t) (byte1 | (byte2 << 8));
						break;
					case 2: // 24-bit
						byte1 = bb_stream.popFromStream();
						byte2 = bb_stream.popFromStream();
						byte3 = bb_stream.popFromStream();

						tag2_3s32_values[i] = signExtend24Bit(byte1 | (byte2 << 8) | (byte3 << 16));
						break;
					case 3: // 32-bit
						byte1 = bb_stream.popFromStream();
						byte2 = bb_stream.popFromStream();
						byte3 = bb_stream.popFromStream();
						byte4 = bb_stream.popFromStream();

						tag2_3s32_values[i] = (int32_t) (byte1 | (byte2 << 8) | (byte3 << 16) | (byte4 << 24));
						break;
				}

				leadByte >>= 2;
			}
			break;
	}

	final_value = true;
}

void BlackboxParser::decodeTAG8_4S16(Stream &bb_stream, bool sign)
{
	uint8_t selector;
	uint8_t char1, char2;
	uint8_t buffer;
	int nibbleIndex;

	int i;

	enum {
		FIELD_ZERO  = 0,
		FIELD_4BIT  = 1,
		FIELD_8BIT  = 2,
		FIELD_16BIT = 3
	};

	selector = bb_stream.popFromStream();

	//Read the 4 values from the stream
	nibbleIndex = 0;
	for (i = 0; i < 4; i++) {
		switch (selector & 0x03) {
			case FIELD_ZERO:
				tag8_4s16_values[i] = 0;
				break;
			case FIELD_4BIT:
				if (nibbleIndex == 0) {
					buffer = (uint8_t) bb_stream.popFromStream();
					tag8_4s16_values[i] = signExtend4Bit(buffer >> 4);
					nibbleIndex = 1;
				} else {
					tag8_4s16_values[i] = signExtend4Bit(buffer & 0x0F);
					nibbleIndex = 0;
				}
				break;
			case FIELD_8BIT:
				if (nibbleIndex == 0) {
					//Sign extend...
					tag8_4s16_values[i] = (int32_t) (int8_t) bb_stream.popFromStream();
				} else {
					char1 = buffer << 4;
					buffer = (uint8_t) bb_stream.popFromStream();;

					char1 |= buffer >> 4;
					tag8_4s16_values[i] = (int32_t) (int8_t) char1;
				}
				break;
			case FIELD_16BIT:
				if (nibbleIndex == 0) {
					char1 = (uint8_t) bb_stream.popFromStream();;
					char2 = (uint8_t) bb_stream.popFromStream();;

					//Sign extend...
					tag8_4s16_values[i] = (int16_t) (uint16_t) ((char1 << 8) | char2);
				} else {
					/*
					 * We're in the low 4 bits of the current buffer, then one byte, then the high 4 bits of the next
					 * buffer.
					 */
					char1 = (uint8_t) bb_stream.popFromStream();;
					char2 = (uint8_t) bb_stream.popFromStream();;

					tag8_4s16_values[i] = (int16_t) (uint16_t) ((buffer << 12) | (char1 << 4) | (char2 >> 4));

					buffer = char2;
				}
				break;
		}

		selector >>= 2;
	}

	final_value = true;
}

void BlackboxParser::decodeTAG8_8SVB(Stream& bb_stream, bool sign)
{

	//First an 8 bit header is written. The bits in this field all correspond
	//to a field to be written (e.g. if LSB is 1 then first field will be non-zero).
	/*uint8_t header = bb_stream.popFromStream();

	for(int i = 0; i<tag8_8svb_fields; i++, header >>= 1)
	{
		tag8_8svb_values[i] = (header & 0x01) ? decodeSVB_tag8(bb_stream, sign) : 0;
	}

	final_value = true;
	*/


	//First - check LSB. If 1, then decode this byte and set to current field
	if((header_bytes & (1 << 0)) == 1)
	{
		decodeSVB(bb_stream, sign);
		header_bytes = header_bytes <<= 1;
		final_value = true;
	}
	else
	{
		scurr_value = 0;
		final_value = true;
	}


}

void BlackboxParser::decodeNULL(Stream& bb_stream, bool sign)
{
    if(sign == 0)
        curr_value = bb_stream.popFromStream();
    else
        scurr_value = (int8_t) bb_stream.popFromStream();

	final_value = true;
}


/*
 * BEGIN PREDICTOR FUNCTIONS
 */

/*
 * predictLastValue subtracts the previous value of the field from
 * the raw field value.
 */
int32_t BlackboxParser::predictLastValue(bool sign, int32_t value)
{
	if(sign == 0)
		value += prev_frame.unsigned_field_values[curr_unsigned_field];
	else
		value += prev_frame.signed_field_values[curr_signed_field];

    return value;
}

/*
 * predictZero does not modify the field value
 */
int32_t BlackboxParser::predictZero(bool sign, int32_t value)
{
	return value;
}

/*
 * predictStraightLine assumes the slope between the current measurement (cur_frame)
 * and the previous measurement(prev_frame) will be similar to the slope between the
 * previous measurement and the measurement before that (prev_frame2)
 */
int32_t BlackboxParser::predictStraightLine(bool sign, int32_t value)
{
	//Signed Field values
	int32_t sprev_val;
	int32_t sprev_val2;

	//Unsigned Field values
	uint32_t uprev_val2;
	uint32_t uprev_val;


	if(sign == 0)
		uprev_val2 = prev_frame2.unsigned_field_values[curr_unsigned_field];
	else
		sprev_val2 = prev_frame2.signed_field_values[curr_signed_field];



	if(sign == 0)
		uprev_val = prev_frame.unsigned_field_values[curr_unsigned_field];
	else
		sprev_val = prev_frame.signed_field_values[curr_signed_field];


	//Compute slope and add to curr_value -- check if we are working
	//with signed or unsigned field
	if(sign == 0)
		value += (uprev_val2 - (2 * uprev_val));
	else
		value += (sprev_val2 - (2 * sprev_val));

    return value;

}

/*
 * Uses the average of the two previously computed field vaues
 * as the predictor
 */
int32_t BlackboxParser::predictAverage2(bool sign, int32_t value)
{
	//Unsigned field
	uint32_t uprev_val2;
	uint32_t uprev_val;

	//Signed field
	int32_t sprev_val2;
	int32_t sprev_val;

	if(sign == 0)
		uprev_val2 = prev_frame2.unsigned_field_values[curr_unsigned_field];
	else
		sprev_val2 = prev_frame2.signed_field_values[curr_signed_field];



	if(sign == 0)
		uprev_val = prev_frame.unsigned_field_values[curr_unsigned_field];
	else
		sprev_val = prev_frame.signed_field_values[curr_signed_field];

	//Compute slope and add to curr_value
	if(sign == 0)
		value += (uprev_val + uprev_val2)/2;
	else
		value += (sprev_val + sprev_val2)/2;

    return value;
}

/*
 * Uses min_throttle value from header as predictor. This value is 1150
 */
int32_t BlackboxParser::predictMinThrottle(bool sign, int32_t value)
{
    value += min_throttle;
    return value;
}

/*
 * Uses motor0 value which was predicted earlier in decoding as the predictor.
 */
int32_t BlackboxParser::predictMotor0(bool sign, int32_t value)
{
	//This function should never be used with a signed value...
	value += curr_frame.unsigned_field_values[MOTOR0];
    return value;
}

/*
 * Assumes field will be incremented by one.
 */
int32_t BlackboxParser::predictIncrement(bool sign, int32_t value)
{
    value++;
    return value;
}

/*
 * Tool Functions for sign extension
 */

int32_t BlackboxParser::signExtend24Bit(uint32_t u)
{
    //If sign bit is set, fill the top bits with 1s to sign-extend
    return (u & 0x800000) ? (int32_t) (u | 0xFF000000) : (int32_t) u;
}

int32_t BlackboxParser::signExtend14Bit(uint16_t word)
{
    //If sign bit is set, fill the top bits with 1s to sign-extend
    return (word & 0x2000) ? (int32_t) (int16_t) (word | 0xC000) : word;
}

int32_t BlackboxParser::signExtend6Bit(uint8_t byte)
{
    //If sign bit is set, fill the top bits with 1s to sign-extend
    return (byte & 0x20) ? (int32_t) (int8_t) (byte | 0xC0) : byte;
}

int32_t BlackboxParser::signExtend4Bit(uint8_t nibble)
{
    //If sign bit is set, fill the top bits with 1s to sign-extend
    return (nibble & 0x08) ? (int32_t) (int8_t) (nibble | 0xF0) : nibble;
}

int32_t BlackboxParser::
signExtend2Bit(uint8_t byte)
{
    //If sign bit is set, fill the top bits with 1s to sign-extend
    return (byte & 0x02) ? (int32_t) (int8_t) (byte | 0xFC) : byte;
}

int32_t BlackboxParser::zigZagDecode(uint32_t value)
{
	return (value >> 1) ^ -(int32_t) (value & 1);
}
