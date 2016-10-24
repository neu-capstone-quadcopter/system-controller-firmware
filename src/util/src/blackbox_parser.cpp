#include "blackbox_parser.hpp"


/*
 * This function will parse our stream until it finds either an
 * \nI or a \nP and pick the correct decoding scheme
 */
BlackboxParser::decodeFrame(Stream &bb_stream){

	//Skip this logic if we're in the middle of decoding a frame
	if(decoding_i_frame)
	{
		decodeIFrame(bb_stream);
	}
	else if(decoding_p_frame)
	{
		decodePFrame(bb_stream);
	}
	else
	{
		//Let's figure out what type of frame we're looking at. The below logic
		//assumes that we won't get a sequence of two newline characters in a row
		//before our frame specifier (e.g. "\n\nI"
		uint8_t curr_byte = bb_stream.popFromStream();
		if(curr_byte == '\n')
		{
			uint8_t next_byte = bb_stream.popFromStream();
			if(next_byte == 'I')
				decodeIFrame(bb_stream);
			else if(next_byte == 'P')
				decodePFrame(bb_stream);
		}
	}
}

/*
 * This function will decode any I Frames that are contained within our stream
 */
BlackboxParser::decodeIFrame(Stream &bb_stream){
	//We are now decoding I Frame
	decoding_i_frame = true;

	//Figure out the predictor/encoding used for this field/byte
	uint8_t curr_predictor = i_field_predictors[curr_field];
	uint8_t curr_encoding = i_field_encodings[curr_field];

	uint8_t curr_byte = bb_stream.popFromStream();

	//Let's decode this byte
	switch(curr_encoding)
	{
	case 0:
		decodeUVB(curr_byte);
		break;
	case 1:
		decodeSVB(curr_byte);
		break;
	case 6:
		decodeTAG8_8SVB(curr_byte);
		break;
	case 7:
		decodeTAG2_2S32(curr_byte);
		break;
	case 8:
		decodeTAG8_4S16(curr_byte);
		break;
	case 9:
		decodeNull(curr_byte);
		break;
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
	}


}
