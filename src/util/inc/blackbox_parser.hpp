#ifndef UTIL_BLACKBOX_PARSER_HPP
#define UTIL_BLACKBOX_PARSER_HPP

class BlackboxParser{
public:
	//High Level Decodes
	void decodeFrame(Stream &bb_stream);
	void decodeIFrame(Stream &bb_stream);
	void decodePFrame(Stream &bb_stream);

	//Field Decoders
	void decodeUVB(uint8_t); //0
	void decodeSVB(uint8_t); //1
	void decodeTAG8_8SVB(uint8_t); //6
	void decodeTAG2_2S32(uint8_t); //7
	void decodeTAG8_4S16(uint8_t); //8
	void decodeNULL(uint8_t); //9

	//Field Predictors
	void predictZero(uint8_t); //0
	void predictLastValue(uint8_t); //1
	void predictStraightLine(uint8_t); //2
	void predictAverage2(uint8_t); //3
	void predictMinThrottle(uint8_t); //4
	void predictMotor0(uint8_t); //5
	void predictIncrement(uint8_t); //6


private:

	//Flag value used to keep track of which field we are decoding
	uint8_t curr_field = 0;

	//Flags indicating decoding
	bool decoding_i_frame = false;
	bool decoding_p_frame = false;

	uint8_t field_signs[] = {0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,0,1,1,1,1,1,1,0,0,0,0};
	uint8_t i_field_predictors[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,4,5,5,5};
	uint8_t p_field_predictors[] = {6,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,3,3,3};
	uint8_t i_field_encodings[] = {1,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0};
	uint8_t p_field_encodings[] = {9,0,0,0,0,7,7,7,0,0,8,8,8,8,6,6,6,6,6,6,0,0,0,0,0,0,0,0,0,0};
};

#endif //UTIL_BLACKBOX_PARSER_HPP
