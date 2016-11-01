#ifndef UTIL_BLACKBOX_PARSER_HPP
#define UTIL_BLACKBOX_PARSER_HPP

#include <cstdint>
#include <cstring>
#include "FreeRTOS.h"
#include "blackbox_stream.hpp"

#define NUM_FIELDS 30

enum bb_unsigned_frame_fields{
	LOOP_ITERAION,
	TIME,
	RC_COMMAND3,
	AMPERAGE_LATEST,
	RSSI,
	MOTOR0,
	MOTOR1,
	MOTOR2,
	MOTOR3
};

enum bb_signed_frame_fields{
	AXIS_P0,
	AXIS_P1,
	AXIS_P2,
	AXIS_I0,
	AXIS_I1,
	AXIS_I2,
	AXIS_D0,
	AXIS_D1,
	RC_COMMAND0,
	RC_COMMAND1,
	RC_COMMAND2,
	MAG_ADC0,
	MAG_ADC1,
	MAG_ADC2,
	BARO_ALT,
	GYRO_ADC0,
	GYRO_ADC1,
	GYRO_ADC2,
	ACC_SMOOTH0,
	ACC_SMOOTH1,
	ACC_SMOOTH2
};

struct blackbox_frame {

	uint32_t unsigned_field_values[MOTOR3 + 1] = {0};
	int32_t signed_field_values[ACC_SMOOTH2 + 1] = {0};
	bool isEmpty = true;

	void addUnsignedField(uint8_t curr_field, uint32_t curr_value)
	{
		unsigned_field_values[curr_field] = curr_value;
	}

	void addSignedField(uint8_t curr_field, int32_t scurr_value)
	{
		signed_field_values[curr_field] = scurr_value;
	}

	bool isFrameEmpty()
	{
		return isEmpty;
	}

};

class BlackboxParser{
public:
	//Constructor
	BlackboxParser(){};

	//High Level Decodes
	void decodeFrameType(Stream &bb_stream);
	void decodeFrame(Stream &bb_stream, char frame_type);

	//Field Decoders
	void decodeUVB(uint8_t&, Stream &); //0
	void decodeSVB(uint8_t, uint8_t); //1
	void decodeTAG8_8SVB(uint8_t); //6
	void decodeTAG2_2S32(uint8_t); //7
	void decodeTAG8_4S16(uint8_t); //8
	void decodeNULL(uint8_t); //9

	//Field Predictors
	void predictZero(bool); //0
	void predictLastValue(bool); //1
	void predictStraightLine(bool); //2
	void predictAverage2(bool); //3
	void predictMinThrottle(bool); //4
	void predictMotor0(bool); //5
	void predictIncrement(bool); //6


private:

	//Flag value used to keep track of which field we are decoding
	uint8_t curr_field = 0;
	uint8_t curr_unsigned_field = 0;
	uint8_t curr_signed_field = 0;

	//Used if a field requires more than one byte to represent
	uint32_t curr_value = 0;
	int32_t scurr_value;
	uint8_t num_bytes_used = 0;

	//This used to store header byte of TAG decoders
	uint32_t header_bytes;
	bool header_read; //have we already read our header bits?
	uint8_t tag8_fields_completed = 0;
	uint8_t tag32_bytes_required = 0;
	uint8_t tag32_bytes_decoded = 0;
	uint8_t tag16_bytes_required = 0;
	uint8_t tag16_bytes_decoded = 0;


	//Flags indicating decoding
	bool decoding_i_frame = false;
	bool decoding_p_frame = false;
	bool final_value = true;
	bool frame_complete = false;

	//Min throttle value for prediction
	uint32_t min_throttle = 1150;

	bool field_signs[NUM_FIELDS] = {0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,0,1,1,1,1,1,1,0,0,0,0};
	uint8_t i_field_predictors[NUM_FIELDS] = {0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,4,5,5,5};
	uint8_t p_field_predictors[NUM_FIELDS] = {6,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,3,3,3};
	uint8_t i_field_encodings[NUM_FIELDS] = {1,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0};
	uint8_t p_field_encodings[NUM_FIELDS] = {9,0,0,0,0,7,7,7,0,0,8,8,8,8,6,6,6,6,6,6,0,0,0,0,0,0,0,0,0,0};

	//Working Frames
	blackbox_frame prev_frame2;
	blackbox_frame prev_frame;
	blackbox_frame curr_frame;
};

#endif //UTIL_BLACKBOX_PARSER_HPP
