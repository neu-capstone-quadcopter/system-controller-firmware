#ifndef UTIL_BLACKBOX_PARSER_HPP
#define UTIL_BLACKBOX_PARSER_HPP

#include <telemetry_message.hpp>
#include <cstdint>
#include <cstring>
#include "FreeRTOS.h"
#include "blackbox_stream.hpp"
#include "nav_computer_task.hpp"

#define NUM_FIELDS 29
#define NUM_MSG_FIELDS 9

enum bb_unsigned_frame_fields{
	LOOP_ITERAION,
	TIME,
	RC_COMMAND3,
	VBAT_LATEST,
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

    uint32_t getUnsignedField(uint8_t curr_field)
    {
        return unsigned_field_values[curr_field];
    }

    int32_t getSignedField(uint8_t curr_field)
    {
        return signed_field_values[curr_field];
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
	void decodeUVB(Stream &, bool); //0
	void decodeSVB(Stream&, bool); //1
	int32_t decodeSVB_tag8(Stream&, bool); //N/A
	void decodeNeg14(Stream&, bool); //3
	void decodeTAG8_8SVB(Stream&, bool); //6
	void decodeTAG2_3S32(Stream&, bool); //7
	void decodeTAG8_4S16(Stream&, bool); //8
	void decodeNULL(Stream&, bool); //9

	//Field Predictors
	int32_t predictZero(bool,int32_t); //0
	int32_t predictLastValue(bool,int32_t); //1
	int32_t predictStraightLine(bool,int32_t); //2
	int32_t predictAverage2(bool,int32_t); //3
	int32_t predictMinThrottle(bool,int32_t); //4
	int32_t predictMotor0(bool,int32_t); //5
	int32_t predictIncrement(bool,int32_t); //6

    //Tool functions
    int32_t signExtend24Bit(uint32_t u);
    int32_t signExtend14Bit(uint16_t word);
    int32_t signExtend6Bit(uint8_t byte);
    int32_t signExtend4Bit(uint8_t nibble);
    int32_t signExtend2Bit(uint8_t byte);
    int32_t zigZagDecode(uint32_t value);


private:

    //TEST
    uint8_t blackbox_values[300];
    uint8_t blackbox_values_index = 0;

	//Flag value used to keep track of which field we are decoding
	uint8_t curr_field = 0;
	uint8_t curr_unsigned_field = 0;
	uint8_t curr_signed_field = 0;
    uint8_t fields_decoded = 0;

    //Used in UVB Decoding
    uint8_t uvb_num_bytes = 0;


	//Used if a field requires more than one byte to represent
	uint32_t curr_value = 0;
	int32_t scurr_value;
	uint8_t num_bytes_used = 0;

	//This used to store header byte of TAG decoders
	uint32_t header_bytes;
	bool header_read; //have we already read our header bits?
	uint8_t tag8_fields_completed = 0;

    //Used in Tag2_3s32
    int32_t tag2_3s32_values[3];
    uint8_t tag2_3s32_fields = 3;
    bool decoding_tag2_3s32 = false;

    //Used in tag8_4s16
    int32_t tag8_4s16_values[4];
    uint8_t tag8_4s16_fields = 4;
    bool decoding_tag8_4s16 = false;

    //Used in tag8_8svb
    int32_t tag8_8svb_values[7];
    uint8_t tag8_8svb_fields = 7;
    bool decoding_tag8_8svb = false;

	//Flags indicating decoding
    bool frame_id_found = false;
	bool decoding_i_frame = false;
	bool decoding_p_frame = false;
    bool i_frame_decoded = false; //Used as pre-req for P frame decoding to start
	bool final_value = true;
	bool frame_complete = false;

    //Can we start applying predictor logic?
    bool start_decompression = false;


	//Min throttle value for prediction
	uint32_t min_throttle = 1150;

	bool field_signs[NUM_FIELDS] = {0,0,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,0,1,1,1,1,1,1,0,0,0,0};
	uint8_t i_field_predictors[NUM_FIELDS] = {0,0,0,0,0,0,0,0,0,0,0,4,9,0,0,0,0,0,0,0,0,0,0,0,0,4,5,5,5};
	uint8_t p_field_predictors[NUM_FIELDS] = {6,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,3,3,3};
	uint8_t i_field_encodings[NUM_FIELDS] = {1,1,0,0,0,0,0,0,0,0,0,1,3,1,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0};
	uint8_t p_field_encodings[NUM_FIELDS] = {9,0,0,0,0,7,7,7,8,8,8,8,6,6,6,6,6,6,6,0,0,0,0,0,0,0,0,0,0};

	//Working Frames
	blackbox_frame prev_frame2;
	blackbox_frame prev_frame;
	blackbox_frame curr_frame;

	//The message we will send to the Nav Computer
	TelemetryMessage msg_to_send;
};

#endif //UTIL_BLACKBOX_PARSER_HPP
