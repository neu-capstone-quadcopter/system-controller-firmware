#ifndef UTIL_TELEMETRY_PARSER_HPP_
#define UTIL_TELEMETRY_PARSER_HPP_

#include "blackbox_stream.hpp"

class TelemetryParser
{
public:
	//Constructor
	TelemetryParser();

	//Parser Function
	void parseForData(Stream&);

private:

};

#endif //UTIL_TELEMETRY_PARSER_HPP_


//Checklist:
//1. Outline Basic Class Structure
//2. Create a function that will parse a frame
//3. Create a Frame Structure to hold the data
//4. Create Nav Computer Message Type
//5. Test Messages with Nav Computer
