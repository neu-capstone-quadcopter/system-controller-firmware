#ifndef UTIL_BLACKBOX_STREAM_HPP
#define UTIL_BLACKBOX_STREAM_HPP

#define STREAM_BUFFER_SIZE 200

#include "FreeRTOS.h"
#include "semphr.h"

#define SEMAPHORE_WAIT_TIME 10

class Stream {
public:

	Stream()
	{
		read_ptr = stream_buffer;
		write_ptr = stream_buffer;
		stream_read_semaphore = xSemaphoreCreateBinary();
	}

	bool streamIsEmpty()
	{
		return read_ptr == write_ptr;
	}

	void addToStream(uint8_t* data, uint8_t len)
	{

		if(len >= STREAM_BUFFER_SIZE)
		{
			//Potentially throw some error if len too large for buffer
		}

		if((write_ptr + len) <= (stream_buffer + STREAM_BUFFER_SIZE))
		{
			memcpy(write_ptr, data, len);
			write_ptr += len;
		}
		else
		{
			uint8_t rollover = (uint8_t)((write_ptr + len) - (stream_buffer + STREAM_BUFFER_SIZE));
			uint8_t amt_to_write = (reinterpret_cast<uint32_t>(write_ptr) + len) - rollover;
			memcpy(write_ptr, data, amt_to_write);
			write_ptr = stream_buffer;
			memcpy(write_ptr, data + amt_to_write, rollover);
			write_ptr += rollover;

		}

		xSemaphoreGiveFromISR(stream_read_semaphore, NULL);

	}

	uint8_t popFromStream()
	{
		//Pop one byte at a time based on our read ptr
		if(read_ptr == write_ptr)
		{
			xSemaphoreTake(stream_read_semaphore, SEMAPHORE_WAIT_TIME);
		}

		uint8_t val = *read_ptr;
		if(read_ptr < (stream_buffer + STREAM_BUFFER_SIZE))
			read_ptr ++;
		else
			read_ptr = stream_buffer;

		return val;
	}


private:
	uint8_t stream_buffer[STREAM_BUFFER_SIZE]; //Use char* to leverage str functions
	uint8_t *read_ptr;
	uint8_t *write_ptr;
	SemaphoreHandle_t stream_read_semaphore;
};

#endif //UTIL_BLACKBOX_STREAM_HPP
