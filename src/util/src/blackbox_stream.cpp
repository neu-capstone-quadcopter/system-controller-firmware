#include "blackbox_stream.hpp"

	Stream::Stream()
	{
		read_ptr = stream_buffer;
		write_ptr = stream_buffer;

		/*uint8_t i_frame_data[45] = {'I',190,190,154,147,6,0,0,177,4,0,0,27,0,0,2,2,231,7,0,214,12,'>',240,11,199,1,'a',255,7,4,7,1,191,5,241,2,164,'A',0,206,4,206,4,0};
		memcpy(write_ptr,i_frame_data,45);
		write_ptr += 45;
		uint8_t p_frame_data1[21]= {'P',208,15,0,0,0,3,0,0,0,0,2,0,1,0,0,2,0,2,2,0};
		memcpy(write_ptr, p_frame_data1, 21);
		write_ptr += 21;
		uint8_t p_frame_data2[20] = {'P',4,0,0,0,3,0,1,0,0,2,2,1,0,0,2,0,6,2,0};
		memcpy(write_ptr, p_frame_data2, 20);
		write_ptr += 20;
		*/

	}

	void Stream::allocate()
	{
		stream_read_semaphore = xSemaphoreCreateBinary();
	}

	bool Stream::streamIsEmpty()
	{
		return read_ptr == write_ptr;
	}

	void Stream::addToStream(uint8_t* data, uint8_t len, BaseType_t *woke_task)
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
			uint8_t amt_to_write = reinterpret_cast<uint32_t>(stream_buffer + STREAM_BUFFER_SIZE) - reinterpret_cast<uint32_t>(write_ptr);
			memcpy(write_ptr, data, amt_to_write);
			write_ptr = stream_buffer;
			memcpy(write_ptr, data + amt_to_write, rollover);
			write_ptr += rollover;

		}

		delete[] data;
		xSemaphoreGiveFromISR(stream_read_semaphore, woke_task);
	}

	uint8_t Stream::popFromStream()
	{
		//Pop one byte at a time based on our read ptr
		if(read_ptr == write_ptr)
		{
			xSemaphoreTake(stream_read_semaphore, portMAX_DELAY);
		}

		uint8_t val = *read_ptr;
		if(read_ptr < (stream_buffer + STREAM_BUFFER_SIZE))
			read_ptr ++;
		else
			read_ptr = stream_buffer;

		return val;
	}
