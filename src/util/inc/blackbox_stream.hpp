#ifndef UTIL_BLACKBOX_STREAM_HPP
#define UTIL_BLACKBOX_STREAM_HPP

#define STREAM_BUFFER_SIZE 200

#include "FreeRTOS.h"
#include "semphr.h"

#define SEMAPHORE_WAIT_TIME 10

class Stream {
public:

	Stream();
	void allocate();
	uint8_t popFromStream();
	void addToStream(uint8_t* data, uint8_t len, BaseType_t *woke_task);
	bool streamIsEmpty();


private:
	uint8_t stream_buffer[STREAM_BUFFER_SIZE];
	uint8_t *read_ptr;
	uint8_t *write_ptr;
	SemaphoreHandle_t stream_read_semaphore;
};

#endif //UTIL_BLACKBOX_STREAM_HPP
