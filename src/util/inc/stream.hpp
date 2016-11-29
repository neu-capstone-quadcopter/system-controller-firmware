#ifndef UTIL_BLACKBOX_STREAM_HPP
#define UTIL_BLACKBOX_STREAM_HPP

#define STREAM_BUFFER_SIZE 200

#include "FreeRTOS.h"
#include "semphr.h"

class Stream {
public:
	void allocate();
	uint8_t pop();
	void push(uint8_t* data, uint8_t len, BaseType_t *woke_task);
private:
	QueueHandle_t stream_queue;
};

#endif //UTIL_BLACKBOX_STREAM_HPP
