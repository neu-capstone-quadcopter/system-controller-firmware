#include "stream.hpp"
#include "uart_console_task.hpp"

void Stream::allocate()
{
	stream_queue = xQueueCreate(250, sizeof(uint8_t));
}

void Stream::push(uint8_t* data, uint8_t len, BaseType_t *woke_task)
{
	for(int i = 0; i < len; i++)
	{
		xQueueSendToBackFromISR(stream_queue, data + i, woke_task);

	}
}

uint8_t Stream::pop()
{
	uint8_t val;
	xQueueReceive(stream_queue, &val, portMAX_DELAY);
	return val;
}
