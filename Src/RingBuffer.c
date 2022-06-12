
#include <stdlib.h>
#include "RingBuffer.h"


// The definition of our circular buffer structure is hidden from the user
struct circular_buf_t {
	uint8_t * buffer;  // Pointer for the real vector used in the structure
	size_t  tail;      // Index of next free space - write index
	size_t  head;      // Index of oldest data element - read index
	size_t  max;       // Total size of the buffer
	bool    full;      // True if buffer is full
};


static void advance_write_pointer(cbuf_handle_t cbuf);
static void advance_read_pointer(cbuf_handle_t cbuf);



cbuf_handle_t circular_buf_init(uint8_t* buffer, size_t size)
{
	cbuf_handle_t cbuf = malloc(sizeof(circular_buf_t));

	cbuf->buffer = buffer;
	cbuf->max = size;
	circular_buf_reset(cbuf);

	return cbuf;
}


void circular_buf_free(cbuf_handle_t cbuf)
{
	free(cbuf);
}


void circular_buf_reset(cbuf_handle_t cbuf)
{
    cbuf->tail = 0;
    cbuf->head = 0;
    cbuf->full = false;
}


int circular_buf_put(cbuf_handle_t cbuf, uint8_t data)
{
    int result = -1;

    if(!circular_buf_is_full(cbuf))
    {
        cbuf->buffer[cbuf->tail] = data;
        advance_write_pointer(cbuf);
        result = 0;
    }
    return result;
}

int circular_buf_get(cbuf_handle_t cbuf, uint8_t * data)
{
    int result = -1;

    if(!circular_buf_is_empty(cbuf))
    {
        *data = cbuf->buffer[cbuf->head];
        advance_read_pointer(cbuf);

        result = 0;
    }
    return result;
}


void circular_buf_overwrite(cbuf_handle_t cbuf, uint8_t data)
{
    cbuf->buffer[cbuf->tail] = data;

    advance_write_pointer(cbuf);
}


static void advance_write_pointer(cbuf_handle_t cbuf)
{
	if(cbuf->full)
    {
        cbuf->head = (cbuf->head + 1) % cbuf->max;
    }

	cbuf->tail = (cbuf->tail + 1) % cbuf->max;

	// We mark full because we will advance head on the next time around
	cbuf->full = (cbuf->tail == cbuf->head);
}


static void advance_read_pointer(cbuf_handle_t cbuf)
{
	cbuf->full = false;
	cbuf->head = (cbuf->head + 1) % cbuf->max;
}


bool circular_buf_is_empty(cbuf_handle_t cbuf)
{
    return (!cbuf->full && (cbuf->tail == cbuf->head));
}


bool circular_buf_is_full(cbuf_handle_t cbuf)
{
    return cbuf->full;
}


size_t circular_buf_size(cbuf_handle_t cbuf)
{
	size_t size = cbuf->max;

	if(!cbuf->full)
	{
		if(cbuf->tail >= cbuf->head)
		{
			size = (cbuf->tail - cbuf->head);
		}
		else
		{
			size = (cbuf->max + cbuf->tail - cbuf->head);
		}
	}
	return size;
}


size_t circular_buf_capacity(cbuf_handle_t cbuf)
{
	return cbuf->max;
}
