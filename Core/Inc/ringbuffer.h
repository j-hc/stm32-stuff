#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define RB_DATA uint16_t

typedef struct {
	size_t size;
	RB_DATA *buf;
	size_t head;
	size_t tail;
} RingBuffer;

bool rb_is_full(RingBuffer *rb);
bool rb_is_empty(RingBuffer *rb);
void rb_queue(RingBuffer *rb, RB_DATA data);
RB_DATA rb_dequeue(RingBuffer *rb);
RingBuffer rb_init(RB_DATA *buf, size_t size);
void rb_print(RingBuffer *rb);

#endif  // __RING_BUFFER_H
