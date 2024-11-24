#include "ringbuffer.h"

#include <assert.h>
#include <stdio.h>

inline bool rb_is_full(RingBuffer* rb) {
    return ((rb->head - rb->tail) % rb->size) == rb->size - 1;
}

inline bool rb_is_empty(RingBuffer* rb) {
    return rb->head == rb->tail;
}

void rb_queue(RingBuffer* rb, RB_DATA data) {
    if (rb_is_full(rb))
        rb->tail = (rb->tail + 1) % rb->size;
    rb->buf[rb->head] = data;
    rb->head = (rb->head + 1) % rb->size;
}

RB_DATA rb_dequeue(RingBuffer* rb) {
    assert(!rb_is_empty(rb));
    RB_DATA ret = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1) % rb->size;
    return ret;
}

RingBuffer rb_init(RB_DATA* buf, size_t size) {
    return (RingBuffer){size, buf, 0, 0};
}

// void rb_print(RingBuffer* rb) {
//     for (size_t i = 0; i < rb->size; i++)
//         printfd("%02d ", rb->buf[i]);
//     printfd("\n");
//     for (size_t i = 0; i < rb->size; i++) {
//         if (i == rb->tail && i == rb->head) {
//             printfd("*");
//         } else if (i == rb->tail) {
//             printfd("~");
//         } else if (i == rb->head) {
//             printfd("^");
//         } else {
//             printfd(" ");
//         }
//         printfd("  ");
//     }
//     printfd("n");
// }
