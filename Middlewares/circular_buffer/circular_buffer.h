#ifndef __CIRCULAR_BUFFER_H__
#define __CIRCULAR_BUFFER_H__

#include <stdint.h>

#define CIRCULAR_BUFFER_SIZE 10

typedef uint8_t data_type_t;

typedef struct {
    data_type_t  buffer[CIRCULAR_BUFFER_SIZE];      // Pointer to the buffer memory
    uint32_t head;         // Index of the head (write position)
    uint32_t tail;         // Index of the tail (read position)
    uint32_t max_size;     // Maximum size of the buffer
    int full;           // Flag to indicate if the buffer is full
} CircularBuffer_t;

CircularBuffer_t *create_Empty_Circular_Buffer(void);
uint8_t is_buffer_empty(CircularBuffer_t *pbuf);
uint8_t is_buffer_full(CircularBuffer_t *pbuf);

uint8_t insert_data(CircularBuffer_t *pbuf, data_type_t data);
uint8_t read_data(CircularBuffer_t *pbuf, data_type_t *data);

uint8_t get_head_pos(CircularBuffer_t *pbuf, uint32_t *head_pos);
uint8_t head_pos_increment(CircularBuffer_t *pbuf, uint32_t increment_value);
#endif /* __CIRCULAR_BUFFER_H__ */

