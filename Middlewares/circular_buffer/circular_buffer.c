
#include "circular_buffer.h"
#include <stdlib.h>
#include <string.h>
#include "elog.h"

CircularBuffer_t * create_Empty_Circular_Buffer(void)
{
    CircularBuffer_t *pbuf = NULL;

    pbuf = (CircularBuffer_t *)malloc(sizeof(CircularBuffer_t));
    if (NULL == pbuf)
    {
        elog_error("CircularBuffer", "Failed to allocate memory for circular buffer");
        return NULL;
    }
    memset(pbuf->buffer, 0, sizeof(pbuf->buffer));
    pbuf->max_size = CIRCULAR_BUFFER_SIZE;
    pbuf->head = 0;
    pbuf->tail = 0;
    pbuf->full = 0;

    return pbuf;
}

/**
 * @brief Checks if the circular buffer is empty.
 * 
 * Steps:
 *  1. Check if the buffer pointer is NULL. If it is, log an error and return 0xFF.
 *  2. Compare the head and tail indices of the buffer.
 *  
 * @param[in] pbuf       : Pointer to the circular buffer.
 * 
 * @return 0xFF; : Buffer pointer is NULL.
 * @return 0x00; : Buffer is not empty.
 * @return 0x01; : Buffer is empty.
 * */
uint8_t is_buffer_empty(CircularBuffer_t *pbuf)
{
    if(NULL == pbuf)
    {
        elog_error("CircularBuffer", "Buffer pointer is NULL");
        return 0xFF; // Consider NULL buffer as empty
    }
    return (pbuf->head == pbuf->tail);
}


/**
 * @brief Checks if the circular buffer is full.
 * 
 * Steps:
 *  1. Check if the buffer pointer is NULL. If it is, log an error and return 0xFF.
 *  2. Calculate the next head position and compare it with the tail index.
 *  3. If they are equal, the buffer is full; otherwise, it is not full.
 *  
 * @param[in] pbuf       : Pointer to the circular buffer.
 * 
 * @return 0xFF; : Buffer pointer is NULL.
 * @return 0x00; : Buffer is not full.
 * @return 0x01; : Buffer is full.
 * */
uint8_t is_buffer_full(CircularBuffer_t *pbuf)
{
    if(NULL == pbuf)
    {
        elog_error("CircularBuffer", "Buffer pointer is NULL");
        return 0xFF; // Consider NULL buffer as not full
    }

    if(((pbuf->head + 1) % pbuf->max_size) == (pbuf->tail % pbuf->max_size))
    {
        elog_info("CircularBuffer", "Buffer is full");
        return 0x01; // Buffer is full
    }
    return 0x00; // Buffer is not full
}

/**
 * @brief Inserts data into the circular buffer.
 * 
 * Steps:
 *  1. Check if the buffer pointer is NULL. If it is, log an error and return 0xFF.
 *  2. Check if the buffer is full using is_buffer_full(). 
 *      If it is, log a warning and return 0x00.
 *  3. Insert the data at the head index and update the head index.
 *  4. If the buffer becomes full after insertion, set the full flag.
 *  
 * @param[in] pbuf       : Pointer to the circular buffer.
 * @param[in] data       : Data to be inserted into the buffer.
 * 
 * @return 0xFF; : Buffer pointer is NULL.
 * @return 0x00; : Buffer is full.
 * @return 0x01; : Data inserted successfully.
 * */
uint8_t insert_data(CircularBuffer_t *pbuf, data_type_t data)
{
    if(NULL == pbuf)
    {
        elog_error("CircularBuffer", "Buffer pointer is NULL");
        return 0xFF; // Error: NULL buffer
    }

    if(is_buffer_full(pbuf))
    {
        elog_error("CircularBuffer", "Buffer is full, cannot insert data");
        return 0x00; // Buffer is full
    }

    pbuf->buffer[pbuf->head] = data;
    pbuf->head = (pbuf->head + 1) % pbuf->max_size;

    if(is_buffer_full(pbuf))
    {
        pbuf->full = 1; // Mark buffer as full
    }

    return 0x01; // Data inserted successfully
}

uint8_t read_data(CircularBuffer_t *pbuf, data_type_t *data)
{
    if(NULL == pbuf || NULL == data)
    {
        elog_error("CircularBuffer", "Buffer pointer or data pointer is NULL");
        return 0xFF; // Error: NULL buffer or data pointer
    }

    if(is_buffer_empty(pbuf))
    {
        elog_error("CircularBuffer", "Buffer is empty, cannot read data");
        return 0x00; // Buffer is empty
    }

    *data = pbuf->buffer[pbuf->tail];
    pbuf->tail = (pbuf->tail + 1) % pbuf->max_size;
    pbuf->full = 0; // Mark buffer as not full after reading

    return 0x01; // Data read successfully
}
