

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "queue.h"
#include "usart.h"
#include "cmsis_os.h"
#include "bsp_uart_driver.h"
#include "elog.h"
#include "circular_buffer.h"

#define TAG "bsp_uart_driver"
#define BUFFER_A 0
#define BUFFER_B 1
#define IRQ_BUFFER_RDY_SIGNAL 0xA1
#define DATA_RDY_SIGNAL 0xA2

extern UART_HandleTypeDef huart1;
extern QueueHandle_t queue_data_proc;

#if 0 //AB buffer test
uint8_t buffer_flag = BUFFER_A;
uint8_t g_data_buffer_A[1] = {0x00};
uint8_t g_data_buffer_B[1] = {0x00};
#endif

static QueueHandle_t uart_receive_irq_queue = NULL;
static CircularBuffer_t *g_pbuf_irq = NULL;
uint8_t g_received_byte = 0;

void uart_driver_func(void *argument)
{
    CircularBuffer_t *pbuf = NULL;
    pbuf = create_Empty_Circular_Buffer();
    if (NULL == pbuf)
    {
        elog_error(TAG, "Circular buffer creation failed");
        return;
    }
    else
    {
        elog_info(TAG, "Circular buffer creation success");
        g_pbuf_irq = pbuf; // Assign to global pointer for ISR access
    }

    uart_receive_irq_queue = xQueueCreate(1, 4);
    if (NULL == uart_receive_irq_queue)
    {
        elog_error(TAG, "uart_receive_irq_queue creation failed");
        return;
    }
    else
    {
        elog_info(TAG, "uart_receive_irq_queue creation success");
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, pbuf->buffer, CIRCULAR_BUFFER_SIZE);

#if 0
    if (HAL_OK == HAL_UART_Receive_IT(&huart1, &g_received_byte, 1))
    {
        /* Start UART receive interrupt */
        elog_info(TAG, "Start UART receive interrupt successfully!");
    }
    else
    {
        elog_error(TAG, "Start UART receive interrupt failed!");
    }
#endif

#if 0
    buffer_flag = BUFFER_A;
    if (HAL_OK == HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_data_buffer_A, 1))
    {
        /* Start UART receive interrupt */
        elog_info(TAG, "Start UART receive interrupt successfully!");
    }
    else
    {
        elog_error(TAG, "Start UART receive interrupt failed!");
    }
#endif
    uint8_t receivedata = 0;
    // Implement UART driver functionality here
    for(;;)
    {   elog_info(TAG, "uart_rec_A_func is running...");
        if (pdTRUE == xQueueReceive(uart_receive_irq_queue, &receivedata, portMAX_DELAY))
        {
            elog_info(TAG, "Data received from queue: %x", receivedata);
        }

        if(IRQ_BUFFER_RDY_SIGNAL == receivedata)
        {
            // uint8_t data_from_cbuf = 0;
            // if(0x01 == read_data(g_pbuf_irq, &data_from_cbuf))
            // {
            //     elog_info(TAG, "Data read from circular buffer: %c", data_from_cbuf);
            // }
        }

        uint8_t data_to_send = DATA_RDY_SIGNAL;
        if(pdPASS == xQueueOverwrite(queue_data_proc, &data_to_send))
        {
            printf("Data sent to queue_data_proc successfully\r\n");
        }


        //osDelay(1000);
    }
}

uint8_t get_circular_buffer_handle(void **ppbuf)
{
    if(NULL == ppbuf)
    {
        elog_error(TAG, "Pointer to pointer is NULL");
        return 0;
    }
    *ppbuf = (void *)g_pbuf_irq;
    return 1;
}

/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    elog_info(TAG, "HAL_UART_RxCpltCallback");
#if 0 //AB buffer test
    if(BUFFER_A == buffer_flag)
    {
        elog_debug(TAG, "A:Received data: %c", g_data_buffer_A[0]);
        buffer_flag = BUFFER_B;
        if (HAL_OK == HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_data_buffer_B, 1))
        {
            /* Start UART receive interrupt */
            elog_info(TAG, "Switch to buffer B and start UART receive interrupt successfully!");
        }
        else
        {
            elog_error(TAG, "Switch to buffer B and start UART receive interrupt failed!");
        }
    }
    else
    {
        elog_debug(TAG, "B:Received data: %c", g_data_buffer_B[0]);
        buffer_flag = BUFFER_A;
        if (HAL_OK == HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_data_buffer_A, 1))
        {
            /* Start UART receive interrupt */
            elog_info(TAG, "Switch to buffer A and start UART receive interrupt successfully!");
        }
        else
        {
            elog_error(TAG, "Switch to buffer A and start UART receive interrupt failed!");
        }
    }
#endif

#if 0 //circular buffer test
    if(NULL == g_pbuf_irq)
    {
        elog_error(TAG, "Global circular buffer pointer is NULL");
        return;
    }
    
    uint8_t insert_status = 0;
    insert_status = insert_data(g_pbuf_irq, g_received_byte);
    if(0x01 == insert_status)
    {
        elog_debug(TAG, "Inserted data: %c into circular buffer", g_received_byte);
        if(HAL_OK == HAL_UART_Receive_IT(&huart1, &g_received_byte, 1))
        {
            /* Restart UART receive interrupt */
            elog_info(TAG, "Restart UART receive interrupt successfully!");
        }
        else
        {
            elog_error(TAG, "Restart UART receive interrupt failed!");
        }
    }
    else if(0x00 == insert_status)
    {
        elog_error(TAG, "Circular buffer is full, cannot insert data: %c", g_received_byte);
    }
    else
    {
        elog_error(TAG, "Error inserting data into circular buffer");
    }
#endif

    if(NULL == g_pbuf_irq)
    {
        elog_error(TAG, "Global circular buffer pointer is NULL");
        return;
    }
    
    uint8_t insert_status = 0;
    insert_status = insert_data(g_pbuf_irq, g_received_byte);
    if(0x01 == insert_status)
    {
        elog_debug(TAG, "Inserted data: %c into circular buffer", g_received_byte);
        if(HAL_OK == HAL_UART_Receive_IT(&huart1, &g_received_byte, 1))
        {
        }
    }

    uint8_t data_to_send = IRQ_BUFFER_RDY_SIGNAL;
    if(pdPASS == xQueueOverwriteFromISR(uart_receive_irq_queue, &data_to_send, NULL))
    {
        //printf("Data sent to queue successfully\r\n");
    }

}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(NULL == g_pbuf_irq)
    {
        elog_error(TAG, "Global circular buffer pointer is NULL");
        return;
    }

    elog_debug(TAG, "HAL_UARTEx_RxEventCallback");
    elog_debug(TAG, "size = [%d]", Size);

}

void dma_half_irq_callback(uint32_t num_of_data)
{
    uint32_t head_pos = 0;
    if(NULL == g_pbuf_irq)
    {
        elog_error("HALF", "Global circular buffer pointer is NULL");
        return;
    }
    if(get_head_pos(g_pbuf_irq, &head_pos))
    {
        // elog_debug("HALF", "DMA half IRQ callback, head pos: %d", head_pos);
    }
    else
    {
        elog_error("HALF", "Get head position failed");
        return;
    }
    uint32_t data_pos_in_buf = (CIRCULAR_BUFFER_SIZE/2)-1;
    // elog_debug("HALF", "data_pos_in_buf = %d", data_pos_in_buf);
    uint32_t head_pos_in_buf = head_pos % CIRCULAR_BUFFER_SIZE;
    // elog_debug("HALF", "head_pos_in_buf = %d", head_pos_in_buf);
    uint32_t shift = 0;
    if(data_pos_in_buf < head_pos_in_buf)
    {
        shift = (data_pos_in_buf + CIRCULAR_BUFFER_SIZE) - head_pos_in_buf + 1;
        //elog_debug("HALF", "dma complete shift = %d", shift);
    } 
    else
    {
        shift = data_pos_in_buf - head_pos_in_buf + 1;
        //elog_debug("HALF", "dma normal shift = %d", shift);
    }

    if(shift > 0)
    {
        head_pos_increment(g_pbuf_irq, shift);
        get_head_pos(g_pbuf_irq, &head_pos);
        // elog_debug("COMPLETE", "pos aft increment %d", head_pos);
    }
    else
    {
        elog_error("COMPLETE", "Handle negative shift error");
        return;
    }

    // Notify the data ready
    uint8_t data_to_send = IRQ_BUFFER_RDY_SIGNAL;
    if(pdPASS == xQueueOverwriteFromISR(uart_receive_irq_queue, &data_to_send, NULL))
    {
        printf("Data sent to queue successfully\r\n");
    }
}

void dma_complete_irq_callback(uint32_t num_of_data)
{
    uint32_t head_pos = 0;
    if(NULL == g_pbuf_irq)
    {
        elog_error("COMPLETE", "Global circular buffer pointer is NULL");
        return;
    }
    if(get_head_pos(g_pbuf_irq, &head_pos))
    {
        // elog_debug("COMPLETE", "DMA complete IRQ, head pos val: %d", head_pos);
    }
    else
    {
        elog_error("COMPLETE", "Get head position failed");
        return;
    }
    uint32_t data_pos_in_buf = CIRCULAR_BUFFER_SIZE-1;
    // elog_debug("COMPLETE", "data_pos_in_buf = %d", data_pos_in_buf);
    uint32_t head_pos_in_buf = head_pos % CIRCULAR_BUFFER_SIZE;
    // elog_debug("COMPLETE", "head_pos_in_buf = %d", head_pos_in_buf);
    uint32_t shift = 0;
    if(data_pos_in_buf < head_pos_in_buf)
    {
        shift = (data_pos_in_buf + CIRCULAR_BUFFER_SIZE) - head_pos_in_buf + 1;
        //elog_debug("COMPLETE", "dma complete shift = %d", shift);
    } 
    else
    {
        shift = data_pos_in_buf - head_pos_in_buf + 1;
        //elog_debug("COMPLETE", "dma normal shift = %d", shift);
    }
    if(shift > 0)
    {
        head_pos_increment(g_pbuf_irq, shift);
        get_head_pos(g_pbuf_irq, &head_pos);
        // elog_debug("COMPLETE", "pos aft increment %d", head_pos);
    }
    else
    {
        elog_error("COMPLETE", "Handle negative shift error");
        return;
    }

    // Notify the data ready
    uint8_t data_to_send = IRQ_BUFFER_RDY_SIGNAL;
    if(pdPASS == xQueueOverwriteFromISR(uart_receive_irq_queue, &data_to_send, NULL))
    {
        printf("Data sent to queue successfully\r\n");
    }
}

void uart_idle_irq_callback(uint32_t num_of_data)
{
    uint32_t head_pos = 0;
    if(NULL == g_pbuf_irq)
    {
        elog_error("IDLE", "Global circular buffer pointer is NULL");
        return;
    }
    if(get_head_pos(g_pbuf_irq, &head_pos))
    {
        // elog_debug("IDLE", "UART idle IRQ callback, head pos: %d", head_pos);
    }
    else
    {
        elog_error("IDLE", "Get head position failed");
        return;
    }

    uint32_t data_pos_in_buf = (num_of_data-1);
    // elog_debug("IDLE", "data_pos_in_buf = %d", data_pos_in_buf);
    uint32_t head_pos_in_buf = head_pos % CIRCULAR_BUFFER_SIZE;
    // elog_debug("IDLE", "head_pos_in_buf = %d", head_pos_in_buf);
    uint32_t shift = 0;
    if(data_pos_in_buf < head_pos_in_buf)
    {
        shift = (data_pos_in_buf + CIRCULAR_BUFFER_SIZE) - head_pos_in_buf + 1;
        //elog_debug("COMPLETE", "dma complete shift = %d", shift);
    }
    else
    {
        shift = data_pos_in_buf - head_pos_in_buf + 1;
        //elog_debug("COMPLETE", "dma normal shift = %d", shift);
    }
    if(shift > 0)
    {
        head_pos_increment(g_pbuf_irq, shift);
        get_head_pos(g_pbuf_irq, &head_pos);
        // elog_debug("IDLE", "pos aft increment %d", head_pos);
    }
    else
    {
        elog_error("IDLE", "Handle negative shift error");
        return;
    }

    uint8_t data_to_send = IRQ_BUFFER_RDY_SIGNAL;
    if(pdPASS == xQueueOverwriteFromISR(uart_receive_irq_queue, &data_to_send, NULL))
    {
        printf("Data sent to queue successfully\r\n");
    }
}
/* USER CODE END 1 */

