#ifndef BSP_UART_DRIVER_H__
#define BSP_UART_DRIVER_H__

void uart_driver_func(void *argument);

uint8_t get_circular_buffer_handle(void **ppbuf);

void dma_half_irq_callback(uint32_t num_of_data);
void uart_idle_irq_callback(uint32_t num_of_data);
void dma_complete_irq_callback(uint32_t num_of_data);
#endif /* BSP_UART_DRIVER_H */

