

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "queue.h"
#include "usart.h"
#include "cmsis_os.h"
#include "bsp_uart_driver.h"
#include "elog.h"

#define TAG "bsp_uart_driver"
#define BUFFER_A 0
#define BUFFER_B 1

extern UART_HandleTypeDef huart1;
extern QueueHandle_t queue_irq_rec_A;

uint8_t buffer_flag = BUFFER_A;
uint8_t g_data_buffer_A[1] = {0x00};
uint8_t g_data_buffer_B[1] = {0x00};

void uart_driver_func(void *argument)
{
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

    // Implement UART driver functionality here
    for(;;)
    {
        // Example: Polling or handling UART events
    }
}

/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
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
    
    // uint8_t data_to_send = 1;
    // if(pdPASS == xQueueSendFromISR(queue_irq_rec_A, &data_to_send, NULL))
    // {
    //     printf("Data sent to queue successfully\r\n");
    // }
    // printf("uart1 interrupt running\r\n");

}
/* USER CODE END 1 */

