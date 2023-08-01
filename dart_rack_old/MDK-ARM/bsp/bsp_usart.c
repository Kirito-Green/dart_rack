#include "bsp_usart.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;

void usart1_rx_dma_init(uint8_t *rx1_buf, uint16_t dma_buf_num)
{
    // 开启idle中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    // 开启接收
    HAL_UART_Receive_DMA(&huart1, rx1_buf, dma_buf_num);
}

void usart1_rx_dma_unable(void)
{
    __HAL_UART_DISABLE(&huart1);
}

void usart1_rx_dma_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart1);
    __HAL_DMA_DISABLE(&hdma_usart1_rx);

    hdma_usart1_rx.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart1_rx);
    __HAL_UART_ENABLE(&huart1);
}

void usart6_rx_dma_init(uint8_t *rx6_buf, uint16_t dma_buf_num)
{
    // 开启idle中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    // 开启接收
    HAL_UART_Receive_DMA(&huart6, rx6_buf, dma_buf_num);
}
