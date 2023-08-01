#include "bsp_usart.h"
#include "main.h"
#include "stdio.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart8;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;

/**
 * @brief 该函数使用指定的缓冲区和缓冲区大小初始化 USART1 接收 DMA，并启用空闲中断。
 *
 * @param rx1_buf rx1_buf 参数是指向将存储接收到的数据的缓冲区的指针。它的类型为 uint8_t*，这意味着它是一个指向无符号 8 位整数数组的指针。
 * @param dma_buf_num 参数“dma_buf_num”表示要接收并存储在 DMA 缓冲区中的字节数。
 */
void usart1_rx_dma_init(uint8_t *rx1_buf, uint16_t dma_buf_num)
{
    // 开启idle中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    // 开启接收
    HAL_UART_Receive_DMA(&huart1, rx1_buf, dma_buf_num);
}

/**
 * @brief 该函数禁用 USART1 外设及其关联的 DMA 来接收数据。
 */
void usart1_rx_dma_unable(void)
{
    __HAL_UART_DISABLE(&huart1);
}

/**
 * @brief 函数“usart1_rx_dma_restart”通过禁用和启用 UART 和 DMA，以及重置 DMA 缓冲区编号来重新启动 USART1 接收 DMA。
 *
 * @param dma_buf_num 参数“dma_buf_num”是DMA缓冲区中可以存储的数据项的数量。
 */
void usart1_rx_dma_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart1);
    __HAL_DMA_DISABLE(&hdma_usart1_rx);

    hdma_usart1_rx.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart1_rx);
    __HAL_UART_ENABLE(&huart1);
}

/**
 * @brief 该函数初始化 USART6 外设以使用 DMA 接收数据并启用空闲中断。
 *
 * @param rx6_buf rx6_buf 参数是指向将存储接收到的数据的缓冲区的指针。
 * @param dma_buf_num 参数“dma_buf_num”表示要接收并存储在 DMA 缓冲区中的字节数。
 */
void usart6_rx_dma_init(uint8_t *rx6_buf, uint16_t dma_buf_num)
{
    // 开启idle中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    // 开启接收
    HAL_UART_Receive_DMA(&huart6, rx6_buf, dma_buf_num);
}

/**
 * @brief fputc 函数使用 HAL_UART_Transmit 函数通过 UART 传输字符。
 *
 * @param ch 参数“ch”是要写入文件的字符。它是“int”类型，因为它可以表示字符和文件结束标记（EOF）。
 * @param f 参数“f”是指向 FILE 结构的指针。该结构表示文件流并包含有关正在访问的文件的信息，例如其位置和缓冲状态。
 * 在本例中，它用于指定将用于传输数据的 UART 外设 (huart8)。
 *
 * @return 写入文件的字符。
 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart8, (uint8_t *)&ch, 1, 0xffff);
    return ch;
}
