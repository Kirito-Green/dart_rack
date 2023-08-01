#ifndef BSP_USART_H
#define BSP_USART_H

#include "main.h"

extern void usart1_rx_dma_init(uint8_t *rx1_buf, uint16_t dma_buf_num);
extern void usart6_rx_dma_init(uint8_t *rx6_buf, uint16_t dma_buf_num);
extern void usart1_rx_dma_unable(void);
extern void usart1_rx_dma_restart(uint16_t dma_buf_num);

#endif
