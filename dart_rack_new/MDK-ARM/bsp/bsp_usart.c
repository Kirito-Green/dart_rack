#include "bsp_usart.h"
#include "main.h"
#include "stdio.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart8;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;

/**
 * @brief �ú���ʹ��ָ���Ļ������ͻ�������С��ʼ�� USART1 ���� DMA�������ÿ����жϡ�
 *
 * @param rx1_buf rx1_buf ������ָ�򽫴洢���յ������ݵĻ�������ָ�롣��������Ϊ uint8_t*������ζ������һ��ָ���޷��� 8 λ���������ָ�롣
 * @param dma_buf_num ������dma_buf_num����ʾҪ���ղ��洢�� DMA �������е��ֽ�����
 */
void usart1_rx_dma_init(uint8_t *rx1_buf, uint16_t dma_buf_num)
{
    // ����idle�ж�
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    // ��������
    HAL_UART_Receive_DMA(&huart1, rx1_buf, dma_buf_num);
}

/**
 * @brief �ú������� USART1 ���輰������� DMA ���������ݡ�
 */
void usart1_rx_dma_unable(void)
{
    __HAL_UART_DISABLE(&huart1);
}

/**
 * @brief ������usart1_rx_dma_restart��ͨ�����ú����� UART �� DMA���Լ����� DMA ������������������� USART1 ���� DMA��
 *
 * @param dma_buf_num ������dma_buf_num����DMA�������п��Դ洢���������������
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
 * @brief �ú�����ʼ�� USART6 ������ʹ�� DMA �������ݲ����ÿ����жϡ�
 *
 * @param rx6_buf rx6_buf ������ָ�򽫴洢���յ������ݵĻ�������ָ�롣
 * @param dma_buf_num ������dma_buf_num����ʾҪ���ղ��洢�� DMA �������е��ֽ�����
 */
void usart6_rx_dma_init(uint8_t *rx6_buf, uint16_t dma_buf_num)
{
    // ����idle�ж�
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    // ��������
    HAL_UART_Receive_DMA(&huart6, rx6_buf, dma_buf_num);
}

/**
 * @brief fputc ����ʹ�� HAL_UART_Transmit ����ͨ�� UART �����ַ���
 *
 * @param ch ������ch����Ҫд���ļ����ַ������ǡ�int�����ͣ���Ϊ�����Ա�ʾ�ַ����ļ��������(EOF)��
 * @param f ������f����ָ�� FILE �ṹ��ָ�롣�ýṹ��ʾ�ļ����������й����ڷ��ʵ��ļ�����Ϣ��������λ�úͻ���״̬��
 * �ڱ����У�������ָ�������ڴ������ݵ� UART ���� (huart8)��
 *
 * @return д���ļ����ַ���
 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart8, (uint8_t *)&ch, 1, 0xffff);
    return ch;
}
