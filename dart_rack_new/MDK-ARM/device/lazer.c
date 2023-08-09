#include "lazer.h"

/**
 * ������lazer_on��ͨ������Ӧ�� GPIO ����д����ź����򿪼�������
 */
void lazer_on(void)
{
    HAL_GPIO_WritePin(GPIO_LAZER_PORT, GPIO_LAZER_PIN, GPIO_PIN_RESET);
}

/**
 * ���� `lazer_off` ͨ������Ӧ�� GPIO ��������Ϊ�ߵ�ƽ���رռ��⡣
 */
void lazer_off(void)
{
    HAL_GPIO_WritePin(GPIO_LAZER_PORT, GPIO_LAZER_PIN, GPIO_PIN_SET);
}
