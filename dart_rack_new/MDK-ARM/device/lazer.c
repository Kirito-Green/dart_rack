#include "lazer.h"

/**
 * 函数“lazer_on”通过向相应的 GPIO 引脚写入低信号来打开激光器。
 */
void lazer_on(void)
{
    HAL_GPIO_WritePin(GPIO_LAZER_PORT, GPIO_LAZER_PIN, GPIO_PIN_RESET);
}

/**
 * 函数 `lazer_off` 通过将相应的 GPIO 引脚设置为高电平来关闭激光。
 */
void lazer_off(void)
{
    HAL_GPIO_WritePin(GPIO_LAZER_PORT, GPIO_LAZER_PIN, GPIO_PIN_SET);
}
