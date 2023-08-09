#include "user_lib.h"
#include "tim.h"
// #include "stm32f4xx_hal_tim.h"

/**
 * 函数“get_max”返回两个给定浮点数之间的最大值。
 *
 * @param v1 第一个参数 v1 是单精度浮点数 (fp32)。
 * @param v2 参数v2是单精度浮点数（fp32）。
 *
 * @return v1 和 v2 之间的最大值。
 */
fp32 get_max(fp32 v1, fp32 v2)
{
    return v1 > v2 ? v1 : v2;
}

/**
 * @brief 函数“is_zero”检查浮点值是否在一定容差内接近零。
 *
 * @param val 参数“val”的类型为单精度浮点数。
 *
 * @return 布尔值 (bool_t)，指示给定值 (val) 是否被视为零。
 */
bool_t is_zero(fp32 val)
{
    return fabsf(val) < (fp32)1e-4 ? 1 : 0;
}

/**
 * @brief 判断浮点数符号
 *
 * @param val 参数“val”的类型为32 位精度的浮点数的自定义数据类型。
 *
 * @return int16_t 值，它是有符号的 16 位整数。如果输入值小于-1e-4，则该函数返回-1；如果输入值大于1e-4，则返回1；否则返回0。
 */
int16_t sign(fp32 val)
{
    if (val < -1e-4)
        return -1;
    if (val > 1e-4)
        return 1;
    return 0;
}

/**
 * @brief 阶跃函数
 *
 * @param val 变量“val”的类型为fp32，通常代表32位浮点数。它表示需要与“JudgePoint”参数进行比较的值。
 * @param JudgePoint JudgePoint 参数是一个浮点值，表示阈值。 step_function 函数将 val 参数与 JudgePoint 值进行比较，如果 val 小于或等于
 * JudgePoint，则返回 0，否则返回 1。
 *
 * @return int16_t 值。
 */
int16_t step_function(fp32 val, fp32 JudgePoint)
{
    return val <= JudgePoint ? 0 : 1;
}

fp32 choose_shortest_path(fp32 set, fp32 ref, fp32 val_range)
{
    fp32 front_dis = set - ref;
    int16_t s      = sign(front_dis);
    if (s != 0) {
        fp32 back_dis = (val_range - fabsf(set - ref)) * (-s);
        return fabsf(front_dis) < fabsf(back_dis) ? front_dis : back_dis;
    }
    return front_dis;
}

/**
 * @brief 非线性处理函数
 *
 * @param error 用于处理的误差
 * @param alpha 幂指数，幂指数小于1时误差大增益小误差小增益大；
 * 幂指数大于1是误差小增益小 误差大增益大
 * @param delta 该值越大截止频率变大，带宽变宽，跟踪速度变快
 * 改值越小截止频率变小，带宽窄，跟踪速度变快
 *
 * @return fp32值
 */
fp32 fal(fp32 error, fp32 alpha, fp32 delta)
{
    return fabsf(error) > delta ? pow(fabsf(error), alpha) * sign(error) : error / pow(delta, 1 - alpha);
}

/**
 * @brief 该函数检查给定值是否在指定范围内。
 *
 * @param val 您要检查的值是否在范围内。
 * @param min 输入值应在范围内的最小值。
 * @param max “max”参数是“val”参数可以具有的最大值，以便函数返回 true。
 *
 * @return 布尔值，1 或 0。
 */
bool_t in_range(fp32 val, fp32 min, fp32 max)
{
    if (val >= min && val <= max)
        return 1;
    return 0;
}

/**
 * @brief 函数“get_close”检查给定值是否在目标值的指定范围内。
 *
 * @param val 您要检查是否接近目标值的值。
 * @param target 我们要与给定值“val”进行比较的目标值。
 * @param range range 参数表示可接受的范围，在该范围内，val 和目标值之间的差异被认为是接近的。
 *
 * @return 一个布尔值。如果“val”和“target”之间的绝对差小于“range”，则返回 1，否则返回 0。
 */
bool_t get_close(fp32 val, fp32 target, fp32 range)
{
    if (fabsf(val - target) < range) {
        return 1;
    }
    return 0;
}

/**
 * @brief 函数 int_pow 通过将给定值递归地与自身相乘指定次数来计算给定值的幂。
 *
 * @param val 将求幂的基值。
 * @param times “times”参数的类型为uint8_t，这意味着它是一个无符号8位整数。它表示该值应与其自身相乘的次数。
 *
 * @return 将值“val”提高到“times”次方的结果。
 */
int16_t int_pow(int16_t val, uint8_t times)
{
    if (times == 0)
        return 1;
    return val * int_pow(val, times - 1);
}

static uint8_t rx_buf[20];
static uint8_t rx_len;
/**
 * @brief 函数“fp32_to_string”将浮点数转换为具有指定小数位数的字符串表示形式。
 *
 * @param val “val”参数是一个要转换为字符串的浮点数。
 * @param digit 参数“digit”表示转换后的字符串中包含的小数位数。
 * @param target_rx_buf 指向将存储结果字符串的缓冲区的指针。
 * @param target_rx_len 参数“target_rx_len”是指向“uint8_t”变量的指针，该变量将存储结果字符串的长度。
 */
void fp32_to_string(fp32 val, uint8_t digit, uint8_t *target_rx_buf, uint8_t *target_rx_len)
{
    uint16_t temp_val = roundf(val * int_pow(10, digit));
    rx_len            = 0;
    /* int to string */
    while (temp_val) {
        rx_buf[rx_len] = (uint8_t)(temp_val % 10);
        temp_val /= 10;
        rx_len++;
    }
    /* reverse */
    uint8_t temp_save;
    for (uint8_t i = 0; i < rx_len / 2; i++) {
        temp_save              = rx_buf[i];
        rx_buf[i]              = rx_buf[rx_len - i - 1];
        rx_buf[rx_len - i - 1] = temp_save;
    }
    memcpy(&rx_buf[rx_len - digit + 1], &rx_buf[rx_len - digit], digit);
    rx_buf[rx_len - digit] = (uint8_t)'.';
    rx_len++;

    /* 传输结果 */
    memcpy(target_rx_buf, &rx_buf, rx_len + 1);
    memcpy(target_rx_len, &rx_len, 1);
}

/**
 * @brief 函数“systick_delay_us”用于使用 SysTick 计时器引入微秒级的延迟。
 *
 * @param us 参数“us”表示以微秒为单位的延迟时间。
 */
void systick_delay_us(uint16_t us)
{
    uint32_t temp;
    SysTick->LOAD = SYSTICK_PERIOID * us;
    SysTick->VAL  = 0x00; // clear timer
    SysTick->CTRL = 0x01; // enable
    do {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && (!(temp & (1 << 16))));

    SysTick->VAL  = 0x00;
    SysTick->CTRL = 0x00;
}

/**
 * @brief 函数 systick_delay_ms 用于使用 SysTick 计时器创建以毫秒为单位的延迟。
 *
 * @param ms 参数“ms”表示延迟的毫秒数。
 */
void systick_delay_ms(uint32_t ms)
{
    uint32_t temp;
    SysTick->LOAD = 1000 * SYSTICK_PERIOID * ms;
    SysTick->VAL  = 0x00; // clear timer
    SysTick->CTRL = 0x01; // enable
    do {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && (!(temp & (1 << 16))));

    SysTick->VAL  = 0x00;
    SysTick->CTRL = 0x00;
}

/**
 * @brief 函数 htim_delay_us 用于使用 TIM3 定时器创建微秒级的延迟。
 *
 * @param us 参数“us”表示延迟的微秒数。
 */
void htim_delay_us(uint16_t us)
{

    htim3.Instance->CNT = us - 1;                            // 向计数器装要递减的数，减到0后会触发定时器的TIM_FLAG_UpDate标志位
    htim3.Instance->CR1 |= TIM_CR1_CEN;                      // 使能计数器， 计数器开始递减
    while ((htim3.Instance->SR & TIM_FLAG_UPDATE) != SET) {} // 等到计数器减到0
    htim3.Instance->CR1 &= (~TIM_CR1_CEN);                   // 关闭计数器
    htim3.Instance->SR &= ~TIM_FLAG_UPDATE;                  // 清除定时器变为0的标志位
}

/**
 * @brief 函数 htim_delay_ms 将程序执行延迟指定的毫秒数。
 *
 * @param ms 参数“ms”的类型为uint32_t，代表无符号32位整数。它表示延迟的毫秒数。
 */
void htim_delay_ms(uint32_t ms)
{
    htim_delay_us(1000 * ms);
}

/**
 * @brief 函数“mcu_delay_us”用于通过执行一系列无操作指令来引入微秒级的延迟。
 *
 * @param us 参数“us”表示延迟的微秒数。
 */
void mcu_delay_us(uint16_t us)
{
    for (uint16_t i = 0; i < us; i++) {
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
    }
}

/**
 * @brief 函数“mcu_delay_ms”是一个延迟函数，通过多次调用另一个函数“mcu_delay_us”来等待指定的毫秒数。
 *
 * @param ms 参数“ms”的类型为uint16_t，代表无符号16位整数。它表示应执行延迟的毫秒数。
 */
void mcu_delay_ms(uint16_t ms)
{
    for (uint16_t i = 0; i < 1000; i++) {
        mcu_delay_us(ms);
    }
}
