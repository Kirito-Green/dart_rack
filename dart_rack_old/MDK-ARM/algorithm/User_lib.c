#include "User_lib.h"

bool_t is_zero(fp32 val)
{
    return fabsf(val) < (fp32)1e-4 ? 1 : 0;
}

int16_t sign(fp32 val)
{
    if (val < 0.0f) return -1;
    if (val > 0.0f) return 1;
    return 0;
}

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

fp32 fal(fp32 error, fp32 alpha, fp32 delta)
{
    return fabsf(error) > delta ? pow(fabsf(error), alpha) * sign(error) : error / pow(delta, 1 - alpha);
}

bool_t in_range(fp32 val, fp32 min, fp32 max)
{
    if (val >= min && val <= max) return 1;
    return 0;
}

/* 整数幂 */
int16_t int_pow(int16_t val, uint8_t times)
{
    if (times == 1)
        return val;
    return val * int_pow(val, times - 1);
}

static uint8_t rx_buf[20];
static uint8_t rx_len;
/* 浮点型转字符串 */
void fp32_to_string(fp32 val, uint8_t digit, uint8_t *target_rx_buf, uint8_t *target_rx_len)
{
    uint32_t temp_val = roundf(val * int_pow(10, digit));
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
