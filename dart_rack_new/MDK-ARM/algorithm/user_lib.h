#ifndef USER_LIB_H
#define USER_LIB_H

#include "struct_typedef.h"
#include "arm_math.h"
#include "arm_const_structs.h"

#define SYSTICK_PERIOID 22
#define TIM_PERIOD      1
/* 锟睫凤拷锟斤拷锟斤拷 */
#define LimitMax(input, max)       \
    {                              \
        if (input > max) {         \
            input = max;           \
        } else if (input < -max) { \
            input = -max;          \
        }                          \
    }

/* 双锟竭诧拷锟斤拷锟睫凤拷 */
#define constrain(val, min_val, max_val)  \
    {                                     \
        if (val > max_val) val = max_val; \
        if (val < min_val) val = min_val; \
    }

/* 循锟斤拷锟睫凤拷 */
#define loop_constrain(val, min_val, max_val) \
    {                                         \
        fp32 dis = max_val - min_val;         \
        while (val < min_val) val += dis;     \
        while (val > max_val) val -= dis;     \
    }

extern fp32 get_max(fp32 v1, fp32 v2);
extern bool_t is_zero(fp32 val);
extern int16_t sign(fp32 val);
extern int16_t step_function(fp32 val, fp32 JudgePoint);
extern fp32 choose_shortest_path(fp32 set, fp32 ref, fp32 val_range);
extern fp32 fal(fp32 error, fp32 alpha, fp32 delta);
extern bool_t in_range(fp32 val, fp32 min, fp32 max);
extern bool_t get_close(fp32 val, fp32 target, fp32 range);
extern int16_t int_pow(int16_t val, uint8_t times);
extern void fp32_to_string(fp32 val, uint8_t digit, uint8_t *target_rx_buf, uint8_t *target_rx_len);
extern void systick_delay_us(uint16_t us);
extern void systick_delay_ms(uint32_t ms);
extern void htim_delay_us(uint16_t us);
extern void htim_delay_ms(uint32_t ms);
extern void mcu_delay_us(uint16_t us);
extern void mcu_delay_ms(uint16_t ms);

#endif
