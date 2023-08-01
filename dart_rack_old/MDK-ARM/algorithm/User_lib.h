#ifndef USER_LIB_H
#define USER_LIB_H

#include "struct_typedef.h"
#include "arm_math.h"
#include "arm_const_structs.h"

/* 限幅函数 */
#define LimitMax(input, max)       \
    {                              \
        if (input > max) {         \
            input = max;           \
        } else if (input < -max) { \
            input = -max;          \
        }                          \
    }

/* 双边不等限幅 */
#define constrain(val, min_val, max_val)  \
    {                                     \
        if (val > max_val) val = max_val; \
        if (val < min_val) val = min_val; \
    }

/* 循环限幅 */
#define loop_constrain(val, min_val, max_val) \
    {                                         \
        fp32 dis = max_val - min_val;         \
        while (val < min_val) val += dis;     \
        while (val > max_val) val -= dis;     \
    }

extern bool_t is_zero(fp32 val);
extern int16_t sign(fp32 val);
extern int16_t step_function(fp32 val, fp32 JudgePoint);
extern fp32 choose_shortest_path(fp32 set, fp32 ref, fp32 val_range);
extern fp32 fal(fp32 error, fp32 alpha, fp32 delta);
extern bool_t in_range(fp32 val, fp32 min, fp32 max);
extern int16_t int_pow(int16_t val, uint8_t times);
extern void fp32_to_string(fp32 val, uint8_t digit, uint8_t *target_rx_buf, uint8_t *target_rx_len);

#endif
