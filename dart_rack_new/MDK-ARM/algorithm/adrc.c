#include "adrc.h"
#include "user_lib.h"

/**
 * @brief 该函数根据给定的输入和条件计算变量的值。
 *
 * @param fp32 x1 所求解变量的初始值。
 * @param fp32 x2 参数x2代表第二输入值。
 * @param fp32 delta 参数“delta”表示用于数值微分的小值。它通常是一个小的正数，例如 0.001 或 0.0001。
 * @param fp32 h 参数“h”表示计算中使用的步长或时间间隔。它用于确定变量随时间的变化。
 *
 * @return fp32 类型的值。
 */
fp32 fst(fp32 x1, fp32 x2, fp32 delta, fp32 h)
{
    fp32 d  = delta * h;
    fp32 d0 = h * d;
    fp32 y  = x1 + h * x2;
    fp32 a0 = sqrt(pow(d, 2.0) + 8 * delta * fabsf(y));
    fp32 a  = fabsf(y) > d0 ? x2 + (a0 - d) / 2.0f * sign(y) : x2 + y / h;
    return fabsf(a) > d ? -delta * sign(a) : -delta * a / d;
}

/**
 * @brief 该函数初始化线性有源抗扰控制器 (LADRC) 的参数。
 *
 * @param ardc 指向 ladrc_type_def 类型结构的指针，其中包含 LADRC 算法的参数和变量。
 * @param LADRC LADRC 参数是一个包含 6 个浮点值的数组。这些值代表 LADRC（线性有源抗扰控制）结构初始化时使用的不同参数。
 * @param max_out “max_out”参数表示LADRC（线性有源抗扰控制）控制器可以产生的最大输出值。它用于将控制器的输出限制在特定范围内。
 *
 * @return 什么都没有（无效）。
 */
void LADRC_init(ladrc_type_def *ardc,
                const fp32 LADRC[6],
                fp32 max_out)
{
    if (ardc == NULL || ardc == NULL) {
        return;
    }
    /*  */
    ardc->r1              = 0.0f;
    ardc->r2              = 0.0f;
    ardc->sampling_period = LADRC[0];
    ardc->tracking_delta  = LADRC[1];

    /*  */
    ardc->w0            = LADRC[2];
    ardc->b             = LADRC[3];
    ardc->z1            = 0.0f;
    ardc->z2            = 0.0f;
    ardc->z3            = 0.0f;
    ardc->observe_beta1 = 3 * ardc->w0;
    ardc->observe_beta2 = 3 * pow(ardc->w0, 2.0f);
    ardc->observe_beta3 = pow(ardc->w0, 3.0f);

    /*  */
    ardc->k1 = LADRC[4];
    ardc->k2 = LADRC[5];

    /*  */
    ardc->max_out = max_out;
}

/**
 * @brief 该函数初始化 ADRC（主动抗扰控制）控制器的参数。
 *
 * @param ardc 指向需要初始化的ADRC结构的指针。
 * @param ADRC ADRC 是一个由 12 个浮点值组成的数组，表示 ADRC（主动抗扰控制）算法的参数。
 * @param max_out “max_out”参数是 ADRC（主动抗扰控制）控制器可以产生的最大输出值。它用于将控制器的输出限制在特定范围内。
 *
 * @return 该函数不返回任何值。它有一个 void 返回类型。
 */
void ADRC_init(adrc_type_def *ardc,
               const fp32 ADRC[12],
               fp32 max_out)
{
    if (ardc == NULL || ardc == NULL) {
        return;
    }
    /*  */
    ardc->r1              = 0.0f;
    ardc->r2              = 0.0f;
    ardc->sampling_period = ADRC[0];
    ardc->tracking_delta  = ADRC[1];

    /*  */
    ardc->w0             = ADRC[2];
    ardc->b              = ADRC[3];
    ardc->z1             = 0.0f;
    ardc->z2             = 0.0f;
    ardc->z3             = 0.0f;
    ardc->observe_delta  = ADRC[4];
    ardc->observe_beta1  = 3 * ardc->w0;
    ardc->observe_beta2  = 3 * pow(ardc->w0, 2.0f);
    ardc->observe_beta3  = pow(ardc->w0, 3.0f);
    ardc->observe_alpha1 = ADRC[5];
    ardc->observe_alpha2 = ADRC[6];

    /* */
    ardc->out_delta  = ADRC[7];
    ardc->out_alpha1 = ADRC[8];
    ardc->out_alpha2 = ADRC[9];
    ardc->out_beta1  = ADRC[10];
    ardc->out_beta2  = ADRC[11];

    /*  */
    ardc->max_out = max_out;
}

/**
 * @brief 该函数使用指定参数初始化级联 LADRC 控制器。
 *
 * @param c_adrc 指向cascade_ladrc_t类型结构的指针，其中包含LADRC控制器的两个实例（ladrc_outside和ladrc_inside）。
 * @param oADRC oADRC 是一个由 6 个浮点数组成的数组，表示外部 ADRC（主动抗扰控制）控制器的参数。
 * @param iADRC iADRC 参数是一个由 6 个浮点数组成的数组，表示内部 ADRC（主动抗扰控制）参数。
 * @param oADRC_max_out 外部 ADRC 控制器的最大输出值。
 * @param iADRC_max_out 参数“iADRC_max_out”表示内部 ADRC 控制器的最大输出值。
 */
void cascade_LADRC_init(cascade_ladrc_t *c_adrc,
                        const fp32 oADRC[6],
                        const fp32 iADRC[6],
                        fp32 oADRC_max_out,
                        fp32 iADRC_max_out)
{
    LADRC_init(&c_adrc->ladrc_outside, oADRC, oADRC_max_out);
    LADRC_init(&c_adrc->ladrc_inside, iADRC, iADRC_max_out);
}

/**
 * @brief 该函数根据参考值和设定值计算线性有源抗扰控制器 (LADRC) 的输出。
 *
 * @param ardc 指向“ladrc_type_def”类型结构的指针，其中包含 LADRC 计算中使用的参数和变量。
 * @param ref “ref”参数代表控制系统的参考值。它是系统应该达到或维持的期望值。
 * @param set “设置”参数表示控制系统所需的设定点或参考值。这是系统试图实现或维持的价值。
 *
 * @return `ardc->out` 的值，其类型为 `fp32`。
 */
fp32 LADRC_calc(ladrc_type_def *ardc, fp32 ref, fp32 set)
{
    if (ardc == NULL) {
        return 0.0f;
    }
    ardc->last_set = ardc->set;
    ardc->last_out = ardc->out;
    ardc->set      = set;
    ardc->fdb      = ref;

    /*  */
    ardc->r1 += ardc->sampling_period * ardc->r2;
    ardc->r2 += ardc->sampling_period * fst(ardc->r1 - set, ardc->r2, ardc->tracking_delta, ardc->sampling_period);

    ardc->e1 = ardc->r1 - ardc->z1;
    ardc->e2 = ardc->r2 - ardc->z2;

    /* */
    ardc->out = ardc->k1 * ardc->e1 + ardc->k2 * ardc->e2 - ardc->z3 / ardc->b;
    LimitMax(ardc->out, ardc->max_out);

    /*  */
    fp32 epsilon = ardc->z1 - ref;
    ardc->z1 += ardc->sampling_period * (ardc->z2 - ardc->observe_beta1 * epsilon);
    ardc->z2 += ardc->sampling_period * (ardc->z3 - ardc->observe_beta2 * epsilon + ardc->b * ardc->out);
    ardc->z3 += -ardc->sampling_period * ardc->observe_beta3 * epsilon;

    return ardc->out;
}

/**
 * @brief 该函数根据给定的输入和参数计算有源抗扰控制 (ADRC) 算法的输出。
 *
 * @param ardc 指向“adrc_type_def”类型结构的指针，其中包含 ADRC 计算中使用的各种参数和变量。
 * @param ref “ref”参数表示系统试图达到的参考值或期望值。
 * @param set “设置”参数表示控制系统所需的设定点或参考值。这是系统试图实现或维持的价值。
 *
 * @return fp32（浮点 32 位）类型的值，表示 ADRC（主动抗扰控制）计算的输出。
 */
fp32 ADRC_calc(adrc_type_def *ardc, fp32 ref, fp32 set)
{
    if (ardc == NULL) {
        return 0.0f;
    }
    ardc->last_set = ardc->set;
    ardc->last_out = ardc->out;
    ardc->set      = set;
    ardc->fdb      = ref;

    /*  */
    ardc->r1 += ardc->sampling_period * ardc->r2;
    ardc->r2 += ardc->sampling_period * fst(ardc->r1 - set, ardc->r2, ardc->tracking_delta, ardc->sampling_period);

    ardc->e1 = ardc->r1 - ardc->z1;
    ardc->e2 = ardc->r2 - ardc->z2;

    /*  */
    ardc->out = ardc->out_beta1 * fal(ardc->e1, ardc->out_alpha1, ardc->out_delta) +
                ardc->out_beta2 * fal(ardc->e2, ardc->out_alpha2, ardc->out_delta) -
                ardc->z3 / ardc->b;
    LimitMax(ardc->out, ardc->max_out);

    /*  */
    fp32 epsilon = ardc->z1 - ref;
    ardc->z1 += ardc->sampling_period * (ardc->z2 - ardc->observe_beta1 * epsilon);
    ardc->z2 += ardc->sampling_period * (ardc->z3 - ardc->observe_beta2 * fal(epsilon, ardc->observe_alpha1, ardc->observe_delta) + ardc->b * ardc->out);
    ardc->z3 += -ardc->sampling_period * ardc->observe_beta3 * fal(epsilon, ardc->observe_alpha2, ardc->observe_delta);

    return ardc->out;
}

/**
 * @brief 该函数根据给定的参考和设置计算级联 LADRC 控制器的输出。
 *
 * @param c_adrc 指向cascade_ladrc_t类型的结构体的指针，其中包含计算中使用的各种变量和结构体。
 * @param o_ref 级联LADRC控制器外环的参考输出值。
 * @param i_ref 参数“i_ref”表示输入参考值。
 * @param o_set 级联 LADRC 控制器所需的输出设定值。
 *
 * @return `c_adrc->out` 的值，其类型为 `fp32`。
 */
fp32 cascade_LADRC_calc(cascade_ladrc_t *c_adrc,
                        fp32 o_ref,
                        fp32 i_ref,
                        fp32 o_set)
{
    c_adrc->s_set             = o_set;
    c_adrc->s_ref             = o_ref;
    c_adrc->v_ref             = i_ref;
    c_adrc->ladrc_outside.fdb = o_ref;
    c_adrc->ladrc_inside.fdb  = i_ref;

    c_adrc->v_set = LADRC_calc(&c_adrc->ladrc_outside, c_adrc->s_ref, c_adrc->s_set);
    c_adrc->out   = LADRC_calc(&c_adrc->ladrc_inside, c_adrc->v_ref, c_adrc->v_set);

    return c_adrc->out;
}
