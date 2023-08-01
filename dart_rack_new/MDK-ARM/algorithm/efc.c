#include "efc.h"
#include "user_lib.h"

/**
 * @brief 该函数使用给定参数初始化 EFC（最速控制）结构。
 *
 * @param efc 指向 efc_type_def 类型的结构的指针。该结构可能包含与电子反馈控制系统相关的变量和设置。
 * @param kp kp 是控制系统中使用的比例增益参数。它确定对期望值和实际值之间的误差的响应强度。 kp 值越高，响应越强。
 * @param kd kd 是控制系统中微分增益的值。它用于确定响应误差信号变化的输出变化率。
 * @param max_out EFC（最速控制）可以产生的最大输出值。这用于将控制信号限制在一定范围内。
 *
 * @return 什么都没有（无效）。
 */
void EFC_Init(efc_type_def *efc, fp32 kp, fp32 kd, fp32 max_out)
{
    if (efc == NULL) {
        return;
    }
    efc->max_out = max_out;
    efc->Kp      = kp;
    efc->Kd      = kd;

    efc->out = 0;
}

/**
 * @brief 该函数根据参考值和设定值计算比例积分控制器（PI 控制器）的输出，并将输出限制为最大值。
 *
 * @param efc 指向 efc_type_def 类型的结构的指针。该结构可能包含与 EFC（电子流量控制器）系统相关的各种参数和变量。
 * @param ref “ref”参数是EFC（电子流量控制器）试图达到的参考值。它是一个浮点数 (fp32)，表示 EFC 反馈环路的所需值。
 * @param set “设置”参数表示控制系统所需的设定点或目标值。它是控制系统试图实现或维持的价值。
 *
 * @return 变量“efc->out”的值，其类型为“fp32”。
 */
fp32 EFC_PIO_calc(efc_type_def *efc, fp32 ref, fp32 set)
{
    efc->set = set;
    efc->fdb = ref;

    efc->out = efc->Kp * EFPI(set) - EFLO(ref);
    LimitMax(efc->out, efc->max_out);

    return efc->out;
}

/**
 * @brief 该函数使用比例增益和微分增益，根据参考值和设定值计算 PID 控制器的输出。
 *
 * @param efc 指向 efc_type_def 类型的结构的指针。该结构可能包含与 EFC（电子流量控制器）系统相关的变量和参数。
 * @param ref “ref”参数是控制器试图达到的参考值或设定点。
 * @param set “设置”参数表示控制系统所需的设定点或参考值。这是系统试图实现或维持的价值。
 *
 * @return 变量“efc->out”的值，其类型为 fp32（浮点 32 位）。
 */
fp32 EFC_PID_calc(efc_type_def *efc, fp32 ref, fp32 set)
{
    efc->set = set;
    efc->fdb = ref;

    efc->out = efc->Kp * EFPI(set) - efc->Kd * EFPD(ref);
    LimitMax(efc->out, efc->max_out);

    return efc->out;
}

/**
 * @brief EFPI 函数计算最速PI控制器
 *
 * @param z 参数“z”是单精度浮点数（fp32）。
 *
 * @return 浮点值 (fp32)。
 */
fp32 EFPI(fp32 z)
{
    return (pow(z, 3.0f) - 2.582f * pow(z, 2.0f) + 2.222f * z - 0.6376f) /
           (pow(z, 3.0f) - 2.633 * pow(z, 2.0f) + 2.304f * z - 0.6703f);
}

/**
 * @brief 函数 EFPD 函数计算最速PD控制器
 *
 * @param z 参数“z”是单精度浮点数（fp32）。
 *
 * @return 使用给定公式计算的浮点值 (fp32)。
 */
fp32 EFPD(fp32 z)
{
    return (2.0f * pow(z, 3.0f) - 5.214f * pow(z, 2.0f) + 4.524f * z - 1.307f) /
           (pow(z, 3.0f) - 2.582f * pow(z, 2.0f) + 2.222f * z - 0.6376f);
}

/**
 * @brief 函数 EFLO 计算最速观测器
 *
 * @param z 参数“z”表示浮点数。
 *
 * @return fp32 类型的值（浮点 32 位）
 */
fp32 EFLO(fp32 z)
{
    return (0.007643f * pow(z, 3.0f) - 0.01974f * pow(z, 2.0f) + 0.01699f * z - 0.004874f) /
           (pow(z, 4.0f) - 3.188f * pow(z, 3.0f) + 3.786f * pow(z, 2.0) - 1.984f * z + 0.3864f);
}
