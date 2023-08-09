#include "combination.h"
#include "setting.h"
#include "pid.h"
#include "adrc.h"
#include "efc.h"

/**
 * @brief 该函数使用指定参数初始化级联 PID-LADRC 控制器。
 *
 * @param c_pid_ladrc 指向将被初始化的cascade_pid_ladrc_t结构的指针。
 * @param oPID oPID 参数是一个由三个浮点值组成的数组，表示外部 PID 控制器的系数。这三个值分别对应PID控制器的比例增益、积分增益和微分增益。
 * @param iADRC iADRC 是一个包含 6 个浮点值的数组，表示内部 ADRC(主动抗扰控制)控制器的参数。这些参数用于调整内部控制器并确定其行为。
 * @param o_init_target 外部 PID 控制器的初始目标值。
 * @param o_input_alpha 参数“o_input_alpha”是用于 PID 控制器中输入滤波器的 alpha 值。它确定当前输入值与先前输入值相比的权重。较高的 alpha
 * 值赋予当前输入值更大的权重，从而导致更快的响应，但可能产生更多噪声
 * @param o_output_alpha 参数“o_output_alpha”是PID控制器的输出滤波器系数。它确定 PID 控制器的输出响应误差信号变化而变化的速率。
 * “o_output_alpha”值越高，PID 控制器的输出响应越快
 * @param o_delta 函数“cascade_PID_LADRC_init”中的参数“o_delta”用于设置外部PID控制器的误差限度值，该值决定了PID误差非线性双端处理的边界值
 * @param o_alpha 函数“cascade_PID_LADRC_init”中的参数“o_alpha”用于设置外部PID控制器的alpha参数值。 alpha 参数决定了 PID非线性处理弯曲程度
 * @param oPID_max_out 外部 PID 控制器的最大输出值。
 * @param oPID_max_iout oPID_max_iout 是外部 PID 控制器的最大输出值。该参数限制外部PID控制器可以输出的最大值。
 * @param iADRC_max_out 参数“iADRC_max_out”表示内部 ADRC 控制器的最大输出值。它用于将内部控制器的输出限制在特定范围内。
 * @param o_dead_area
 * “cascade_PID_LADRC_init”函数中的“o_dead_area”参数用于指定PID控制器输出的死区。死区是控制器输出为零的设定值周围的范围。它有助于防止设备周围出现小幅振荡
 */
void cascade_PID_LADRC_init(cascade_pid_ladrc_t *c_pid_ladrc,
                            const fp32 oPID[3],
                            const fp32 iADRC[6],
                            fp32 o_init_target,
                            fp32 o_input_alpha,
                            fp32 o_output_alpha,
                            fp32 o_delta,
                            fp32 o_alpha,
                            fp32 oPID_max_out,
                            fp32 oPID_max_iout,
                            fp32 iADRC_max_out,
                            fp32 o_dead_area)
{
    PID_init(&c_pid_ladrc->pid_outside, PID_POSITION, oPID, o_input_alpha, o_output_alpha, o_delta, o_alpha, oPID_max_iout, oPID_max_out, o_dead_area);
    LADRC_init(&c_pid_ladrc->ladrc_inside, iADRC, iADRC_max_out);
}

/**
 * @brief 该函数根据给定的参考值和设置计算级联 PID-LADRC 控制器的输出。
 *
 * @param c_pid_ladrc 指向cascade_pid_ladrc_t类型的结构体的指针，其中包含计算中使用的各种变量和结构体。
 * @param o_ref 级联PID-LADRC控制器外环的参考值。它代表受控系统的期望输出值。
 * @param i_ref 参数“i_ref”表示级联 PID-LADRC 控制器内环的参考输入。这是内循环跟踪的期望值。
 * @param o_set 系统所需的输出值。
 *
 * @return `c_pid_ladrc->out` 的值，其类型为 `fp32`。
 */
fp32 cascade_PID_LADRC_calc(cascade_pid_ladrc_t *c_pid_ladrc,
                            fp32 o_ref,
                            fp32 i_ref,
                            fp32 o_set)
{
    c_pid_ladrc->s_set            = o_set;
    c_pid_ladrc->s_ref            = o_ref;
    c_pid_ladrc->v_ref            = i_ref;
    c_pid_ladrc->pid_outside.fdb  = o_ref;
    c_pid_ladrc->ladrc_inside.fdb = i_ref;

    c_pid_ladrc->v_set = PID_calc(&c_pid_ladrc->pid_outside, c_pid_ladrc->s_ref, c_pid_ladrc->s_set, CIRCLE_ANGLE);
    c_pid_ladrc->out   = LADRC_calc(&c_pid_ladrc->ladrc_inside, c_pid_ladrc->v_ref, c_pid_ladrc->v_set);

    return c_pid_ladrc->out;
}
