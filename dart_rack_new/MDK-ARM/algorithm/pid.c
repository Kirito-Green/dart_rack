/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note
  * @history    2021-7-23   增加了串级pid的函数和接口
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  V2.0.0     Jly-23-2021     Qylann          1. 增加部分函数
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "pid.h"
#include "main.h"
#include "user_lib.h"

/**
 * @brief 该函数使用给定参数初始化 PID 控制器。
 *
 * @param pid 指向需要初始化的PID结构的指针。
 * @param mode “mode”参数用于指定PID控制器的工作模式。根据具体的实现，它可以具有不同的值，但通常它用于在不同的控制模式之间进行选择，例如位置控制、速度控制或其他控制策略。
 * @param PID PID 数组分别包含比例增益、积分增益和微分增益。这些增益用于计算 PID 控制器的输出。
 * @param input_alpha input_alpha 参数是一个浮点值，表示 PID 控制器中用于过滤输入信号的 alpha 值。它确定当前输入值与先前输入值相比的权重。
 * input_alpha 的值越高，当前输入值的权重就越大，从而导致速度更快
 * @param output_alpha output_alpha 参数是一个浮点值，表示用于过滤 PID 控制器输出的 alpha 值。它用于平滑输出信号的变化。
 * @param delta PID_init 函数中的“delta”参数是一个浮点值，表示非线性误差处理的误差限度。该值越大截止频率变大，带宽变宽，跟踪速度变快
 * 改值越小截止频率变小，带宽窄，跟踪速度变快
 * @param alpha alpha参数是用于调整非线性误差处理的表示非线性误差处理中幂指数，幂指数小于1时误差大增益小误差小增益大；
 * 幂指数大于1是误差小增益小 误差大增益大
 * @param max_out PID控制器可以产生的最大输出值。
 * @param max_iout 参数“max_iout”表示PID控制器积分项的最大允许值。它限制积分项的累积，防止积分项变得太大而导致系统不稳定或超调。
 * @param dead_area dead_area 参数表示所需设定点周围的范围，其中误差被认为可以忽略不计，并且不采取任何控制操作。
 *
 * @return 什么都没有（无效）。
 */
void PID_init(pid_type_def *pid,
              uint8_t mode,
              const fp32 PID[3],
              fp32 input_alpha,
              fp32 output_alpha,
              fp32 delta, fp32 alpha,
              fp32 max_out,
              fp32 max_iout,
              fp32 dead_area)
{
    if (pid == NULL || PID == NULL) {
        return;
    }

    pid->dead_area        = dead_area;
    pid->dead_area_stable = dead_area;
    pid->i_alpha          = input_alpha;
    pid->o_alpha          = output_alpha;
    pid->delta            = delta;
    pid->alpha            = alpha;
    pid->mode             = mode;
    pid->Kp               = PID[0];
    pid->Ki               = PID[1];
    pid->Kd               = PID[2];
    pid->out              = 0;
    pid->max_out          = max_out;
    pid->max_iout         = max_iout;
    // pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    // pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
 * @brief 该函数使用指定参数初始化级联 PID 控制器。
 *
 * @param c_pid 指向将被初始化的cascade_pid_t结构的指针。
 * @param oPID oPID 参数是三个浮点值的数组，表示外部 PID 控制器的比例增益、积分增益和微分增益。
 * @param iPID “iPID”参数是一个由三个浮点值组成的数组，表示内部 PID 控制器的比例增益、积分增益和微分增益。
 * @param o_input_alpha 参数“o_input_alpha”是用于过滤外部PID控制器输入的alpha值。它确定当前输入值与先前输入值相比的权重。较高的 alpha
 * 值将为当前输入值赋予更大的权重，从而导致更快的响应，但可能
 * @param o_output_alpha 参数“o_output_alpha”是用于外部 PID 控制器输出的 alpha 值。它确定控制器输出响应误差而变化的速率。较高的 alpha
 * 值将导致更快的响应，但也可能导致超调和不稳定。
 * @param i_input_alpha 参数“i_input_alpha”是用于内部PID控制器输入的alpha值。它决定内部 PID 控制器中输入值的更新速率。
 * @param i_output_alpha 参数“i_output_alpha”是用于内部PID控制器输出的alpha值。它决定内部 PID 控制器输出变化的速率。
 * “i_output_alpha”的值越高，内部 PID 控制器的输出响应越快。
 * @param o_delta 函数“cascade_PID_init”中的参数“o_delta”表示外部PID控制器表示非线性误差处理的误差限度。该值越大截止频率变大，带宽变宽，跟踪速度变快
 * 改值越小截止频率变小，带宽窄，跟踪速度变快
 * @param o_alpha 函数“cascade_PID_init”中的参数“o_alpha”用于设置外部PID控制器的alpha值。 alpha
 * 值确定非线性误差处理中幂指数，幂指数小于1时误差大增益小误差小增益大；幂指数大于1是误差小增益小 误差大增益大
 * @param i_delta 函数“cascade_PID_init”中的参数“i_delta”表示内部PID控制器表示非线性误差处理的误差限度。该值越大截止频率变大，带宽变宽，跟踪速度变快
 * 改值越小截止频率变小，带宽窄，跟踪速度变快
 * @param i_alpha 参数“i_alpha”是用于内部PID控制器积分项的alpha值。它确定非线性误差处理中幂指数，幂指数小于1时误差大增益小误差小增益大；
 * 幂指数大于1是误差小增益小 误差大增益大
 * @param oPID_max_out 参数“oPID_max_out”表示外部 PID 控制器的最大输出值。它限制控制器可以作为控制信号输出的最大值。
 * @param oPID_max_iout 参数“oPID_max_iout”表示外部PID控制器的最大积分输出值。它限制积分项可以达到的最大值，防止过度饱和。
 * @param iPID_max_out 参数“iPID_max_out”表示内部PID控制器的最大输出值。它限制了内部PID控制器可以输出的最大值。
 * @param iPID_max_iout 参数“iPID_max_iout”表示内部PID控制器的最大积分输出限制。它限制了内部PID控制器的积分项可以达到的最大值。
 * @param o_dead_area
 * 参数“o_dead_area”指的是外部PID控制器输出中的死区或死区。它是设定点周围的一个范围，在该范围内控制器输出被视为零。该死区内的任何错误都不会导致控制器输出发生任何变化。
 * @param i_dead_area 参数“i_dead_area”表示内部PID控制器的死区。死区是控制器不响应或产生任何输出的输入值范围。它用于防止输入信号中的微小波动或噪声引起不必要的控制动作。
 */
void cascade_PID_init(cascade_pid_t *c_pid,
                      const fp32 oPID[3], const fp32 iPID[3],
                      fp32 o_input_alpha, fp32 o_output_alpha,
                      fp32 i_input_alpha, fp32 i_output_alpha,
                      fp32 o_delta, fp32 o_alpha,
                      fp32 i_delta, fp32 i_alpha,
                      fp32 oPID_max_out, fp32 oPID_max_iout,
                      fp32 iPID_max_out, fp32 iPID_max_iout,
                      fp32 o_dead_area, fp32 i_dead_area)
{
    PID_init(&c_pid->pid_outside, PID_POSITION, oPID, o_input_alpha, o_output_alpha, o_delta, o_alpha, oPID_max_out, oPID_max_iout, o_dead_area);
    PID_init(&c_pid->pid_inside, PID_POSITION, iPID, i_input_alpha, i_output_alpha, i_delta, i_alpha, iPID_max_out, iPID_max_iout, i_dead_area);
}

/**
 * @brief 该函数根据参考值、设定值和当前值计算 PID 控制器的输出。
 *
 * @param pid 指向 pid_type_def 类型结构的指针，其中包含 PID 控制器参数和变量。
 * @param ref “ref”参数表示系统试图达到的参考值或设定点。
 * @param set “设置”参数表示受控系统的期望设定点或目标值。这是系统试图达到或维持的价值。
 * @param val_range 参数“val_range”表示误差可以取的值的范围。在功能中用于选择设定值和参考值之间的最短路径。
 *
 * @return fp32（浮点 32 位）类型的值，表示 PID 控制器的输出。
 */
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set, fp32 val_range)
{
    if (pid == NULL) {
        return 0.0f;
    }
    pid->last_set = pid->set;
    pid->last_out = pid->out;
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set      = set;

#ifdef INPUT_FILTER_FIRSET_ORDER
    pid->set = pid->i_alpha * pid->last_set + (1 - pid->i_alpha) * pid->set; // 低通滤波缓启动
#endif

    pid->fdb      = ref;
    pid->error[0] = set - ref;
    if (val_range > 0)
        pid->error[0] = choose_shortest_path(set, ref, val_range); // 最短路径选择
    if (pid->mode == PID_POSITION) {
#ifdef PID_NON_LINEAR_OUTPUT
        pid->error[0] = fal(pid->error[0], pid->alpha, pid->delta); // 非线性计算
#endif

        pid->Pout = pid->Kp * pid->error[0];

#ifdef PID_I_DISCREATE
        if (fabsf(pid->error[0]) <= I_CONTROL_DISCTEATE) { // 积分分离
            pid->Iout += pid->Ki * pid->error[0];
        } else {
            pid->Iout = 0;
        }
#else
        pid->Iout += pid->Ki * pid->error[0];
#endif

        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout    = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout; // 线性输出

#ifdef OUTPUT_FILTER_FIRST_ORDER
        pid->out = pid->o_alpha * pid->last_out + (1 - pid->o_alpha) * pid->out; // 低通滤波输出
#endif

        if (fabsf(pid->error[0]) <= pid->dead_area) { // 死区不输出省电
            pid->out = ZERO_POINT;
        }
        LimitMax(pid->out, pid->max_out);
    } else if (pid->mode == PID_DELTA) {
#ifdef PID_NON_LINEAR_OUTPUT
        pid->error[0] = fal(pid->error[0], pid->alpha, pid->delta);
#endif
        pid->Pout    = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout    = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);

#ifdef PID_D_DISCREATE
        if (fabsf(pid->error[0]) - fabsf(pid->error[1]) > D_CONTROL_POINT) { // D控制分离
            pid->Dout = pid->Kd * pid->Dbuf[0];
        } else {
            pid->Dout = 0;
        }
#else
        pid->Dout = pid->Kd * pid->Dbuf[0];
#endif

        pid->out = pid->Pout + pid->Iout + pid->Dout;

#ifdef OUTPUT_FILTER_FIRST_ORDER
        pid->out = pid->o_alpha * pid->last_out + (1 - pid->o_alpha) * pid->out; // 低通滤波输出
#endif

        if (fabsf(pid->error[0]) <= pid->dead_area) { // 死区不输出省电
            pid->out = ZERO_POINT;
        }
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

/**
 * @brief 该函数根据给定的参考值和范围计算级联 PID 控制器的输出。
 *
 * @param c_pid 指向cascade_pid_t类型结构的指针，其中包含级联PID控制器所需的变量。
 * @param o_ref 串级PID控制器外环的参考值。它代表受控系统的期望输出值。
 * @param i_ref 参数“i_ref”表示内部 PID 控制器的参考输入。这是内环控制的期望值。
 * @param o_set 外部 PID 环路所需的输出设定值。
 * @param o_range 参数“o_range”表示输出变量“o_set”的范围。它在 PID 计算中用于确定控制器的比例项、积分项和微分项。
 * @param i_range 参数“i_range”表示输入变量的范围。用于内部PID控制器的计算。
 *
 * @return 变量“c_pid->out”的值，其类型为 fp32。
 */
fp32 cascade_PID_calc(cascade_pid_t *c_pid, fp32 o_ref, fp32 i_ref, fp32 o_set, fp32 o_range, fp32 i_range)
{
    c_pid->s_set = o_set;
    c_pid->s_fdb = o_ref;
    c_pid->v_fdb = i_ref;

    c_pid->v_set = PID_calc(&c_pid->pid_outside, c_pid->s_fdb, c_pid->s_set, o_range);
    if (is_zero(c_pid->v_set)) { // dead area
        c_pid->out                   = ZERO_POINT;
        c_pid->pid_outside.dead_area = 2.0f * c_pid->pid_outside.dead_area_stable;
    } else {
        c_pid->out                   = PID_calc(&c_pid->pid_inside, c_pid->v_fdb, c_pid->v_set, i_range);
        c_pid->pid_outside.dead_area = c_pid->pid_outside.dead_area_stable;
    }
    return c_pid->out;
}

/**
 * @brief 函数“PID_clear”清除 PID 控制器结构中各种变量的值。
 *
 * @param pid 指向 pid_type_def 类型的结构的指针。
 *
 * @return 如果“pid”参数为“NULL”，函数将返回而不执行任何操作。
 */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

/**
 * @brief 函数“cascade_PID_clear”清除级联 PID 结构内的 PID 控制器。
 *
 * @param c_pid c_pid 是一个指向cascade_pid_t 类型的结构的指针。
 */
void cascade_PID_clear(cascade_pid_t *c_pid)
{
    PID_clear(&c_pid->pid_outside);
    PID_clear(&c_pid->pid_inside);
}
