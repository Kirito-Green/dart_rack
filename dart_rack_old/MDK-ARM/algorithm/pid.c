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
#include "User_lib.h"

/**
 * @brief          pid struct data init
 * @param[out]     pid: PID struct data point
 * @param[in]      mode: PID_POSITION: normal pid
 *                 PID_DELTA: delta pid
 * @param[in]      PID: 0: kp, 1: ki, 2:kd
 * @param[in]      max_out: pid max out
 * @param[in]      max_iout: pid max iout
 * @retval         none
 */
/**
 * @brief          pid struct data init
 * @param[out]     pid: PID结构数据指针
 * @param[in]      mode: PID_POSITION:普通PID
 *                 PID_DELTA: 差分PID
 * @param[in]      PID: 0: kp, 1: ki, 2:kd
 * @param[in]      max_out: pid最大输出
 * @param[in]      max_iout: pid最大积分输出
 * @retval         none
 */
void PID_init(pid_type_def *pid,
              uint8_t mode,
              const fp32 PID[3],
              fp32 init_target,
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

    pid->dead_area = dead_area;
    pid->i_alpha   = input_alpha;
    pid->o_alpha   = output_alpha;
    pid->delta     = delta;
    pid->alpha     = alpha;
    pid->mode      = mode;
    pid->Kp        = PID[0];
    pid->Ki        = PID[1];
    pid->Kd        = PID[2];
    pid->set       = init_target;
    pid->out       = 0;
    pid->max_out   = max_out;
    pid->max_iout  = max_iout;
    // pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    // pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
 * @brief          cascade pid struct data init
 * @param[out]     c_pid: cascade PID struct data point
 * @param[in]      oPID: 0: kp, 1: ki, 2:kd
 * @param[in]      iPID: 0: kp, 1: ki, 2:kd
 * @param[in]      oPID_max_out: outside pid max out
 * @param[in]      oPID_max_iout: outside pid max iout
 * @param[in]      iPID_max_out: inside pid max out
 * @param[in]      iPID_max_iout: inside pid max iout
 * @retval         none
 */
/**
 * @brief          cascade pid struct data init
 * @param[out]     c_pid: 差分PID结构数据指针
 * @param[in]      oPID: 0: kp, 1: ki, 2:kd
 * @param[in]      iPID: 0: kp, 1: ki, 2:kd
 * @param[in]      oPID_max_out: 外环pid最大输出
 * @param[in]      oPID_max_iout: 外环pid最大积分输出
 * @param[in]      iPID_max_out: 内环pid最大输出
 * @param[in]      iPID_max_iout: 内环pid最大积分输出
 * @retval         none
 */
void cascade_PID_init(cascade_pid_t *c_pid,
                      const fp32 oPID[3], const fp32 iPID[3],
                      fp32 o_init_target, fp32 i_init_target,
                      fp32 o_input_alpha, fp32 o_output_alpha,
                      fp32 i_input_alpha, fp32 i_output_alpha,
                      fp32 o_delta, fp32 o_alpha,
                      fp32 i_delta, fp32 i_alpha,
                      fp32 oPID_max_out, fp32 oPID_max_iout,
                      fp32 iPID_max_out, fp32 iPID_max_iout,
                      fp32 o_dead_area, fp32 i_dead_area)
{
    PID_init(&c_pid->pid_outside, PID_POSITION, oPID, o_init_target, o_input_alpha, o_output_alpha, o_delta, o_alpha, oPID_max_out, oPID_max_iout, o_dead_area);
    PID_init(&c_pid->pid_inside, PID_POSITION, iPID, i_init_target, i_input_alpha, i_output_alpha, i_delta, i_alpha, iPID_max_out, iPID_max_iout, i_dead_area);
}

/**
 * @brief          pid calculate
 * @param[out]     pid: PID struct data point
 * @param[in]      ref: feedback data
 * @param[in]      set: set point
 * @retval         pid out
 */
/**
 * @brief          pid计算
 * @param[out]     pid: PID结构数据指针
 * @param[in]      ref: 反馈数据
 * @param[in]      set: 设定值
 * @retval         pid输出
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
    if (val_range > 0) pid->error[0] = choose_shortest_path(set, ref, val_range); // 最短路径选择
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

        if (fabsf(pid->error[0]) < pid->dead_area) { // 死区不输出省电
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

        if (fabsf(pid->error[0]) < pid->dead_area) { // 死区不输出省电
            pid->out = ZERO_POINT;
        }
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

/**
 * @brief          cascade pid calculate
 * @param[out]     c_pid: cascade PID struct data point
 * @param[in]      o_ref: outside feedback data
 * @param[in]      i_ref: inside feedback data
 * @param[in]      s_set: outside set point
 * @retval         cascade pid out
 */
/**
 * @brief          差分pid计算
 * @param[out]     c_pid: 差分PID结构数据指针
 * @param[in]      o_ref: 外环反馈数据
 * @param[in]      i_ref: 内环反馈数据
 * @param[in]      s_set: 外环设定值
 * @retval         差分pid输出
 */
fp32 cascade_PID_calc(cascade_pid_t *c_pid, fp32 o_ref, fp32 i_ref, fp32 o_set, fp32 o_range, fp32 i_range)
{
    c_pid->s_set = o_set;
    c_pid->s_fdb = o_ref;
    c_pid->v_fdb = i_ref;

    c_pid->v_set = PID_calc(&c_pid->pid_outside, c_pid->s_fdb, c_pid->s_set, o_range);
    c_pid->out   = PID_calc(&c_pid->pid_inside, c_pid->v_fdb, c_pid->v_set, i_range);

    return c_pid->out;
}

/**
 * @brief          pid out clear
 * @param[out]     pid: PID struct data point
 * @retval         none
 */
/**
 * @brief          pid 输出清除
 * @param[out]     pid: PID结构数据指针
 * @retval         none
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
 * @brief          cascade pid out clear
 * @param[out]     c_pid: cascade PID struct data point
 * @retval         none
 */
/**
 * @brief          差分pid输出清除
 * @param[out]     c_pid: 差分PID结构数据指针
 * @retval         none
 */
void cascade_PID_clear(cascade_pid_t *c_pid)
{
    PID_clear(&c_pid->pid_outside);
    PID_clear(&c_pid->pid_inside);
}
