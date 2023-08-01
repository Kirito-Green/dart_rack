/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pidʵ�ֺ�����������ʼ����PID���㺯����
  * @note
  * @history    2021-7-23   �����˴���pid�ĺ����ͽӿ�
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *  V2.0.0     Jly-23-2021     Qylann          1. ���Ӳ��ֺ���
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
 * @brief �ú���ʹ�ø���������ʼ�� PID ��������
 *
 * @param pid ָ����Ҫ��ʼ����PID�ṹ��ָ�롣
 * @param mode ��mode����������ָ��PID�������Ĺ���ģʽ�����ݾ����ʵ�֣������Ծ��в�ͬ��ֵ����ͨ���������ڲ�ͬ�Ŀ���ģʽ֮�����ѡ������λ�ÿ��ơ��ٶȿ��ƻ��������Ʋ��ԡ�
 * @param PID PID ����ֱ�����������桢���������΢�����档��Щ�������ڼ��� PID �������������
 * @param input_alpha input_alpha ������һ������ֵ����ʾ PID �����������ڹ��������źŵ� alpha ֵ����ȷ����ǰ����ֵ����ǰ����ֵ��ȵ�Ȩ�ء�
 * input_alpha ��ֵԽ�ߣ���ǰ����ֵ��Ȩ�ؾ�Խ�󣬴Ӷ������ٶȸ���
 * @param output_alpha output_alpha ������һ������ֵ����ʾ���ڹ��� PID ����������� alpha ֵ��������ƽ������źŵı仯��
 * @param delta PID_init �����еġ�delta��������һ������ֵ����ʾ���������������޶ȡ���ֵԽ���ֹƵ�ʱ�󣬴����������ٶȱ��
 * ��ֵԽС��ֹƵ�ʱ�С������խ�������ٶȱ��
 * @param alpha alpha���������ڵ�������������ı�ʾ��������������ָ������ָ��С��1ʱ��������С���С�����
 * ��ָ������1�����С����С ���������
 * @param max_out PID���������Բ�����������ֵ��
 * @param max_iout ������max_iout����ʾPID��������������������ֵ�������ƻ�������ۻ�����ֹ��������̫�������ϵͳ���ȶ��򳬵���
 * @param dead_area dead_area ������ʾ�����趨����Χ�ķ�Χ����������Ϊ���Ժ��Բ��ƣ����Ҳ���ȡ�κο��Ʋ�����
 *
 * @return ʲô��û�У���Ч����
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
 * @brief �ú���ʹ��ָ��������ʼ������ PID ��������
 *
 * @param c_pid ָ�򽫱���ʼ����cascade_pid_t�ṹ��ָ�롣
 * @param oPID oPID ��������������ֵ�����飬��ʾ�ⲿ PID �������ı������桢���������΢�����档
 * @param iPID ��iPID��������һ������������ֵ��ɵ����飬��ʾ�ڲ� PID �������ı������桢���������΢�����档
 * @param o_input_alpha ������o_input_alpha�������ڹ����ⲿPID�����������alphaֵ����ȷ����ǰ����ֵ����ǰ����ֵ��ȵ�Ȩ�ء��ϸߵ� alpha
 * ֵ��Ϊ��ǰ����ֵ��������Ȩ�أ��Ӷ����¸������Ӧ��������
 * @param o_output_alpha ������o_output_alpha���������ⲿ PID ����������� alpha ֵ����ȷ�������������Ӧ�����仯�����ʡ��ϸߵ� alpha
 * ֵ�����¸������Ӧ����Ҳ���ܵ��³����Ͳ��ȶ���
 * @param i_input_alpha ������i_input_alpha���������ڲ�PID�����������alphaֵ���������ڲ� PID ������������ֵ�ĸ������ʡ�
 * @param i_output_alpha ������i_output_alpha���������ڲ�PID�����������alphaֵ���������ڲ� PID ����������仯�����ʡ�
 * ��i_output_alpha����ֵԽ�ߣ��ڲ� PID �������������ӦԽ�졣
 * @param o_delta ������cascade_PID_init���еĲ�����o_delta����ʾ�ⲿPID��������ʾ���������������޶ȡ���ֵԽ���ֹƵ�ʱ�󣬴����������ٶȱ��
 * ��ֵԽС��ֹƵ�ʱ�С������խ�������ٶȱ��
 * @param o_alpha ������cascade_PID_init���еĲ�����o_alpha�����������ⲿPID��������alphaֵ�� alpha
 * ֵȷ����������������ָ������ָ��С��1ʱ��������С���С�������ָ������1�����С����С ���������
 * @param i_delta ������cascade_PID_init���еĲ�����i_delta����ʾ�ڲ�PID��������ʾ���������������޶ȡ���ֵԽ���ֹƵ�ʱ�󣬴����������ٶȱ��
 * ��ֵԽС��ֹƵ�ʱ�С������խ�������ٶȱ��
 * @param i_alpha ������i_alpha���������ڲ�PID�������������alphaֵ����ȷ����������������ָ������ָ��С��1ʱ��������С���С�����
 * ��ָ������1�����С����С ���������
 * @param oPID_max_out ������oPID_max_out����ʾ�ⲿ PID ��������������ֵ�������ƿ�����������Ϊ�����ź���������ֵ��
 * @param oPID_max_iout ������oPID_max_iout����ʾ�ⲿPID�����������������ֵ�������ƻ�������Դﵽ�����ֵ����ֹ���ȱ��͡�
 * @param iPID_max_out ������iPID_max_out����ʾ�ڲ�PID��������������ֵ�����������ڲ�PID������������������ֵ��
 * @param iPID_max_iout ������iPID_max_iout����ʾ�ڲ�PID��������������������ơ����������ڲ�PID�������Ļ�������Դﵽ�����ֵ��
 * @param o_dead_area
 * ������o_dead_area��ָ�����ⲿPID����������е������������������趨����Χ��һ����Χ���ڸ÷�Χ�ڿ������������Ϊ�㡣�������ڵ��κδ��󶼲��ᵼ�¿�������������κα仯��
 * @param i_dead_area ������i_dead_area����ʾ�ڲ�PID�������������������ǿ���������Ӧ������κ����������ֵ��Χ�������ڷ�ֹ�����ź��е�΢С�������������𲻱�Ҫ�Ŀ��ƶ�����
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
 * @brief �ú������ݲο�ֵ���趨ֵ�͵�ǰֵ���� PID �������������
 *
 * @param pid ָ�� pid_type_def ���ͽṹ��ָ�룬���а��� PID �����������ͱ�����
 * @param ref ��ref��������ʾϵͳ��ͼ�ﵽ�Ĳο�ֵ���趨�㡣
 * @param set �����á�������ʾ�ܿ�ϵͳ�������趨���Ŀ��ֵ������ϵͳ��ͼ�ﵽ��ά�ֵļ�ֵ��
 * @param val_range ������val_range����ʾ������ȡ��ֵ�ķ�Χ���ڹ���������ѡ���趨ֵ�Ͳο�ֵ֮������·����
 *
 * @return fp32������ 32 λ�����͵�ֵ����ʾ PID �������������
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
    pid->set = pid->i_alpha * pid->last_set + (1 - pid->i_alpha) * pid->set; // ��ͨ�˲�������
#endif

    pid->fdb      = ref;
    pid->error[0] = set - ref;
    if (val_range > 0)
        pid->error[0] = choose_shortest_path(set, ref, val_range); // ���·��ѡ��
    if (pid->mode == PID_POSITION) {
#ifdef PID_NON_LINEAR_OUTPUT
        pid->error[0] = fal(pid->error[0], pid->alpha, pid->delta); // �����Լ���
#endif

        pid->Pout = pid->Kp * pid->error[0];

#ifdef PID_I_DISCREATE
        if (fabsf(pid->error[0]) <= I_CONTROL_DISCTEATE) { // ���ַ���
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
        pid->out = pid->Pout + pid->Iout + pid->Dout; // �������

#ifdef OUTPUT_FILTER_FIRST_ORDER
        pid->out = pid->o_alpha * pid->last_out + (1 - pid->o_alpha) * pid->out; // ��ͨ�˲����
#endif

        if (fabsf(pid->error[0]) <= pid->dead_area) { // ���������ʡ��
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
        if (fabsf(pid->error[0]) - fabsf(pid->error[1]) > D_CONTROL_POINT) { // D���Ʒ���
            pid->Dout = pid->Kd * pid->Dbuf[0];
        } else {
            pid->Dout = 0;
        }
#else
        pid->Dout = pid->Kd * pid->Dbuf[0];
#endif

        pid->out = pid->Pout + pid->Iout + pid->Dout;

#ifdef OUTPUT_FILTER_FIRST_ORDER
        pid->out = pid->o_alpha * pid->last_out + (1 - pid->o_alpha) * pid->out; // ��ͨ�˲����
#endif

        if (fabsf(pid->error[0]) <= pid->dead_area) { // ���������ʡ��
            pid->out = ZERO_POINT;
        }
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

/**
 * @brief �ú������ݸ����Ĳο�ֵ�ͷ�Χ���㼶�� PID �������������
 *
 * @param c_pid ָ��cascade_pid_t���ͽṹ��ָ�룬���а�������PID����������ı�����
 * @param o_ref ����PID�������⻷�Ĳο�ֵ���������ܿ�ϵͳ���������ֵ��
 * @param i_ref ������i_ref����ʾ�ڲ� PID �������Ĳο����롣�����ڻ����Ƶ�����ֵ��
 * @param o_set �ⲿ PID ��·���������趨ֵ��
 * @param o_range ������o_range����ʾ���������o_set���ķ�Χ������ PID ����������ȷ���������ı�����������΢���
 * @param i_range ������i_range����ʾ��������ķ�Χ�������ڲ�PID�������ļ��㡣
 *
 * @return ������c_pid->out����ֵ��������Ϊ fp32��
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
 * @brief ������PID_clear����� PID �������ṹ�и��ֱ�����ֵ��
 *
 * @param pid ָ�� pid_type_def ���͵Ľṹ��ָ�롣
 *
 * @return �����pid������Ϊ��NULL�������������ض���ִ���κβ�����
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
 * @brief ������cascade_PID_clear��������� PID �ṹ�ڵ� PID ��������
 *
 * @param c_pid c_pid ��һ��ָ��cascade_pid_t ���͵Ľṹ��ָ�롣
 */
void cascade_PID_clear(cascade_pid_t *c_pid)
{
    PID_clear(&c_pid->pid_outside);
    PID_clear(&c_pid->pid_inside);
}
