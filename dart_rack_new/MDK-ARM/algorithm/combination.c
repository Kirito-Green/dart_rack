#include "combination.h"
#include "setting.h"
#include "pid.h"
#include "adrc.h"
#include "efc.h"

/**
 * @brief �ú���ʹ��ָ��������ʼ������ PID-LADRC ��������
 *
 * @param c_pid_ladrc ָ�򽫱���ʼ����cascade_pid_ladrc_t�ṹ��ָ�롣
 * @param oPID oPID ������һ������������ֵ��ɵ����飬��ʾ�ⲿ PID ��������ϵ����������ֵ�ֱ��ӦPID�������ı������桢���������΢�����档
 * @param iADRC iADRC ��һ������ 6 ������ֵ�����飬��ʾ�ڲ� ADRC(�������ſ���)�������Ĳ�������Щ�������ڵ����ڲ���������ȷ������Ϊ��
 * @param o_init_target �ⲿ PID �������ĳ�ʼĿ��ֵ��
 * @param o_input_alpha ������o_input_alpha�������� PID �������������˲����� alpha ֵ����ȷ����ǰ����ֵ����ǰ����ֵ��ȵ�Ȩ�ء��ϸߵ� alpha
 * ֵ���赱ǰ����ֵ�����Ȩ�أ��Ӷ����¸������Ӧ�������ܲ�����������
 * @param o_output_alpha ������o_output_alpha����PID������������˲���ϵ������ȷ�� PID �������������Ӧ����źű仯���仯�����ʡ�
 * ��o_output_alpha��ֵԽ�ߣ�PID �������������ӦԽ��
 * @param o_delta ������cascade_PID_LADRC_init���еĲ�����o_delta�����������ⲿPID������������޶�ֵ����ֵ������PID��������˫�˴���ı߽�ֵ
 * @param o_alpha ������cascade_PID_LADRC_init���еĲ�����o_alpha�����������ⲿPID��������alpha����ֵ�� alpha ���������� PID�����Դ��������̶�
 * @param oPID_max_out �ⲿ PID ��������������ֵ��
 * @param oPID_max_iout oPID_max_iout ���ⲿ PID ��������������ֵ���ò��������ⲿPID������������������ֵ��
 * @param iADRC_max_out ������iADRC_max_out����ʾ�ڲ� ADRC ��������������ֵ�������ڽ��ڲ�������������������ض���Χ�ڡ�
 * @param o_dead_area
 * ��cascade_PID_LADRC_init�������еġ�o_dead_area����������ָ��PID����������������������ǿ��������Ϊ����趨ֵ��Χ�ķ�Χ���������ڷ�ֹ�豸��Χ����С����
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
 * @brief �ú������ݸ����Ĳο�ֵ�����ü��㼶�� PID-LADRC �������������
 *
 * @param c_pid_ladrc ָ��cascade_pid_ladrc_t���͵Ľṹ���ָ�룬���а���������ʹ�õĸ��ֱ����ͽṹ�塣
 * @param o_ref ����PID-LADRC�������⻷�Ĳο�ֵ���������ܿ�ϵͳ���������ֵ��
 * @param i_ref ������i_ref����ʾ���� PID-LADRC �������ڻ��Ĳο����롣������ѭ�����ٵ�����ֵ��
 * @param o_set ϵͳ��������ֵ��
 *
 * @return `c_pid_ladrc->out` ��ֵ��������Ϊ `fp32`��
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
