#include "adrc.h"
#include "user_lib.h"

/**
 * @brief �ú������ݸ�����������������������ֵ��
 *
 * @param fp32 x1 ���������ĳ�ʼֵ��
 * @param fp32 x2 ����x2����ڶ�����ֵ��
 * @param fp32 delta ������delta����ʾ������ֵ΢�ֵ�Сֵ����ͨ����һ��С������������ 0.001 �� 0.0001��
 * @param fp32 h ������h����ʾ������ʹ�õĲ�����ʱ������������ȷ��������ʱ��ı仯��
 *
 * @return fp32 ���͵�ֵ��
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
 * @brief �ú�����ʼ��������Դ���ſ����� (LADRC) �Ĳ�����
 *
 * @param ardc ָ�� ladrc_type_def ���ͽṹ��ָ�룬���а��� LADRC �㷨�Ĳ����ͱ�����
 * @param LADRC LADRC ������һ������ 6 ������ֵ�����顣��Щֵ���� LADRC(������Դ���ſ���)�ṹ��ʼ��ʱʹ�õĲ�ͬ������
 * @param max_out ��max_out��������ʾLADRC(������Դ���ſ���)���������Բ�����������ֵ�������ڽ�������������������ض���Χ�ڡ�
 *
 * @return ʲô��û��(��Ч)��
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
 * @brief �ú�����ʼ�� ADRC(�������ſ���)�������Ĳ�����
 *
 * @param ardc ָ����Ҫ��ʼ����ADRC�ṹ��ָ�롣
 * @param ADRC ADRC ��һ���� 12 ������ֵ��ɵ����飬��ʾ ADRC(�������ſ���)�㷨�Ĳ�����
 * @param max_out ��max_out�������� ADRC(�������ſ���)���������Բ�����������ֵ�������ڽ�������������������ض���Χ�ڡ�
 *
 * @return �ú����������κ�ֵ������һ�� void �������͡�
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
 * @brief �ú���ʹ��ָ��������ʼ������ LADRC ��������
 *
 * @param c_adrc ָ��cascade_ladrc_t���ͽṹ��ָ�룬���а���LADRC������������ʵ��(ladrc_outside��ladrc_inside)��
 * @param oADRC oADRC ��һ���� 6 ����������ɵ����飬��ʾ�ⲿ ADRC(�������ſ���)�������Ĳ�����
 * @param iADRC iADRC ������һ���� 6 ����������ɵ����飬��ʾ�ڲ� ADRC(�������ſ���)������
 * @param oADRC_max_out �ⲿ ADRC ��������������ֵ��
 * @param iADRC_max_out ������iADRC_max_out����ʾ�ڲ� ADRC ��������������ֵ��
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
 * @brief �ú������ݲο�ֵ���趨ֵ����������Դ���ſ����� (LADRC) �������
 *
 * @param ardc ָ��ladrc_type_def�����ͽṹ��ָ�룬���а��� LADRC ������ʹ�õĲ����ͱ�����
 * @param ref ��ref�������������ϵͳ�Ĳο�ֵ������ϵͳӦ�ôﵽ��ά�ֵ�����ֵ��
 * @param set �����á�������ʾ����ϵͳ������趨���ο�ֵ������ϵͳ��ͼʵ�ֻ�ά�ֵļ�ֵ��
 *
 * @return `ardc->out` ��ֵ��������Ϊ `fp32`��
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
 * @brief �ú������ݸ���������Ͳ���������Դ���ſ��� (ADRC) �㷨�������
 *
 * @param ardc ָ��adrc_type_def�����ͽṹ��ָ�룬���а��� ADRC ������ʹ�õĸ��ֲ����ͱ�����
 * @param ref ��ref��������ʾϵͳ��ͼ�ﵽ�Ĳο�ֵ������ֵ��
 * @param set �����á�������ʾ����ϵͳ������趨���ο�ֵ������ϵͳ��ͼʵ�ֻ�ά�ֵļ�ֵ��
 *
 * @return fp32(���� 32 λ)���͵�ֵ����ʾ ADRC(�������ſ���)����������
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
 * @brief �ú������ݸ����Ĳο������ü��㼶�� LADRC �������������
 *
 * @param c_adrc ָ��cascade_ladrc_t���͵Ľṹ���ָ�룬���а���������ʹ�õĸ��ֱ����ͽṹ�塣
 * @param o_ref ����LADRC�������⻷�Ĳο����ֵ��
 * @param i_ref ������i_ref����ʾ����ο�ֵ��
 * @param o_set ���� LADRC ���������������趨ֵ��
 *
 * @return `c_adrc->out` ��ֵ��������Ϊ `fp32`��
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
