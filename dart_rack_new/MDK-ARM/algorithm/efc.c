#include "efc.h"
#include "user_lib.h"

/**
 * @brief �ú���ʹ�ø���������ʼ�� EFC(���ٿ���)�ṹ��
 *
 * @param efc ָ�� efc_type_def ���͵Ľṹ��ָ�롣�ýṹ���ܰ�������ӷ�������ϵͳ��صı��������á�
 * @param kp kp �ǿ���ϵͳ��ʹ�õı��������������ȷ��������ֵ��ʵ��ֵ֮���������Ӧǿ�ȡ� kp ֵԽ�ߣ���ӦԽǿ��
 * @param kd kd �ǿ���ϵͳ��΢�������ֵ��������ȷ����Ӧ����źű仯������仯�ʡ�
 * @param max_out EFC(���ٿ���)���Բ�����������ֵ�������ڽ������ź�������һ����Χ�ڡ�
 *
 * @return ʲô��û��(��Ч)��
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
 * @brief �ú������ݲο�ֵ���趨ֵ����������ֿ�����(PI ������)������������������Ϊ���ֵ��
 *
 * @param efc ָ�� efc_type_def ���͵Ľṹ��ָ�롣�ýṹ���ܰ����� EFC(��������������)ϵͳ��صĸ��ֲ����ͱ�����
 * @param ref ��ref��������EFC(��������������)��ͼ�ﵽ�Ĳο�ֵ������һ�������� (fp32)����ʾ EFC ������·������ֵ��
 * @param set �����á�������ʾ����ϵͳ������趨���Ŀ��ֵ�����ǿ���ϵͳ��ͼʵ�ֻ�ά�ֵļ�ֵ��
 *
 * @return ������efc->out����ֵ��������Ϊ��fp32����
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
 * @brief �ú���ʹ�ñ��������΢�����棬���ݲο�ֵ���趨ֵ���� PID �������������
 *
 * @param efc ָ�� efc_type_def ���͵Ľṹ��ָ�롣�ýṹ���ܰ����� EFC(��������������)ϵͳ��صı����Ͳ�����
 * @param ref ��ref�������ǿ�������ͼ�ﵽ�Ĳο�ֵ���趨�㡣
 * @param set �����á�������ʾ����ϵͳ������趨���ο�ֵ������ϵͳ��ͼʵ�ֻ�ά�ֵļ�ֵ��
 *
 * @return ������efc->out����ֵ��������Ϊ fp32(���� 32 λ)��
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
 * @brief EFPI ������������PI������
 *
 * @param z ������z���ǵ����ȸ�����(fp32)��
 *
 * @return ����ֵ (fp32)��
 */
fp32 EFPI(fp32 z)
{
    return (pow(z, 3.0f) - 2.582f * pow(z, 2.0f) + 2.222f * z - 0.6376f) /
           (pow(z, 3.0f) - 2.633 * pow(z, 2.0f) + 2.304f * z - 0.6703f);
}

/**
 * @brief ���� EFPD ������������PD������
 *
 * @param z ������z���ǵ����ȸ�����(fp32)��
 *
 * @return ʹ�ø�����ʽ����ĸ���ֵ (fp32)��
 */
fp32 EFPD(fp32 z)
{
    return (2.0f * pow(z, 3.0f) - 5.214f * pow(z, 2.0f) + 4.524f * z - 1.307f) /
           (pow(z, 3.0f) - 2.582f * pow(z, 2.0f) + 2.222f * z - 0.6376f);
}

/**
 * @brief ���� EFLO �������ٹ۲���
 *
 * @param z ������z����ʾ��������
 *
 * @return fp32 ���͵�ֵ(���� 32 λ)
 */
fp32 EFLO(fp32 z)
{
    return (0.007643f * pow(z, 3.0f) - 0.01974f * pow(z, 2.0f) + 0.01699f * z - 0.004874f) /
           (pow(z, 4.0f) - 3.188f * pow(z, 3.0f) + 3.786f * pow(z, 2.0) - 1.984f * z + 0.3864f);
}
