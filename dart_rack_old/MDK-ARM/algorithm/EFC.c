#include "EFC.h"
#include "User_lib.h"

/* kc��matlab��ʹ�� */
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

fp32 EFC_PIO_calc(efc_type_def *efc, fp32 ref, fp32 set)
{
    efc->set = set;
    efc->fdb = ref;

    efc->out = efc->Kp * EFPI(set) - EFLO(ref);
    LimitMax(efc->out, efc->max_out);

    return efc->out;
}

fp32 EFC_PID_calc(efc_type_def *efc, fp32 ref, fp32 set)
{
    efc->set = set;
    efc->fdb = ref;

    efc->out = efc->Kp * EFPI(set) - efc->Kd * EFPD(ref);
    LimitMax(efc->out, efc->max_out);

    return efc->out;
}

/* ��������PI������ */
fp32 EFPI(fp32 z)
{
    return (pow(z, 3.0f) - 2.582f * pow(z, 2.0f) + 2.222f * z - 0.6376f) /
           (pow(z, 3.0f) - 2.633 * pow(z, 2.0f) + 2.304f * z - 0.6703f);
}

/* ��������PD������ */
fp32 EFPD(fp32 z)
{
    return (2.0f * pow(z, 3.0f) - 5.214f * pow(z, 2.0f) + 4.524f * z - 1.307f) /
           (pow(z, 3.0f) - 2.582f * pow(z, 2.0f) + 2.222f * z - 0.6376f);
}

/* �������ٳ�ǰ������ */
fp32 EFLO(fp32 z)
{
    return (0.007643f * pow(z, 3.0f) - 0.01974f * pow(z, 2.0f) + 0.01699f * z - 0.004874f) /
           (pow(z, 4.0f) - 3.188f * pow(z, 3.0f) + 3.786f * pow(z, 2.0) - 1.984f * z + 0.3864f);
}
