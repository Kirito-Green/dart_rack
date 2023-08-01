#include "ADRC.h"
#include "User_lib.h"

fp32 fst(fp32 x1, fp32 x2, fp32 delta, fp32 h)
{
    fp32 d  = delta * h;
    fp32 d0 = h * d;
    fp32 y  = x1 + h * x2;
    fp32 a0 = sqrt(pow(d, 2.0) + 8 * delta * fabsf(y));
    fp32 a  = fabsf(y) > d0 ? x2 + (a0 - d) / 2.0f * sign(y) : x2 + y / h;
    return fabsf(a) > d ? -delta * sign(a) : -delta * a / d;
}

void LADRC_init(ladrc_type_def *ardc,
                const fp32 LADRC[6],
                fp32 max_out)
{
    if (ardc == NULL || ardc == NULL) {
        return;
    }
    /* 微锟街革拷锟斤拷锟斤拷 */
    ardc->r1              = 0.0f;
    ardc->r2              = 0.0f;
    ardc->sampling_period = LADRC[0];
    ardc->tracking_delta  = LADRC[1];

    /* 锟斤拷锟脚观诧拷锟斤拷 */
    ardc->w0            = LADRC[2];
    ardc->b             = LADRC[3];
    ardc->z1            = 0.0f;
    ardc->z2            = 0.0f;
    ardc->z3            = 0.0f;
    ardc->observe_beta1 = 3 * ardc->w0;
    ardc->observe_beta2 = 3 * pow(ardc->w0, 2.0f);
    ardc->observe_beta3 = pow(ardc->w0, 3.0f);

    /* 锟斤拷锟斤拷锟斤拷锟� */
    ardc->k1 = LADRC[4];
    ardc->k2 = LADRC[5];

    /* 锟睫凤拷 */
    ardc->max_out = max_out;
}

void ADRC_init(adrc_type_def *ardc,
               const fp32 ADRC[12],
               fp32 max_out)
{
    if (ardc == NULL || ardc == NULL) {
        return;
    }
    /* 微锟街革拷锟斤拷锟斤拷 */
    ardc->r1              = 0.0f;
    ardc->r2              = 0.0f;
    ardc->sampling_period = ADRC[0];
    ardc->tracking_delta  = ADRC[1];

    /* 锟斤拷锟脚观诧拷锟斤拷 */
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

    /* 锟斤拷锟斤拷锟斤拷锟斤拷锟� */
    ardc->out_delta  = ADRC[7];
    ardc->out_alpha1 = ADRC[8];
    ardc->out_alpha2 = ADRC[9];
    ardc->out_beta1  = ADRC[10];
    ardc->out_beta2  = ADRC[11];

    /* 锟睫凤拷 */
    ardc->max_out = max_out;
}

void cascade_LADRC_init(cascade_ladrc_t *c_adrc,
                        const fp32 oADRC[6],
                        const fp32 iADRC[6],
                        fp32 oADRC_max_out,
                        fp32 iADRC_max_out)
{
    LADRC_init(&c_adrc->ladrc_outside, oADRC, oADRC_max_out);
    LADRC_init(&c_adrc->ladrc_inside, iADRC, iADRC_max_out);
}

fp32 LADRC_calc(ladrc_type_def *ardc, fp32 ref, fp32 set)
{
    if (ardc == NULL) {
        return 0.0f;
    }
    ardc->last_set = ardc->set;
    ardc->last_out = ardc->out;
    ardc->set      = set;
    ardc->fdb      = ref;

    /* 微锟街革拷锟斤拷锟斤拷 */
    ardc->r1 += ardc->sampling_period * ardc->r2;
    ardc->r2 += ardc->sampling_period * fst(ardc->r1 - set, ardc->r2, ardc->tracking_delta, ardc->sampling_period);

    ardc->e1 = ardc->r1 - ardc->z1;
    ardc->e2 = ardc->r2 - ardc->z2;

    /* 锟斤拷锟斤拷锟斤拷锟� */
    ardc->out = ardc->k1 * ardc->e1 + ardc->k2 * ardc->e2 - ardc->z3 / ardc->b;
    LimitMax(ardc->out, ardc->max_out);

    /* 锟脚讹拷锟斤拷锟斤拷 */
    fp32 epsilon = ardc->z1 - ref;
    ardc->z1 += ardc->sampling_period * (ardc->z2 - ardc->observe_beta1 * epsilon);
    ardc->z2 += ardc->sampling_period * (ardc->z3 - ardc->observe_beta2 * epsilon + ardc->b * ardc->out);
    ardc->z3 += -ardc->sampling_period * ardc->observe_beta3 * epsilon;

    return ardc->out;
}

fp32 ADRC_calc(adrc_type_def *ardc, fp32 ref, fp32 set)
{
    if (ardc == NULL) {
        return 0.0f;
    }
    ardc->last_set = ardc->set;
    ardc->last_out = ardc->out;
    ardc->set      = set;
    ardc->fdb      = ref;

    /* 微锟街革拷锟斤拷锟斤拷 */
    ardc->r1 += ardc->sampling_period * ardc->r2;
    ardc->r2 += ardc->sampling_period * fst(ardc->r1 - set, ardc->r2, ardc->tracking_delta, ardc->sampling_period);

    ardc->e1 = ardc->r1 - ardc->z1;
    ardc->e2 = ardc->r2 - ardc->z2;

    /* 锟斤拷锟斤拷锟斤拷锟斤拷锟� */
    ardc->out = ardc->out_beta1 * fal(ardc->e1, ardc->out_alpha1, ardc->out_delta) +
                ardc->out_beta2 * fal(ardc->e2, ardc->out_alpha2, ardc->out_delta) -
                ardc->z3 / ardc->b;
    LimitMax(ardc->out, ardc->max_out);

    /* 锟脚讹拷锟斤拷锟斤拷 */
    fp32 epsilon = ardc->z1 - ref;
    ardc->z1 += ardc->sampling_period * (ardc->z2 - ardc->observe_beta1 * epsilon);
    ardc->z2 += ardc->sampling_period * (ardc->z3 - ardc->observe_beta2 * fal(epsilon, ardc->observe_alpha1, ardc->observe_delta) + ardc->b * ardc->out);
    ardc->z3 += -ardc->sampling_period * ardc->observe_beta3 * fal(epsilon, ardc->observe_alpha2, ardc->observe_delta);

    return ardc->out;
}

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
