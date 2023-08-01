#include "Combination.h"
#include "Setting.h"
#include "pid.h"
#include "ADRC.h"
#include "EFC.h"

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
    PID_init(&c_pid_ladrc->pid_outside, PID_POSITION, oPID, o_init_target, o_input_alpha, o_output_alpha, o_delta, o_alpha, oPID_max_iout, oPID_max_out, o_dead_area);
    LADRC_init(&c_pid_ladrc->ladrc_inside, iADRC, iADRC_max_out);
}

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
