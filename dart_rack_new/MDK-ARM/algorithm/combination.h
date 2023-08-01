#ifndef COMBINATION_H
#define COMBINATION_H

#include "struct_typedef.h"


extern void cascade_PID_LADRC_init(cascade_pid_ladrc_t* c_pid_ladrc,
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
												fp32 o_dead_area);
									
extern fp32 cascade_PID_LADRC_calc(cascade_pid_ladrc_t* c_pid_ladrc,
											 fp32 o_ref,
											 fp32 i_ref,
						           fp32 o_set);		


#endif

