#ifndef ADRC_H
#define ADRC_H

#include "struct_typedef.h"


extern fp32 fst(fp32 x1, fp32 x2, fp32 delta, fp32 h);

extern void LADRC_init(ladrc_type_def* ardc,
							 const fp32 ADRC[6],
               fp32 max_out);

extern void ADRC_init(adrc_type_def* ardc,
							 const fp32 ADRC[12],
							 fp32 max_out);
extern void cascade_LADRC_init(cascade_ladrc_t* c_adrc,
												const fp32 oADRC[6],
												const fp32 iADRC[6],
												fp32 oADRC_max_out,
												fp32 iADRC_max_out);
extern fp32 LADRC_calc(ladrc_type_def* ardc, fp32 ref, fp32 set);
extern fp32 ADRC_calc(adrc_type_def* ardc, fp32 ref, fp32 set);
extern fp32 cascade_LADRC_calc(cascade_ladrc_t* c_adrc,
											 fp32 o_ref,
											 fp32 i_ref,
						           fp32 o_set);

#endif

