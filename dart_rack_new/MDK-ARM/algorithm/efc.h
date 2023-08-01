#ifndef EFC_H
#define EFC_H

#include "struct_typedef.h"

extern void EFC_Init(efc_type_def* efc, fp32 kp, fp32 kd, fp32 max_out);
extern fp32 EFC_PIO_calc(efc_type_def* efc, fp32 ref, fp32 set);
extern fp32 EFC_PID_calc(efc_type_def* efc, fp32 ref, fp32 set);
extern fp32 EFPI(fp32 z);
extern fp32 EFPD(fp32 z);
extern fp32 EFLO(fp32 z);


#endif
