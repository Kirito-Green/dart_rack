#ifndef _TFCARD_SAVE_THREAD
#define _TFCARD_SAVE_THREAD

#include "ff.h"
#include "struct_typedef.h"

#define MOTOR_SAVE_NUMBER     10
#define TFCARD_BUFF_SIZE      256
#define MOTOR_DIST_ECD_NUMBER 6

extern void tfcard_transmit_thread(void const *argument);
extern FRESULT tfcard_read_motor(DartRackMotorMeasure_t *measure);
extern FRESULT tfcard_save_motor(void);
// extern FRESULT tfcard_save_measure(void);
// extern FRESULT tfcard_save_referee(void);

#endif
