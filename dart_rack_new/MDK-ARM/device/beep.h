#ifndef _BEEP_H
#define _BEEP_H

#include "struct_typedef.h"

#define BEEP_READY_TUNE   21
#define BEEP_FINE_TUNE    14
#define BEEP_ALARM_TUNE   19
#define BEEP_READY_VOLUME 90
#define BEEP_FINE_VOLUME  90
#define BEEP_ALARM_VOLUME 90
#define BEEP_READY_TIMES  3
#define BEEP_FINE_TIMES   1
#define BEEP_ALARM_TIMES  2
#define BEEP_DELAY_TIME   300

extern void beep_ready(void);
extern void beep_fine(void);
extern void beep_alarm(void);
extern void beep_init(void);
extern void beep_stop(void);
extern void beep_play_music_steps(void);
extern void beep_play_seven_feets_soundly(void);
extern void beep_play_beautifully_you_laugh(void);

#endif
