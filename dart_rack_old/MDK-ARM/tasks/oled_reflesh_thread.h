#ifndef OLED_REFLESH_THREAD_H
#define OLED_REFLESH_THREAD_H

#include "main.h"

#define OLED_SHOW_LINE 2

extern void oled_reflesh_thread(void const *argument);
extern void oled_show(void);

#endif
