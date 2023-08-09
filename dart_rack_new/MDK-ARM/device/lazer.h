#ifndef _LAZER_H
#define _LAZER_H

#include "main.h"

#define GPIO_LAZER_PORT GPIOG
#define GPIO_LAZER_PIN  GPIO_PIN_13

extern void lazer_on(void);
extern void lazer_off(void);

#endif
