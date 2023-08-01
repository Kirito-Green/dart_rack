#ifndef OLED_H
#define OLED_H

#include "main.h"

#define NUMBER 0x00
#define LOWER 0x01
#define UPPER 0x02
#define SPECIAL 0x03

#define OLED_CMD 0x00
#define OLED_DATA 0x40
#define OLED_CLEAR 0x00
#define OLED_LIGHT 0x01
#define OLED_TOGGLE 0x02

#define OLED_RST_SET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET)
#define OLED_RST_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
#define OLED_DC_SET_CMD HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET)
#define OLED_DC_SET_DATA HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

extern void oled_write_byte(uint8_t dat, uint8_t cmd);
extern void oled_ready(void);
extern void oled_init(void);
extern void oled_display_on(void);
extern void oled_display_off(void);
extern void oled_set_pos(uint8_t x, uint8_t y);
extern void oled_refresh(void);
extern void oled_operate(uint8_t operate);
extern void oled_refresh(void);
extern void oled_reset(void);
extern void oled_new_line(void);
extern void oled_enter_string(uint8_t *pdata, uint8_t size);
extern void oled_enter_one(uint8_t n, uint8_t type);

#endif
