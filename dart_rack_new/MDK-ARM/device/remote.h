/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief      ңңͨSBUSЭ鴫䣬DMA䷽ʽԼCPU
  *             ԴôڿжͬʱṩһЩDMA
  *             ķʽ֤Ȳεȶԡ
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "struct_typedef.h"
#include "main.h"
#define SBUS_RX_BUF_NUM    36u

#define RC_FRAME_LENGTH    18u

#define RC_CH_VALUE_MIN    ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX    ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP          ((uint16_t)1)
#define RC_SW_MID         ((uint16_t)3)
#define RC_SW_DOWN        ((uint16_t)2)
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s)  (s == RC_SW_MID)
#define switch_is_up(s)   (s == RC_SW_UP)
#define PRESS             ((uint8_t)1)
#define RELEASE           ((uint8_t)0)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W     ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S     ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A     ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D     ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL  ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q     ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E     ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R     ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F     ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G     ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z     ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X     ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C     ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V     ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B     ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */
typedef __PACKED_STRUCT
{
    __PACKED_STRUCT
    {
        int16_t ch[5];
        char s[2];
        char s_last[2];
    }
    rc;
    __PACKED_STRUCT
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    }
    mouse;
    __PACKED_STRUCT
    {
        uint16_t v;
    }
    key;
}
RC_ctrl_t;

/* ----------------------- Internal Data ----------------------------------- */

extern void remote_control_init(void);
extern uint8_t sbus_rx_buf[SBUS_RX_BUF_NUM];
extern const RC_ctrl_t *get_remote_control_point(void);
extern uint8_t RC_data_is_error(void);
extern void slove_RC_lost(void);
extern void slove_data_error(void);
extern void sbus_to_rc(void);

// ļֵ
extern bool_t cheak_key_press(uint16_t key);
// ش
extern bool_t cheak_key_press_once(uint16_t key);

// һҡֵ
extern fp32 remote_channel_rightX(void);
extern fp32 remote_channel_rightY(void);
extern fp32 remote_channel_leftX(void);
extern fp32 remote_channel_leftY(void);
extern fp32 remote_dial(void);

// һƶ
extern fp32 mouse_moveX(void);
extern fp32 mouse_moveY(void);

// Ҽ
extern bool_t mouse_press_left(void);
extern bool_t mouse_press_right(void);

// λü
extern bool_t handle_chassis_up(void);
extern bool_t handle_chassis_down(void);
extern bool_t handle_chassis_right(void);
extern bool_t handle_chassis_left(void);
extern bool_t handle_gimbal_up(void);
extern bool_t handle_gimbal_down(void);
extern bool_t handle_gimbal_right(void);
extern bool_t handle_gimbal_left(void);
extern bool_t handle_dial_down(void);
extern bool_t handle_dial_up(void);
extern bool_t switch_right_up(void);
extern bool_t switch_right_mid(void);
extern bool_t switch_right_down(void);
extern bool_t switch_left_Up(void);
extern bool_t switch_left_mid(void);
extern bool_t switch_left_down(void);

extern bool_t handle_chassis_resetX(void);
extern bool_t handle_chassis_resetY(void);
extern bool_t handle_gimbal_resetX(void);
extern bool_t handle_gimbal_resetY(void);

extern int int_normalized_limit(int input);
extern fp32 fp32_normalized_limit(fp32 input);

#endif
