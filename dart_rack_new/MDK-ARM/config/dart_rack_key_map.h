#ifndef _INFANTRY4_KEY_MAP_
#define _INFANTRY4_KEY_MAP_

#define YAW_REMOTE_SENS   0.35f
#define PITCH_REMOTE_SENS 0.1f
#define YAW_MOUSE_SENS    50
#define PITCH_MOUSE_SENS  30

/*  */
#define REMOTE_CONTROL_SENSE             0.2f
#define KEYBOARD_CONTROL_SENSE           0.001f
#define ANGLE_CONTROL_SENSE              0.25f // 7cm
#define DIST_CONTROL_SENSE               2.0f
#define SPEED_CONTROL_FIRST_LEVEL_SENSE  10.0f
#define SPEED_CONTROL_SECOND_LEVEL_SENSE 100.0f
/* ? */

//
#define SERVO_UP                        1
#define SERVO_MID                       2
#define SERVO_DOWN                      -1
#define DIST_CONTROL_FIRST_LEVEL_SENSE  1.0f
#define DIST_CONTROL_SECOND_LEVEL_SENSE 5.0f
//* 电机复位
#define SHOOT_CMD_RESET_KEYMAP (handle_gimbal_right() && handle_gimbal_up())
#define SHOOT_CMD_ZERO_CONTROL (handle_chassis_resetX() && handle_chassis_resetY() && handle_gimbal_resetX() && handle_gimbal_resetY())
//* 舵机控制
#define SHOOT_CMD_TEST_SERVO_LOAD_KEYMAP   int_normalized_limit(handle_gimbal_right() - handle_gimbal_left())
#define SHOOT_CMD_TEST_SERVO_LAUNCH_KEYMAP int_normalized_limit(handle_gimbal_up() - handle_gimbal_down())
#define SHOOT_CMD_TEST_SERVO_BLOCK_KEYMAP  (check_key_press(KEY_PRESSED_OFFSET_D) + 2 * check_key_press(KEY_PRESSED_OFFSET_S) - check_key_press(KEY_PRESSED_OFFSET_A))
//* 电机控制
#define SHOOT_CMD_TEST_DRAG_KEYMAP    (fp32_normalized_limit(remote_dial()))
#define SHOOT_CMD_TEST_LOAD_KEYMAP    (fp32_normalized_limit(remote_channel_leftX()))
#define SHOOT_CMD_TEST_ADJUST_KEYMAP  (fp32_normalized_limit(remote_channel_leftY()))
#define SHOOT_CMD_MATCH_ADJUST_KEYMAP (int_normalized_limit(handle_chassis_down() - handle_chassis_up() + check_key_press(KEY_PRESSED_OFFSET_SHIFT) - check_key_press(KEY_PRESSED_OFFSET_CTRL)))
//* 云台控制
// yaw
#define GIMBAL_CMD_YAW_COARSE_KEYMAP int_normalized_limit(handle_gimbal_right() - handle_gimbal_left() + check_key_press(KEY_PRESSED_OFFSET_V) - check_key_press(KEY_PRESSED_OFFSET_C))
#define GIMBAL_CMD_YAW_SLIGHT_KEYMAP fp32_normalized_limit(ANGLE_CONTROL_SENSE *(handle_chassis_left() - handle_chassis_right() + check_key_press(KEY_PRESSED_OFFSET_Z) - check_key_press(KEY_PRESSED_OFFSET_X)))
// pitch
#define GIMBAL_CMD_PITCH_SLIGHT_KEYMAP fp32_normalized_limit(DIST_CONTROL_SENSE *(handle_chassis_up() - handle_chassis_down()))
//* 发射指令
#define SHOOT_CMD_MATCH_LOAD_KEYMAP   int_normalized_limit(handle_gimbal_up())
#define SHOOT_CMD_MATCH_LAUNCH_KEYMAP int_normalized_limit(handle_gimbal_down() || check_key_press(KEY_PRESSED_OFFSET_G))

#endif
