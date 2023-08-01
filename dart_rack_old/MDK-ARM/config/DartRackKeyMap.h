#ifndef _INFANTRY4_KEY_MAP_
#define _INFANTRY4_KEY_MAP_

#include "Remote.h"
#define YAW_REMOTE_SENS   0.35f
#define PITCH_REMOTE_SENS 0.1f
#define YAW_MOUSE_SENS    50
#define PITCH_MOUSE_SENS  30

/* 灵敏度设置 */
#define REMOTE_CONTROL_SENSE             0.003f
#define KEYBOARD_CONTROL_SENSE           0.001f
#define ANGLE_CONTROL_SENSE              0.35f
#define SPEED_CONTROL_FIRST_LEVEL_SENSE  10.0f
#define SPEED_CONTROL_SECOND_LEVEL_SENSE 100.0f
/* 控制状态 */

// 飞镖架控制指令
// YAW轴电机
#define GIMBAL_CMD_YAW_COARSE_KEYMAP intNormalizedLimit(HandleGimbalRightside() - HandleGimbalLeftside() + CheakKeyPress(KEY_PRESSED_OFFSET_V) - CheakKeyPress(KEY_PRESSED_OFFSET_C))
#define GIMBAL_CMD_YAW_SLIGHT_KEYMAP fp32NormalizedLimit(ANGLE_CONTROL_SENSE *(HandleChassisLeftside() - HandleChassisRightside() + CheakKeyPress(KEY_PRESSED_OFFSET_Z) - CheakKeyPress(KEY_PRESSED_OFFSET_X)))
// PITCH轴电机
#define GIMBAL_CMD_PITCH_TEST_KEYMAP fp32NormalizedLimit(REMOTE_CONTROL_SENSE *(HandleGimbalUpside() - HandleGimbalDownside()))
/* 链条电机 */
#define SHOOT_CMD_CHAIN_KEYMAP fp32NormalizedLimit(RemoteChannalLeftX())
/* 摩擦轮电机 */
#define SHOOT_CMD_FRICTION_KEYMAP              fp32NormalizedLimit(RemoteDial())
#define SHOOT_CMD_FRICTION_FIRST_LEVEL_KEYMAP  SPEED_CONTROL_FIRST_LEVEL_SENSE *(fp32NormalizedLimit(HandleChassisUpside() - HandleChassisDownside() + CheakKeyPress(KEY_PRESSED_OFFSET_SHIFT) - CheakKeyPress(KEY_PRESSED_OFFSET_CTRL)))
#define SHOOT_CMD_FRICTION_SECOND_LEVEL_KEYMAP SPEED_CONTROL_SECOND_LEVEL_SENSE *(fp32NormalizedLimit(HandleChassisUpside() - HandleChassisDownside() + CheakKeyPress(KEY_PRESSED_OFFSET_SHIFT) - CheakKeyPress(KEY_PRESSED_OFFSET_CTRL)))
// 飞镖发射
#define SHOOT_CMD_LAUCH_KEYMAP (HandleDial() || CheakKeyPress(KEY_PRESSED_OFFSET_G))

#endif
