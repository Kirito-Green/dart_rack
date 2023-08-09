#ifndef DART_RACK_PARAMETER_H
#define DART_RACK_PARAMETER_H

#include "struct_typedef.h"

// 电机最大输出
#define GM6020_MAX_OUTPUT  30000
#define GM6020_MAX_IOUTPUT 10000
#define M3508_MAX_OUTPUT   16384
#define M3508_MAX_IOUTPUT  6000
#define M2006_MAX_OUTPUT   8000
#define M2006_MAX_IOUTPUT  3000

/* 调试最大速度 */
#define COMMAND_JUDGE_POINT         0.1f
#define MOTOR_DRAG_MAX_TEST_DIST    10.0f
#define MOTOR_LOAD_MAX_TEST_DIST    10.0f
#define MOTOR_ADJUST_MAX_TEST_DIST  10.0f
#define MOTOR_DRAG_MAX_TEST_SPEED   5000.0f
#define MOTOR_LOAD_MAX_TEST_SPEED   5000.0f
#define MOTOR_ADJUST_MAX_TEST_SPEED 5000.0f
#define MOTOR_CHANGE_SENSE          0.1f

// 比赛速度控制
#define YAW_MAX_ANGULAR_VELOCITY    5.0f
#define YAW_MAX_ANGULAR_IVELOCITY   0.5f
#define YAW_MAX_SPEED_OUTPUT        30000.0f
#define YAW_MAX_SPEED_IOUTPUT       10000.0f

#define PITCH_MAX_ANGULAR_VELOCITY  2.0f
#define PITCH_MAX_ANGULAR_IVELOCITY 0.5f
#define PITCH_MAX_SPEED_OUTPUT      3000.0f
#define PITCH_MAX_SPEED_IOUTPUT     2000.0f
#define PITCH_MAX_SPEED             6000.0f
#define PITCH_MAX_ISPEED            3000.0f
#define PITCH_MAX_OUTPUT            6000.0f
#define PITCH_MAX_IOUTPUT           3000.0f

#define DRAG_LEFT_MAX_SPEED         3000.0f
#define DRAG_LEFT_MAX_ISPEED        2000.0f
#define DRAG_LEFT_MAX_OUTPUT        7000.0f
#define DRAG_LEFT_MAX_IOUTPUT       4000.0f

#define DRAG_RIGHT_MAX_SPEED        3000.0f
#define DRAG_RIGHT_MAX_ISPEED       2000.0f
#define DRAG_RIGHT_MAX_OUTPUT       7000.0f
#define DRAG_RIGHT_MAX_IOUTPUT      4000.0f

#define LOAD_MAX_SPEED              8000.0f
#define LOAD_MAX_ISPEED             4000.0f
#define LOAD_MAX_OUTPUT             8000.0f
#define LOAD_MAX_IOUTPUT            4000.0f

#define ADJUST_MAX_SPEED            6000.0f
#define ADJUST_MAX_ISPEED           3000.0f
#define ADJUST_MAX_OUTPUT           6000.0f
#define ADJUST_MAX_IOUTPUT          3000.0f

/* 低通滤波参数 */
#define POSITION_INPUT_ALPHA  0.0f
#define POSITION_OUTPUT_ALPHA 0.2f
#define ANGLE_INPUT_ALPHA     0.0f
#define ANGLE_OUTPUT_ALPHA    0.3f
#define SPEED_INPUT_ALPHA     0.3f
#define SPEED_OUTPUT_ALPHA    0.3f

/* 非线性参数死区 */
#define POSITION_DELTA_DECREASE 1.5f
#define POSITION_ALPHA_DECREASE 1.5f
#define POSITION_DELTA_INCREASE 5.0f
#define POSITION_ALPHA_INCREASE 0.7f
#define ANGLE_DELTA             1.0f
#define ANGLE_ALPHA             0.7f
#define SPEED_DELTA             30.0f
#define SPEED_ALPHA             0.81f
#define POSITION_DEAD_AREA      1.0f
#define ANGLE_DEAD_AREA         0.03f
#define SPEED_DEAD_AREA         0.0f

//* 舵机参数 前面为一 后面为二 */
#define TIM_LEFT_FIRST_COMPARE_UP     210
#define TIM_LEFT_SECOND_COMPARE_UP    220
#define TIM_RIGHT_FIRST_COMPARE_UP    70
#define TIM_RIGHT_SECOND_COMPARE_UP   70
#define TIM_BOTTOM_COMPARE_UP         110
#define TIM_TOP_COMPARE_UP            150
#define TIM_TOP_COMPARE_MID           80
#define TIM_LEFT_FIRST_COMPARE_DOWN   50
#define TIM_LEFT_SECOND_COMPARE_DOWN  50
#define TIM_RIGHT_FIRST_COMPARE_DOWN  235
#define TIM_RIGHT_SECOND_COMPARE_DOWN 235
#define TIM_BOTTOM_COMPARE_DOWN       200
#define TIM_TOP_COMPARE_DOWN          50
/* YAW */

#define YAW_ANGLE_OPERATE_KP 8.0f
#define YAW_ANGLE_OPERATE_KI 0.0f
#define YAW_ANGLE_OPERATE_KD 0.0f
fp32 YAW_ANGLE_OPERATE[3] = {YAW_ANGLE_OPERATE_KP, YAW_ANGLE_OPERATE_KI, YAW_ANGLE_OPERATE_KD};

// #define YAW_SPEED_OPERATE_KP 6000.0f
// #define YAW_SPEED_OPERATE_KI 0.0f
// #define YAW_SPEED_OPERATE_KD 1200.0f
#define YAW_SPEED_OPERATE_KP 3000.0f
#define YAW_SPEED_OPERATE_KI 0.0f
#define YAW_SPEED_OPERATE_KD 800.0f
fp32 YAW_SPEED_OPERATE[3] = {YAW_SPEED_OPERATE_KP, YAW_SPEED_OPERATE_KI, YAW_SPEED_OPERATE_KD};

/* PITCH */
#define PITCH_ANGLE_OPERATE_KP 500.0f
#define PITCH_ANGLE_OPERATE_KI 0.0f
#define PITCH_ANGLE_OPERATE_KD 0.0f
fp32 PITCH_ANGLE_OPERATE[3] = {PITCH_ANGLE_OPERATE_KP, PITCH_ANGLE_OPERATE_KI, PITCH_ANGLE_OPERATE_KD};

#define PITCH_SPEED_OPERATE_KP 50.0f
#define PITCH_SPEED_OPERATE_KI 0.0f
#define PITCH_SPEED_OPERATE_KD 10.0f
fp32 PITCH_SPEED_OPERATE[3] = {PITCH_SPEED_OPERATE_KP, PITCH_SPEED_OPERATE_KI, PITCH_SPEED_OPERATE_KD};

/* ---------------------------------- 发射电机 ---------------------------------- */
/* 位置参数 */

#define DRAG_LEFT_MIN_POSIITON  -192.0f
#define DRAG_LEFT_MAX_POSIION   850.0f
#define DRAG_RIGHT_MIN_POSITION -850.0f
#define DRAG_RIGHT_MAX_POSITION 192.0f
#define LOAD_MIN_POSITION       -511.0f
#define LOAD_MAX_POSITION       0.0f
#define ADJUST_MIN_POSITION     0.0f
#define ADJUST_MAX_POSIITON     265.0f
// 装填电机位置参数
#define LOAD_FIRST_POSITION  ZERO_POINT
#define LOAD_SECOND_POSITION -262.0f //(LOAD_MIN_POSITION / 2.0f)
#define LOAD_THIRD_POSITION  LOAD_MIN_POSITION
#define LOAD_FOURTH_POSITION LOAD_MIN_POSITION
#define LOAD_RETURN_DIST     60.0f
// fp32 LOAD_POSITION[4] = {-100.0f, -200.0f, -300.0f, -400.0f};
fp32 LOAD_POSITION[4] = {LOAD_FIRST_POSITION, LOAD_SECOND_POSITION, LOAD_THIRD_POSITION, LOAD_FOURTH_POSITION};
// 左拖拽电机位置参
#define DRAG_LEFT_SAFE_POSITION  DRAG_LEFT_MIN_POSIITON
#define DRAG_LEFT_WAIT_POSITION  363.0f
#define DRAG_LEFT_TOWER_POSITION 650.0f
#define DRAG_LEFT_BASE_POSITION  700.0f
// 右拖拽电机位置参数
#define DRAG_RIGHT_SAFE_POSITION  DRAG_RIGHT_MAX_POSITION
#define DRAG_RIGHT_WAIT_POSITION  (-DRAG_LEFT_WAIT_POSITION)
#define DRAG_RIGHT_TOWER_POSITION (-DRAG_LEFT_TOWER_POSITION)
#define DRAG_RIGHT_BASE_POSITION  (-DRAG_LEFT_BASE_POSITION)
// 调整电机位置参数
#define ADJUST_TOWER_POSITION 3.0f
#define ADJUST_BASE_POSITION  15.0f
// 允许误差
#define PITCH_POSITION_THRESHOLD      0.1f
#define LOAD_POSITION_THRESHOLD       1.5f
#define DRAG_LEFT_POSITION_THRESHOLD  10.0f
#define DRAG_RIGHT_POSITION_THRESHOLD 10.0f
#define ADJUST_POSITION_THRESHOLD     1.0f
// 控制最大时间参数
#define DART_RACK_SLIP_MAX_TIME   500
#define DART_RACK_SHOOT_MAX_TIME  1500
#define SERVO_LOAD_MAX_TIME       1200
#define SERVO_LAUNCH_MAX_TIME     1500
#define SERVO_BLOCK_MOVE_MAX_TIME 800
#define SERVO_BLOCK_WAIT_MAX_TIME 200
#define MOTOR_LOAD_MAX_TIME       1200
#define MOTOR_DRAG_MAX_TIME       3000
#define MOTOR_ADJUST_MAX_TIME     7000
#define MOTOR_DRAG_TIME_MEASURE   5
#define MOTOR_LOAD_TIME_MEASURE   24
#define MOTOR_ADJUST_TIME_MEASURE 280

// 拖拽电机
//* Decrease params
#define DRAG_LEFT_POSITION_OPERATE_KP 5.0f
#define DRAG_LEFT_POSITION_OPERATE_KI 0.0f
#define DRAG_LEFT_POSITION_OPERATE_KD 0.0f
fp32 DRAG_LEFT_POSITION_OPERATE[3] = {DRAG_LEFT_POSITION_OPERATE_KP, DRAG_LEFT_POSITION_OPERATE_KI, DRAG_LEFT_POSITION_OPERATE_KD};

#define DRAG_LEFT_SPEED_OPERATE_KP 50.0f
#define DRAG_LEFT_SPEED_OPERATE_KI 0.0f
#define DRAG_LEFT_SPEED_OPERATE_KD 10.0f
fp32 DRAG_LEFT_SPEED_OPERATE[3] = {DRAG_LEFT_SPEED_OPERATE_KP, DRAG_LEFT_SPEED_OPERATE_KI, DRAG_LEFT_SPEED_OPERATE_KD};

#define DRAG_RIGHT_POSITION_OPERATE_KP 5.0f
#define DRAG_RIGHT_POSITION_OPERATE_KI 0.0f
#define DRAG_RIGHT_POSITION_OPERATE_KD 0.0f
fp32 DRAG_RIGHT_POSITION_OPERATE[3] = {DRAG_RIGHT_POSITION_OPERATE_KP, DRAG_RIGHT_POSITION_OPERATE_KI, DRAG_RIGHT_POSITION_OPERATE_KD};

#define DRAG_RIGHT_SPEED_OPERATE_KP 50.0f
#define DRAG_RIGHT_SPEED_OPERATE_KI 0.0f
#define DRAG_RIGHT_SPEED_OPERATE_KD 10.0f
fp32 DRAG_RIGHT_SPEED_OPERATE[3] = {DRAG_RIGHT_SPEED_OPERATE_KP, DRAG_RIGHT_SPEED_OPERATE_KI, DRAG_RIGHT_SPEED_OPERATE_KD};

//* Increase params
// #define DRAG_LEFT_POSITION_OPERATE_KP 200.0f
// #define DRAG_LEFT_POSITION_OPERATE_KI 0.0f
// #define DRAG_LEFT_POSITION_OPERATE_KD 0.0f
// fp32 DRAG_LEFT_POSITION_OPERATE[3] = {DRAG_LEFT_POSITION_OPERATE_KP, DRAG_LEFT_POSITION_OPERATE_KI, DRAG_LEFT_POSITION_OPERATE_KD};

// #define DRAG_LEFT_SPEED_OPERATE_KP 50.0f
// #define DRAG_LEFT_SPEED_OPERATE_KI 0.0f
// #define DRAG_LEFT_SPEED_OPERATE_KD 10.0f
// fp32 DRAG_LEFT_SPEED_OPERATE[3] = {DRAG_LEFT_SPEED_OPERATE_KP, DRAG_LEFT_SPEED_OPERATE_KI, DRAG_LEFT_SPEED_OPERATE_KD};

// #define DRAG_RIGHT_POSITION_OPERATE_KP 200.0f
// #define DRAG_RIGHT_POSITION_OPERATE_KI 0.0f
// #define DRAG_RIGHT_POSITION_OPERATE_KD 0.0f
// fp32 DRAG_RIGHT_POSITION_OPERATE[3] = {DRAG_RIGHT_POSITION_OPERATE_KP, DRAG_RIGHT_POSITION_OPERATE_KI, DRAG_RIGHT_POSITION_OPERATE_KD};

// #define DRAG_RIGHT_SPEED_OPERATE_KP 50.0f
// #define DRAG_RIGHT_SPEED_OPERATE_KI 0.0f
// #define DRAG_RIGHT_SPEED_OPERATE_KD 10.0f
// fp32 DRAG_RIGHT_SPEED_OPERATE[3] = {DRAG_RIGHT_SPEED_OPERATE_KP, DRAG_RIGHT_SPEED_OPERATE_KI, DRAG_RIGHT_SPEED_OPERATE_KD};

// 装填电机
#define LOAD_POSITION_OPERATE_KP 5.0f
#define LOAD_POSITION_OPERATE_KI 0.0f
#define LOAD_POSITION_OPERATE_KD 0.0f
fp32 LOAD_POSITION_OPERATE[3] = {LOAD_POSITION_OPERATE_KP, LOAD_POSITION_OPERATE_KI, LOAD_POSITION_OPERATE_KD};

#define LOAD_SPEED_OPERATE_KP 3.0f
#define LOAD_SPEED_OPERATE_KI 0.06f
#define LOAD_SPEED_OPERATE_KD 0.15f
fp32 LOAD_SPEED_OPERATE[3] = {LOAD_SPEED_OPERATE_KP, LOAD_SPEED_OPERATE_KI, LOAD_SPEED_OPERATE_KD};

// 行程调节电机
#define ADJUST_POSITION_OPERATE_KP 500.0f
#define ADJUST_POSITION_OPERATE_KI 0.0f
#define ADJUST_POSITION_OPERATE_KD 0.0f
fp32 ADJUST_POSITION_OPERATE[3] = {ADJUST_POSITION_OPERATE_KP, ADJUST_POSITION_OPERATE_KI, ADJUST_POSITION_OPERATE_KD};

#define ADJUST_SPEED_OPERATE_KP 20.0f
#define ADJUST_SPEED_OPERATE_KI 0.0f
#define ADJUST_SPEED_OPERATE_KD 0.0f
fp32 ADJUST_SPEED_OPERATE[3] = {ADJUST_SPEED_OPERATE_KP, ADJUST_SPEED_OPERATE_KI, ADJUST_SPEED_OPERATE_KD};

#define DRAG_LEFT_OPERATE_KP 2.0f
#define DRAG_LEFT_OPERATE_KI 0.06f
#define DRAG_LEFT_OPERATE_KD 0.1f
fp32 DRAG_LEFT_OPERATE[3] = {DRAG_LEFT_OPERATE_KP, DRAG_LEFT_OPERATE_KI, DRAG_LEFT_OPERATE_KD};

#define DRAG_RIGHT_OPERATE_KP 1.5f
#define DRAG_RIGHT_OPERATE_KI 0.06f
#define DRAG_RIGHT_OPERATE_KD 0.1f
fp32 DRAG_RIGHT_OPERATE[3] = {DRAG_RIGHT_OPERATE_KP, DRAG_RIGHT_OPERATE_KI, DRAG_RIGHT_OPERATE_KD};

#define LOAD_OPERATE_KP 1.5f
#define LOAD_OPERATE_KI 0.0f
#define LOAD_OPERATE_KD 0.1f
fp32 LOAD_OPERATE[3] = {LOAD_OPERATE_KP, LOAD_OPERATE_KI, LOAD_OPERATE_KD};

#define ADJUST_OPERATE_KP 2.0f
#define ADJUST_OPERATE_KI 0.06f
#define ADJUST_OPERATE_KD 0.1f
fp32 ADJUST_OPERATE[3] = {ADJUST_OPERATE_KP, ADJUST_OPERATE_KI, ADJUST_OPERATE_KD};

/*-----------------------------------------------------------*/
/* ADRC */
#define ADRC_DEFAULT_SAMPLING_PERIOD 0.01f
#define ADRC_DEFAULT_TRACING_DELTA   40.0f
#define ADRC_DEFAULT_W0              20.0f
#define ADRC_DEFAULT_B               5.0f
#define ADRC_DEFAULT_K1              50.0f
#define ADRC_DUFAULT_K2              20.0f
fp32 LADRC_DEFAULT_OPERATE[6] = {ADRC_DEFAULT_SAMPLING_PERIOD,
                                 ADRC_DEFAULT_TRACING_DELTA,
                                 ADRC_DEFAULT_W0,
                                 ADRC_DEFAULT_B,
                                 ADRC_DEFAULT_K1,
                                 ADRC_DUFAULT_K2};
#define ADRC_SHOOT_SAMPLING_PERIOD 0.01f
#define ADRC_SHOOT_TRACING_DELTA   40.0f
#define ADRC_SHOOT_W0              20.0f
#define ADRC_SHOOT_B               3.0f
#define ADRC_SHOOT_OBSERVE_DELTA   0.01f
#define ADRC_SHOOT_OBSERVE_ALPHA1  0.5f
#define ADRC_SHOOT_OBSERVE_ALPHA2  0.25f
#define ADRC_SHOOT_OUT_DELTA       0.01f
#define ADRC_SHOOT_OUT_ALPHA1      0.75f
#define ADRC_SHOOT_OUT_ALPHA2      1.5f
#define ADRC_SHOOT_OUT_BETA1       200.0f
#define ADRC_SHOOT_OUT_BETA2       1.0f
fp32 ADRC_SHOOT_DEFAULT[12] = {ADRC_SHOOT_SAMPLING_PERIOD,
                               ADRC_SHOOT_TRACING_DELTA,
                               ADRC_SHOOT_W0,
                               ADRC_SHOOT_B,
                               ADRC_SHOOT_OBSERVE_DELTA,
                               ADRC_SHOOT_OBSERVE_ALPHA1,
                               ADRC_SHOOT_OBSERVE_ALPHA2,
                               ADRC_SHOOT_OUT_DELTA,
                               ADRC_SHOOT_OUT_ALPHA1,
                               ADRC_SHOOT_OUT_ALPHA2,
                               ADRC_SHOOT_OUT_BETA1,
                               ADRC_SHOOT_OUT_BETA2};

#define oADRC_SAMPLING_PERIOD 0.03f
#define oADRC_TRACING_DELTA   60.0f
#define oADRC_W0              2.0f
#define oADRC_B               130.0f
#define oADRC_K1              1.0f
#define oADRC_K2              0.0f
fp32 LADRC_ANGLE_OPERATE[6] = {oADRC_SAMPLING_PERIOD,
                               oADRC_TRACING_DELTA,
                               oADRC_W0,
                               oADRC_B,
                               oADRC_K1,
                               oADRC_K2};
#define iADRC_SAMPLING_PERIOD 0.01f
#define iADRC_TRACING_DELTA   80.0f
#define iADRC_W0              10.0f
#define iADRC_B               15.0f
#define iADRC_K1              10.0f
#define iADRC_K2              0.1f
fp32 LADRC_SPEED_OPERATE[6] = {iADRC_SAMPLING_PERIOD,
                               iADRC_TRACING_DELTA,
                               iADRC_W0,
                               iADRC_B,
                               iADRC_K1,
                               iADRC_K2};

#endif
