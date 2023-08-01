#ifndef __INFANTRY4_PARAMETER_H
#define __INFANTRY4_PARAMETER_H

#include "struct_typedef.h"

// 电机最大输出
#define GM6020_MAX_OUTPUT  30000
#define GM6020_MAX_IOUTPUT 10000
#define M3508_MAX_OUTPUT   16384
#define M3508_MAX_IOUTPUT  6000
#define M2006_MAX_OUTPUT   8000
#define M2006_MAX_IOUTPUT  3000

/* 电机调试最大速度 */
#define COMMAND_JUDGE_POINT   0.1f
#define MOTOR_CHAIN_MAX_SPEED 4000.0f
#define MOTOR_SHOOT_MAX_SPEED 2000.0f
#define MOTOR_YAW_MAX_SPEED   1500.0f
#define MOTOR_PITCH_MAX_SPEED 1500.0f
#define MOTOR_CHANGE_SENSE    0.1f

// 电机最大速度
#define YAW_MAX_SPEED               300
#define YAW_MAX_ANGULAR_VELOCITY    2.0f
#define YAW_MAX_ANGULAR_IVELOCITY   0.5f
#define YAW_MAX_OUTPUT              1500
#define PITCH_MAX_SPEED             300
#define PITCH_MAX_ANGULAR_VELOCITY  2.0f
#define PITCH_MAX_ANGULAR_IVELOCITY 0.5f
#define PITCH_MAX_OUTPUT            1500

/* 滤波参数 */
#define ANGLE_INPUT_ALPHA  0.0f
#define ANGLE_OUTPUT_ALPHA 0.0f
#define SPEED_INPUT_ALPHA  0.3f
#define SPEED_OUTPUT_ALPHA 0.3f

/* 差速补偿 */
#define DIFF_DELTA         0.1f
#define DIFF_FIRST_DEGREE  1.0f
#define DIFF_SECOND_DEGREE 1.2f
#define DIFF_FIRST_ALPHA   1.5f
#define DIFF_SECOND_ALPHA  1.5f

/* PID非线性参数 */
#define ANGLE_DELTA 1.0f
#define ANGLE_ALPHA 0.7f
#define SPEED_DELTA 30.0f
#define SPEED_ALPHA 0.81f

/* 其他优化 */
#define ANGLE_DEAD_AREA 0.03f
#define SPEED_DEAD_AREA 0.0f

// 云台运动参数
/* YAW轴电机 */
#define YAW_SPEED_OPERATE_KP 3000.0f
#define YAW_SPEED_OPERATE_KI 0.0f
#define YAW_SPEED_OPERATE_KD 1200.0f
fp32 YAW_SPEED_OPERATE[3] = {YAW_SPEED_OPERATE_KP, YAW_SPEED_OPERATE_KI, YAW_SPEED_OPERATE_KD};

#define YAW_ANGLE_OPERATE_KP 1.5f
#define YAW_ANGLE_OPERATE_KI 0.0f
#define YAW_ANGLE_OPERATE_KD 0.0f
fp32 YAW_ANGLE_OPERATE[3] = {YAW_ANGLE_OPERATE_KP, YAW_ANGLE_OPERATE_KI, YAW_ANGLE_OPERATE_KD};

/* PITCH轴电机 */
#define PITCH_SPEED_OPERATE_KP 5000.0f
#define PITCH_SPEED_OPERATE_KI 0.0f
#define PITCH_SPEED_OPERATE_KD 1200.0f
fp32 PITCH_SPEED_OPERATE[3] = {PITCH_SPEED_OPERATE_KP, PITCH_SPEED_OPERATE_KI, PITCH_SPEED_OPERATE_KD};

#define PITCH_ANGLE_OPERATE_KP 2.0f
#define PITCH_ANGLE_OPERATE_KI 0.0f
#define PITCH_ANGLE_OPERATE_KD 0.0f
fp32 PITCH_ANGLE_OPERATE[3] = {PITCH_ANGLE_OPERATE_KP, PITCH_ANGLE_OPERATE_KI, PITCH_ANGLE_OPERATE_KD};

// 传送带电机参数
#define CHAIN_OPERATE_KP 3.0f
#define CHAIN_OPERATE_KI 0.06f
#define CHAIN_OPERATE_KD 0.0f
fp32 CHAIN_OPERATE[3] = {CHAIN_OPERATE_KP, CHAIN_OPERATE_KI, CHAIN_OPERATE_KD};

//  摩擦轮参数
/* PID参数 */
//  默认速度
#define SHOOT_STOP_KP 13.0f
#define SHOOT_STOP_KI 0.0f
#define SHOOT_STOP_KD 0.0f
fp32 SHOOT_STOP[3] = {SHOOT_STOP_KP, SHOOT_STOP_KI, SHOOT_STOP_KD};
// 编号1摩擦轮
#define SHOOT1_KP 13.0f
#define SHOOT1_KI 0.1f
#define SHOOT1_KD 8.0f
fp32 SHOOT1_READY[3] = {SHOOT1_KP, SHOOT1_KI, SHOOT1_KD};
// 编号2摩擦轮
#define SHOOT2_KP 13.0f
#define SHOOT2_KI 0.1f
#define SHOOT2_KD 8.0f
fp32 SHOOT2_READY[3] = {SHOOT2_KP, SHOOT2_KI, SHOOT2_KD};
// 编号3摩擦轮
#define SHOOT3_KP 13.0f
#define SHOOT3_KI 0.1f
#define SHOOT3_KD 8.0f
fp32 SHOOT3_READY[3] = {SHOOT3_KP, SHOOT3_KI, SHOOT3_KD};
// 编号4摩擦轮
#define SHOOT4_KP 13.0f
#define SHOOT4_KI 0.1f
#define SHOOT4_KD 8.0f
fp32 SHOOT4_READY[3] = {SHOOT4_KP, SHOOT4_KI, SHOOT4_KD};

/*-----------------------------------------------------------*/
/* ADRC参数 */
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

#define ADRC_CHAIN_SAMPLING_PERIOD 0.01f
#define ADRC_CHAIN_TRACING_DELTA   80.0f
#define ADRC_CHAIN_W0              10.0f
#define ADRC_CHAIN_B               30.0f
#define ADRC_CHAIN_K1              30.0f
#define ADRC_CHAIN_K2              5.0f
fp32 LADRC_CHAIN_OPERATE[6] = {ADRC_CHAIN_SAMPLING_PERIOD,
                               ADRC_CHAIN_TRACING_DELTA,
                               ADRC_CHAIN_W0,
                               ADRC_CHAIN_B,
                               ADRC_CHAIN_K1,
                               ADRC_CHAIN_K2};

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
/*-----------------------------------------------------------*/
/* EFC参数 */
#define EFC_DEFAULT_Tw            0.1f
#define FFC_DEFAULT_T_FOF         5.66f
#define EFC_DEFAULT_SAMPLING_TIME 0.005f
#define EFC_DEFAULT_KP            20.0f
#define EFC_DEFAULT_KD            10.0f
#define EFC_DEFAULT_KC            10.0f

/*-----------------------------------------------------------*/

#endif
