#ifndef SETTING_H
#define SETTING_H

#define TFCARD_READ_TIMES_MAX 3

#define POSITIVE              1
#define ZERO                  0
#define NEGTIVE               -1
/* ����������ʱ */
#define MOTOR_OFFLINE_TIMEMAX         50
#define ANGLE_MEASURE_OFFLINE_TIMEMAX 100
#define REMOTE_OFFLINE_TIMEMAX        300
#define REFEREE_OFFLINE_TIMEMAX       3000

/* ������ٱ� */
#define M2006_SLOW_RATE 36.0f
#define M3508_SLOW_RATE 19.2f

/*ת���ܳ�mm*/
#define MOTOR_MAX_ECD       8191.0f
#define M2006_CIRCUMFERENCE 78.539f
#define M3508_CIRCUMFERENCE 135.09f
#define LEAD_SCREW_PITCH    2.0f

/* �������� */
#define ZERO_POINT                  0.0f
#define CIRCLE_ANGLE                360.
#define ANGLE_ENCODER_MEASURE_MAX   32768.0f
#define ANGLE_ENCODER_SAMPLING_TIME 0.1f
#define ANGLE_ENCODER_ZERO_VALOCITY 0.109863f

/* ���ڽṹ���� */
#define DART_RACK_NUM 4
#define DART_LENGTH   0.171f

/* �ǶȲ��� */
#define DART_RACK_AIM_RESET 0
#define DART_RACK_AIM_TOWER -1
#define DART_RACK_AIM_BASE  1
#define YAW_ZERO_ANGLE      180.0f
#define YAW_MIN_ANGLE       165.0f
#define YAW_MAX_ANGLE       195.0f
#define YAW_JUDGE_ANGLE     90.0f // �����ܵĽǶ�
#define YAW_TOWER_ANGLE     (YAW_ZERO_ANGLE + 6.5f)
#define YAW_BASE_ANGLE      (YAW_ZERO_ANGLE - 7.3f)
#define YAW_TOWER_MIN_ANGLE (YAW_ZERO_ANGLE + 6.5f - 2.0f)
#define YAW_TOWER_MAX_ANGLE (YAW_ZERO_ANGLE + 6.5f + 2.0f)
#define YAW_BASE_MIN_ANGLE  (YAW_ZERO_ANGLE - 7.3f - 2.0f)
#define YAW_BASE_MAX_ANGLE  (YAW_ZERO_ANGLE - 7.3f + 2.0f)
#define PITCH_TARGET_ANGLE  37.0f
#define PITCH_MIN_ANGLE     30.0f
#define PITCH_MAX_ANGLE     45.0f
#define PITCH_TARGET_DIST   0.0f
#define PITCH_MIN_DIST      -10.0f
#define PITCH_MAX_DIST      10.0f
#define PITCH_JUDGE_DIST    10.0f

/* ���Ʒ�ʽ */
#define GIMBAL_CONTROL            RC_SW_UP
#define SHOOT_SPEED_CONTROL       RC_SW_MID
#define SHOOT_POSITION_CONTROL    RC_SW_DOWN
#define SHOOT_FULL_CONTROL        RC_SW_UP
#define SHOOT_HALF_CONTROL        RC_SW_MID
#define DIST_CONTROL_FIRST_LEVEL  RC_SW_UP
#define DIST_CONTROL_SECOND_LEVEL RC_SW_MID

/* ����Ų� */
#define MIN_MEASURE_TIME       1000
#define PHOTOELETRICITY_PERIOD 0.0001f
#define VOLTAGE_JUDEG_POINT    1.0f

/* PID���� */
#define PID_I_DISCREATE     //
#define I_CONTROL_DISCTEATE 3000.0f
// #define PID_D_DISCREATE                //
#define D_CONTROL_POINT           20.0f
#define INPUT_FILTER_FIRSET_ORDER //
#define OUTPUT_FILTER_FIRST_ORDER //
#define PID_NON_LINEAR_OUTPUT
#define ANGLE_INIT_TARGET 180.0f
#define SPEED_INIT_TARGET 0.0f

/* ���ID */
#define DRAG_LEFT_MOTOR_ID  0x201
#define DRAG_RIGHT_MOTOR_ID 0x202
#define LOAD_MOTOR_ID       0x203
#define ADJUST_MOTOR_ID     0x204
#define YAW_MOTOR_ID        0x205
#define PITCH_MOTOR_ID      0x206
#define CHAIN_MOTOR_ID      0x207

/* �����װ���� */
#define YAW_MOTOR_DIRECTION        -1
#define PITCH_MOTOR_DIRECTION      -1
#define DRAG_LEFT_MOTOR_DIRECTION  1
#define DRAG_RIGHT_MOTOR_DIRECTION -1
#define LOAD_MOTOR_DIRECTION       -1
#define ADJUST_MOTOR_DIRECTION     1

#endif
