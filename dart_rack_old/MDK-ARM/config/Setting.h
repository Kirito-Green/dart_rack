#ifndef SETTING_H
#define SETTING_H

/* ģ���������ʱ��? */
#define MOTOR_OFFLINE_TIMEMAX         50
#define ANGLE_MEASURE_OFFLINE_TIMEMAX 100
#define REMOTE_OFFLINE_TIMEMAX        300
#define REFEREE_OFFLINE_TIMEMAX       3000

/* �Ƕȱ�������Ϣ */
#define ZERO_POINT                  0.0f
#define CIRCLE_ANGLE                360.0f
#define ANGLE_ENCODER_MEASURE_MAX   32768.0f
#define ANGLE_ENCODER_SAMPLING_TIME 0.1f
#define ANGLE_ENCODER_ZERO_VALOCITY 0.109863f

/* ���ڽṹ��Ϣ */
#define DART_RACK_NUM 4
#define DART_LENGTH   0.171f

/* ���ڿ���ʱ�� */
#define DART_RACK_RESET_TIME   2000
#define DART_RACK_AIM_TIME     2000
#define DART_RACK_LAUCH_TIME   5000
#define DART_RACK_SHELTER_TIME 10
#define RACK1_LAUCH_TIME       6000
#define RACK2_LAUCH_TIME       8000
#define RACK3_LAUCH_TIME       7000
#define RACK4_LAUCH_TIME       8000

/* ǰ��վ����λ�� */
#define DART_RACK_AIM_STRAIGHT 0
#define DART_RACK_AIM_TOWER    -1
#define DART_RACK_AIM_BASE     1
#define YAW_ZERO_ANGLE         180.0f
#define YAW_MIN_ANGLE          165.0f
#define YAW_MAX_ANGLE          195.0f
// #define YAW_TOWER_ANGLE                  (YAW_ZERO_ANGLE + 6.5f)
// #define YAW_BASE_ANGLE                   (YAW_ZERO_ANGLE -7.3f)
#define YAW_TOWER_MIN_ANGLE (YAW_ZERO_ANGLE + 6.5f - 2.0f)
#define YAW_TOWER_MAX_ANGLE (YAW_ZERO_ANGLE + 6.5f + 2.0f)
#define YAW_BASE_MIN_ANGLE  (YAW_ZERO_ANGLE - 7.3f - 2.0f)
#define YAW_BASE_MAX_ANGLE  (YAW_ZERO_ANGLE - 7.3f + 2.0f)
#define PITCH_TARGET_ANGLE  37.0f
#define PITCH_MIN_ANGLE     30.0f
#define PITCH_MAX_ANGLE     45.0f

/*����źŲɼ�����? */
#define MIN_MEASURE_TIME       1000
#define PHOTOELETRICITY_PERIOD 0.0001f
#define VOLTAGE_JUDEG_POINT    1.0f

/* ���Ʒ�ʽ */
#define PID_CONTROL       RC_SW_UP
#define PID_GIMBAL_STABLE RC_SW_UP
#define MANUAL_CONTROL    RC_SW_UP
#define AUTO_CONTROL      RC_SW_MID
#define PID_GIMBAL_RANGE  RC_SW_MID
#define ADRC_CONTROL      RC_SW_MID
#define PID_SPEED_CONTROL RC_SW_DOWN
#define EFC_CONTROL       RC_SW_DOWN
#define SPEED_CONTROL     RC_SW_DOWN

/* PID���Ʒ�ʽ����ѡ�� */
#define PID_I_DISCREATE     // ���ַ���(��ѡ)
#define I_CONTROL_DISCTEATE 3000.0f
// #define PID_D_DISCREATE                // ΢�ַ���(��ѡ)
#define D_CONTROL_POINT           20.0f
#define INPUT_FILTER_FIRSET_ORDER // һ�������ͨ�˲�?(��ѡ)
#define OUTPUT_FILTER_FIRST_ORDER // һ�������ͨ�˲�?(��ѡ)
#define PID_NON_LINEAR_OUTPUT
#define ANGLE_INIT_TARGET 180.0f
#define SPEED_INIT_TARGET 0.0f

// ���ID����
#define SHOOT_MOTOR1_ID 0x201
#define SHOOT_MOTOR2_ID 0x202
#define SHOOT_MOTOR3_ID 0x203
#define SHOOT_MOTOR4_ID 0x204
#define YAW_MOTOR_ID    0x205
#define PITCH_MOTOR_ID  0x206
#define CHAIN_MOTOR_ID  0x207

// ���ת������װ������?
#define YAW_MOTOR_DIRECTION    -1
#define PITCH_MOTOR_DIRECTION  -1
#define CHAIN_MOTOR_DIRECTION  1
#define SHOOT_MOTOR1_DIRECTION 1
#define SHOOT_MOTOR2_DIRECTION -1
#define SHOOT_MOTOR3_DIRECTION 1
#define SHOOT_MOTOR4_DIRECTION -1

#endif
