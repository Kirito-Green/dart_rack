#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

#include "main.h"

#define FRICTION_WHEEL_NUM 4

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

typedef struct
{
    uint8_t mode;
    fp32 i_alpha;
    fp32 o_alpha; // 滤波
    fp32 delta;   // 非线性参数
    fp32 alpha;
    fp32 dead_area; // 死区
    // PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  // 最大输出
    fp32 max_iout; // 最大积分输出

    fp32 last_set;
    fp32 set;
    fp32 fdb;

    fp32 last_out;
    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  // 微分项 0最新 1上一次 2上上次
    fp32 error[3]; // 误差项 0最新 1上一次 2上上次

} pid_type_def;

typedef struct
{
    pid_type_def pid_inside;
    pid_type_def pid_outside;

    fp32 s_set;
    fp32 s_fdb;
    fp32 v_set;
    fp32 v_fdb;
    fp32 out;
} cascade_pid_t;

typedef struct
{
    /* 微分跟踪器 */
    fp32 r1;
    fp32 r2;
    fp32 sampling_period; // 计算步长(取样周期)
    fp32 tracking_delta;  // 跟踪速度

    /* 线性输出 */
    fp32 e1;
    fp32 e2;
    fp32 k1;
    fp32 k2;

    /* 扩张观测器 */
    fp32 w0; // 观测器带宽
    fp32 b;  // 补偿系数 可能是电流与加速度的线性常数
    fp32 z1;
    fp32 z2;
    fp32 z3;
    fp32 observe_beta1;
    fp32 observe_beta2;
    fp32 observe_beta3;

    fp32 max_out; // 最大输出

    fp32 last_set;
    fp32 set;
    fp32 fdb;

    fp32 last_out;
    fp32 out;
} ladrc_type_def;

typedef struct
{
    /* 微分跟踪器 */
    fp32 r1;
    fp32 r2;
    fp32 sampling_period; // 计算步长(取样周期)
    fp32 tracking_delta;  // 跟踪速度

    /* 非线性输出 */
    fp32 e1;
    fp32 e2;
    fp32 out_alpha1;
    fp32 out_alpha2;
    fp32 out_beta1;
    fp32 out_beta2;
    fp32 out_delta;

    /* 扩张观测器 */
    fp32 w0; // 观测器带宽
    fp32 b;  // 补偿系数 可能是电流与加速度的线性常数
    fp32 z1;
    fp32 z2;
    fp32 z3;
    fp32 observe_delta;
    fp32 observe_beta1;
    fp32 observe_beta2;
    fp32 observe_beta3;
    fp32 observe_alpha1;
    fp32 observe_alpha2;

    fp32 max_out; // 最大输出

    fp32 last_set;
    fp32 set;
    fp32 fdb;

    fp32 last_out;
    fp32 out;
} adrc_type_def;

typedef struct
{
    ladrc_type_def ladrc_outside;
    ladrc_type_def ladrc_inside;

    fp32 s_ref;
    fp32 s_set;
    fp32 v_ref;
    fp32 v_set;
    fp32 out;
} cascade_ladrc_t;

typedef struct
{
    pid_type_def pid_outside;
    ladrc_type_def ladrc_inside;

    fp32 s_ref;
    fp32 s_set;
    fp32 v_ref;
    fp32 v_set;
    fp32 out;
} cascade_pid_ladrc_t;

typedef struct
{
    fp32 Kp;
    fp32 Kd;

    fp32 max_out; // 最大输出

    fp32 last_set;
    fp32 set;
    fp32 fdb;

    fp32 last_out;
    fp32 out;
} efc_type_def;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t torque;
    int8_t temperature;
    int16_t last_ecd;
} motor_measure_t;

typedef struct
{
    int16_t angle;
    int16_t loops;
    int16_t angular_velocity;
    int16_t temperature;
} angle_encoder_t;

/* 电机编号位置
   4 3
   2 1
*/

typedef struct {
    fp32 YawMotorAngle;
    fp32 YawMotorSpeed;
    fp32 PitchMotorAngle;
    fp32 PitchMotorSpeed;
} GimbalMotorMeasure_t;

typedef struct {
    fp32 ChainMotorSpeed;
    fp32 ShootMotorSpeed[FRICTION_WHEEL_NUM];
    fp32 ShootMotorSpeedTest[FRICTION_WHEEL_NUM];
    fp32 ShootMotorSpeedDiff[2];
} ShootMotorMeasure_t;

typedef struct {
    GimbalMotorMeasure_t GimbalMotorMeasure;
    ShootMotorMeasure_t ShootMotorMeasure;
} DartRackMotorMeasure_t;

typedef struct {
    fp32 Angle;
    fp32 LastAngle;
    int16_t loops;
    fp32 AnguarVelocity;
} DratRackAngleEncoder_t;

typedef struct {
    DratRackAngleEncoder_t YawAngleEncoder;
    DratRackAngleEncoder_t PitchAngleEncoder;
} DartRackEncoder_t;

#endif
