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
    fp32 o_alpha;
    fp32 delta;
    fp32 alpha;
    fp32 dead_area;
    fp32 dead_area_stable;
    // PID
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //
    fp32 max_iout; //

    fp32 last_set;
    fp32 set;
    fp32 fdb;

    fp32 last_out;
    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //
    fp32 error[3]; //

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
    /*  */
    fp32 r1;
    fp32 r2;
    fp32 sampling_period; //
    fp32 tracking_delta;  //

    /* */
    fp32 e1;
    fp32 e2;
    fp32 k1;
    fp32 k2;

    /*  */
    fp32 w0; //
    fp32 b;  //
    fp32 z1;
    fp32 z2;
    fp32 z3;
    fp32 observe_beta1;
    fp32 observe_beta2;
    fp32 observe_beta3;

    fp32 max_out; //

    fp32 last_set;
    fp32 set;
    fp32 fdb;

    fp32 last_out;
    fp32 out;
} ladrc_type_def;

typedef struct
{
    /*  */
    fp32 r1;
    fp32 r2;
    fp32 sampling_period; //
    fp32 tracking_delta;  //

    /*  */
    fp32 e1;
    fp32 e2;
    fp32 out_alpha1;
    fp32 out_alpha2;
    fp32 out_beta1;
    fp32 out_beta2;
    fp32 out_delta;

    /*  */
    fp32 w0; //
    fp32 b;  //
    fp32 z1;
    fp32 z2;
    fp32 z3;
    fp32 observe_delta;
    fp32 observe_beta1;
    fp32 observe_beta2;
    fp32 observe_beta3;
    fp32 observe_alpha1;
    fp32 observe_alpha2;

    fp32 max_out; //

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

    fp32 max_out;

    fp32 last_set;
    fp32 set;
    fp32 fdb;

    fp32 last_out;
    fp32 out;
} efc_type_def;

typedef struct
{
    bool_t flag;
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t torque;
    int8_t temperature;
    int32_t dist_ecd;
    int16_t last_ecd;
} motor_measure_t;

typedef struct
{
    int16_t angle;
    int16_t loops;
    int16_t angular_velocity;
    int16_t temperature;
} angle_encoder_t;

typedef struct
{
    fp32 yaw_angle;
    fp32 yaw_speed;
    fp32 pitch_speed;
    fp32 pitch_dist;
} GimbalMotorMeasure_t;

typedef struct
{
    fp32 drag_left_speed;
    fp32 drag_right_speed;
    fp32 load_speed;
    fp32 adjust_speed;
    fp32 drag_left_dist;
    fp32 drag_right_dist;
    fp32 load_dist;
    fp32 adjust_dist;
} ShootMotorMeasure_t;

typedef struct
{
    GimbalMotorMeasure_t gimbal_motor_measure;
    ShootMotorMeasure_t shoot_motor_measure;
} DartRackMotorMeasure_t;

typedef struct
{
    fp32 angle;
    fp32 last_angle;
    int16_t loops;
    fp32 angular_velocity;
} DratRackAngleEncoder_t;

typedef struct
{
    DratRackAngleEncoder_t yaw_angle_encoder;
    DratRackAngleEncoder_t pitch_angle_encoder;
} DartRackEncoder_t;

#endif
