#ifndef LAUNCH_THREAD_H
#define LAUNCH_THREAD_H

#include "struct_typedef.h"

typedef struct
{
    pid_type_def drag_left;
    pid_type_def drag_right;
    pid_type_def load;
    pid_type_def adjust;
    cascade_pid_t cascade_drag_left;
    cascade_pid_t cascade_drag_right;
    cascade_pid_t cascade_load;
    cascade_pid_t cascade_adjust;
    cascade_pid_t yaw;
    cascade_pid_t pitch;
} DartRackPid_t;

typedef struct
{
    ladrc_type_def chain;
    // ladrc_type_def shoot[4];
    adrc_type_def shoot[4];
} DartRackAdrc_t;

typedef struct
{
    efc_type_def chain;
    efc_type_def shoot[4];
    efc_type_def yaw;
    efc_type_def pitch;
} DartRackEfc_t;

typedef struct
{
    cascade_pid_ladrc_t pitch;
    cascade_pid_ladrc_t yaw;
} DartRackPidAdrc_t;

typedef struct
{
    fp32 drag_left;
    fp32 drag_right;
    fp32 load;
    fp32 adjust;
    fp32 yaw;
    fp32 pitch;
} DartRackTarget_t;

typedef struct
{
    fp32 drag_left;
    fp32 drag_right;
    fp32 load;
    fp32 adjust;
    fp32 yaw;
    fp32 pitch;
} DartRackCommand_t;

typedef struct
{
    int16_t drag_left;
    int16_t drag_right;
    int16_t load;
    int16_t adjust;
    int16_t yaw;
    int16_t pitch;
} DartRackOutput_t;

typedef enum {
    DART_RACK_NO_FORCE = 0x00,
    DART_RACK_INIT,
    DART_RACK_TEST,
    DART_RACK_MATCH,
} DartRackStateMachine_e;

typedef struct
{
    DartRackPid_t pid;
    DartRackAdrc_t adrc;
    DartRackMotorMeasure_t motor_measure;
    DartRackEncoder_t encoder;
    DartRackTarget_t target;
    DartRackCommand_t command;
    DartRackOutput_t output;
    DartRackStateMachine_e state;
} DartRack_t;

//* 飞镖发射状态机
typedef enum {
    DART_RACK_SHOOT_READY = 0x00,
    DART_RACK_SHOOT_ADJUST,
    DART_RACK_SHOOT_LOAD,
    DART_RACK_SHOOT_MOVE,
    DART_RACK_SHOOT_ON_FIRE,
    DART_RACK_SHOOT_LAUNCH,
} DartRackShootState_e;

//* 装填舵机状态机
typedef enum {
    SERVO_LOAD_READY = 0X00,
    SERVO_LOAD_DOWN,
    SERVO_LOAD_UP,
    SERVO_LOAD_WAIT,
} ServoLoadState_e;

// * 发射舵机状态机
typedef enum {
    SERVO_LAUNCH_READY = 0x00,
    SERVO_LAUNCH_DOWN,
    SERVO_LAUNCH_UP,
} ServoLaunchState_e;

typedef enum {
    SERVO_BLOCK_READY = 0x00,
    SERVO_BLOCK_DOWN,
    SERVO_BLOCK_MID,
    SERVO_BLOCK_UP,
    SERVO_BLOCK_WAIT,
} ServoBlockState_e;

//* 装填电机状态机
typedef enum {
    MOTOR_LOAD_READY = 0x00,
    MOTOR_LOAD_BUSY,
    MOTOR_LOAD_RETURN,
    MOTOR_LOAD_TIMEOUT,
} MotorLoadState_e;

//* 调节电机状态机
typedef enum {
    MOTOR_ADJUST_READY = 0x00,
    MOTOR_ADJUST_BUSY,
    MOTOR_ADJUST_TOWER,
    MOTOR_ADJUST_BASE,
    MOTOR_ADJUST_BLOCKED,
    MOTOR_ADJUST_TIMEOUT,
} MotorAdjustState_e;

//* 拖拽电机状态机
typedef enum {
    MOTOR_DRAG_READY = 0x00,
    MOTOR_DRAG_BUSY,
    MOTOR_DRAG_SAFE,
    MOTOR_DRAG_WAIT,
    MOTOR_DRAG_RETURN,
    MOTOR_DRAG_BLOCKED,
    MOTOR_DRAG_TIMEOUT,
} MotorDragState_e;

extern void launch_thread(void const *argument);
extern void servo_init(void);
extern void gimbal_command_update(void);
extern void shoot_command_update(void);
extern void get_encoder_angle(DartRackEncoder_t *encoder);
extern void get_target(DartRackTarget_t *target);
extern void motor_reset_target(void);
extern void get_state(DartRackStateMachine_e *state);

#endif
