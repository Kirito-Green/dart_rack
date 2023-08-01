#ifndef LAUCH_THREAD_H
#define LAUCH_THREAD_H

#include "struct_typedef.h"

typedef struct
{
    pid_type_def Chain;
    pid_type_def Shoot[4];
    cascade_pid_t Yaw;
    cascade_pid_t Pitch;
} DartRackPid_t;

typedef struct
{
    ladrc_type_def Chain;
    // ladrc_type_def Shoot[4];
    adrc_type_def Shoot[4];
} DartRackAdrc_t;

typedef struct
{
    efc_type_def Chain;
    efc_type_def Shoot[4];
    efc_type_def Yaw;
    efc_type_def Pitch;
} DartRackEfc_t;

typedef struct
{
    cascade_pid_ladrc_t Pitch;
    cascade_pid_ladrc_t Yaw;
} DartRackPidAdrc_t;

typedef struct {
    fp32 Yaw;
    fp32 Pitch;
    fp32 Chain;
    fp32 Shoot;
} DartRackCommand_t;

typedef struct {
    int16_t Yaw;
    int16_t Pitch;
    int16_t Chain;
    int16_t Shoot[FRICTION_WHEEL_NUM];
    fp32 LauchSpeed;
} DartRackOutput_t;

typedef struct {
    fp32 FirstSpeed[4];
    fp32 SecondSpeed[4];
} MotorSpeed_t;

typedef struct {
    MotorSpeed_t Straight;
    MotorSpeed_t Tower;
    MotorSpeed_t Base;
} DartRackTarget_t;

typedef enum {
    DART_RACK_NO_FORCE = 0x00,
    DART_RACK_INIT,
    DART_RACK_TEST,
    DART_RACK_MATCH,
} DartRackStateMachine_e;

typedef struct
{
    DartRackPid_t Pid;
    DartRackAdrc_t Adrc;
    // DartRackEfc_t               Efc;
    DartRackPidAdrc_t PidAdrc;
    DartRackMotorMeasure_t MotorMeasure;
    DartRackEncoder_t Encoder;
    DartRackTarget_t Target;
    DartRackCommand_t Command;
    DartRackOutput_t Output;
    DartRackStateMachine_e StateMachine;
} DartRack_t;

extern DartRack_t DartRack;
extern void LauchThread(void const *argument);
extern void DartRackInit(void);
extern void ShootDiffCompensate(void);
extern void DartRackShoot(void);
extern void DartRackStateMachineUpdate(void);
extern void MotorMeasureUpdate(void);
extern void EncoderMeasureUpdate(void);
extern void MotorAlgorithmUpdate(void);
extern void GimbalCommandUpdate(void);
extern void ShootCommandUpdate(void);
extern void DartLauchSpeedUpdate(void);
extern void get_encoder_angle(DartRackEncoder_t *encoder);
extern void get_shoot_speed(fp32 *first_speed, fp32 *second_speed, ShootMotorMeasure_t *measure);

#endif
