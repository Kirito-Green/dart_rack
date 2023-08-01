/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "Motor.h"
#include "bsp_can.h"
#include "Setting.h"
#include "LauchThread.h"
#include "struct_typedef.h"
#include "CanPacket.h"
#include "User_lib.h"

extern DartRack_t DartRack;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
static uint32_t send_mail_box;
static CAN_TxHeaderTypeDef can_tx_message;
static uint8_t MotorSendBuffer[16];

fp32 CalAngle(int16_t ecd)
{
    return (fp32)(ecd / 8192.0f * 360.0f - 180.0f);
}

void ShootMotorControl(int16_t ShootMotor1, int16_t ShootMotor2, int16_t ShootMotor3, int16_t ShootMotor4)
{
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;

    MotorSendBuffer[0] = ShootMotor1 >> 8;
    MotorSendBuffer[1] = ShootMotor1;
    MotorSendBuffer[2] = ShootMotor2 >> 8;
    MotorSendBuffer[3] = ShootMotor2;
    MotorSendBuffer[4] = ShootMotor3 >> 8;
    MotorSendBuffer[5] = ShootMotor3;
    MotorSendBuffer[6] = ShootMotor4 >> 8;
    MotorSendBuffer[7] = ShootMotor4;

    can_tx_message.StdId = 0x200;
    HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, MotorSendBuffer, &send_mail_box); // 摩擦轮
}

void GimbalMotorControl(int16_t YawMotor, int16_t PitchMotor, int16_t ChainMotor)
{
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;

    MotorSendBuffer[0] = YawMotor >> 8;
    MotorSendBuffer[1] = YawMotor;
    MotorSendBuffer[2] = PitchMotor >> 8;
    MotorSendBuffer[3] = PitchMotor;
    MotorSendBuffer[4] = ChainMotor >> 8;
    MotorSendBuffer[5] = ChainMotor;

    can_tx_message.StdId = 0x1FF;
    HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, MotorSendBuffer, &send_mail_box); // Pitch轴 链条电机
}

static motor_measure_t ChainMotorMeasure;
static motor_measure_t ShootMotorMeasure[4];
static motor_measure_t YawMotorMeasure;
static motor_measure_t PitchMotorMeasure;
static angle_encoder_t YawAngleEncoder;
static angle_encoder_t PitchAngleEncoder;

// motor data read
#define get_motor_measure(ptr, data)                                 \
    {                                                                \
        (ptr)->last_ecd    = (ptr)->ecd;                             \
        (ptr)->ecd         = (uint16_t)((data)[0] << 8 | (data)[1]); \
        (ptr)->speed_rpm   = (uint16_t)((data)[2] << 8 | (data)[3]); \
        (ptr)->torque      = (uint16_t)((data)[4] << 8 | (data)[5]); \
        (ptr)->temperature = (data)[6];                              \
    }

#define get_angle_measure(ptr, data)                                     \
    {                                                                    \
        (ptr)->angle            = (int16_t)((data)[3] << 8 | (data)[2]); \
        (ptr)->loops            = (int16_t)((data)[5] << 8 | (data)[4]); \
        (ptr)->angular_velocity = (int16_t)((data)[7] << 8 | (data)[6]); \
    }

void MotorProcess(uint32_t MotorID, CAN_HandleTypeDef *hcan, uint8_t *message)
{
    switch (MotorID) {
        case SHOOT_MOTOR1_ID:
            get_motor_measure(&ShootMotorMeasure[0], message);
            break;
        case SHOOT_MOTOR2_ID:
            get_motor_measure(&ShootMotorMeasure[1], message);
            break;
        case SHOOT_MOTOR3_ID:
            get_motor_measure(&ShootMotorMeasure[2], message);
            break;
        case SHOOT_MOTOR4_ID:
            get_motor_measure(&ShootMotorMeasure[3], message);
            break;
        case YAW_MOTOR_ID:
            get_motor_measure(&YawMotorMeasure, message);
            break;
        case PITCH_MOTOR_ID:
            get_motor_measure(&PitchMotorMeasure, message);
            break;
        case CHAIN_MOTOR_ID:
            get_motor_measure(&ChainMotorMeasure, message);
            break;
        case YAW_ANGLE_ENCODER_ID:
            get_angle_measure(&YawAngleEncoder, message);
            break;
        case PITCH_ANGLE_ENCODER_ID:
            get_angle_measure(&PitchAngleEncoder, message);
            break;
        default:
            break;
    }
}

static uint32_t time_count = 0;
void AngleEncoderMeasureUpdate(DartRackEncoder_t *Encoder)
{
    time_count++;

    Encoder->YawAngleEncoder.Angle = YawAngleEncoder.angle * CIRCLE_ANGLE / ANGLE_ENCODER_MEASURE_MAX;
    Encoder->YawAngleEncoder.loops = YawAngleEncoder.loops;
    if (time_count >= 10) { // 0.1s测一次角速度
        Encoder->YawAngleEncoder.AnguarVelocity = (fp32)((Encoder->YawAngleEncoder.Angle - Encoder->YawAngleEncoder.LastAngle) /
                                                         ANGLE_ENCODER_SAMPLING_TIME);
        if (is_zero(fabsf(Encoder->YawAngleEncoder.AnguarVelocity) - ANGLE_ENCODER_ZERO_VALOCITY)) { // 防止编码器微小扰动
            Encoder->YawAngleEncoder.AnguarVelocity = ZERO_POINT;
        }
        Encoder->YawAngleEncoder.LastAngle = Encoder->YawAngleEncoder.Angle;
    }
    //	Encoder->YawAngleEncoder.AnguarVelocity = YawAngleEncoder.angular_velocity * CIRCLE_ANGLE
    //																		/ ANGLE_ENCODER_MEASURE_MAX / ANGLE_ENCODER_SAMPLING_TIME;

    Encoder->PitchAngleEncoder.Angle = PitchAngleEncoder.angle * CIRCLE_ANGLE / ANGLE_ENCODER_MEASURE_MAX;
    Encoder->PitchAngleEncoder.loops = PitchAngleEncoder.loops;
    if (time_count >= 10) {
        Encoder->PitchAngleEncoder.AnguarVelocity = (fp32)((Encoder->PitchAngleEncoder.Angle - Encoder->PitchAngleEncoder.LastAngle) /
                                                           ANGLE_ENCODER_SAMPLING_TIME);
        if (is_zero(fabsf(Encoder->PitchAngleEncoder.AnguarVelocity) - ANGLE_ENCODER_ZERO_VALOCITY)) { // 防止编码器微小扰动
            Encoder->PitchAngleEncoder.AnguarVelocity = ZERO_POINT;
        }
        Encoder->PitchAngleEncoder.LastAngle = Encoder->PitchAngleEncoder.Angle;
        time_count                           = 0;
    }
    //	Encoder->PitchAngleEncoder.AnguarVelocity = PitchAngleEncoder.angular_velocity * CIRCLE_ANGLE
    //																		/ ANGLE_ENCODER_MEASURE_MAX / ANGLE_ENCODER_SAMPLING_TIME;
}

void GimbalMotorMeasureUpdate(GimbalMotorMeasure_t *Gimbal)
{
    Gimbal->YawMotorAngle   = CalAngle(YawMotorMeasure.ecd);
    Gimbal->YawMotorSpeed   = YawMotorMeasure.speed_rpm;
    Gimbal->PitchMotorAngle = CalAngle(PitchMotorMeasure.ecd);
    Gimbal->PitchMotorSpeed = PitchMotorMeasure.speed_rpm;
}

void ShootMotorMeasureUpdate(ShootMotorMeasure_t *Shoot)
{
    Shoot->ChainMotorSpeed = ChainMotorMeasure.speed_rpm;
    for (int i = 0; i < FRICTION_WHEEL_NUM; i++) {
        Shoot->ShootMotorSpeed[i] = ShootMotorMeasure[i].speed_rpm;
    }
}
