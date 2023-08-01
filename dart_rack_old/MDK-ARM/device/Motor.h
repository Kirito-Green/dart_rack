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

#ifndef __MOTOR_H
#define __MOTOR_H

#include "struct_typedef.h"
#include "main.h"


extern fp32 CalAngle(int16_t ecd);
extern void ShootMotorControl(int16_t ShootMotor1, int16_t ShootMotor2, int16_t ShootMotor3, int16_t ShootMotor4);
extern void GimbalMotorControl(int16_t YawMotor, int16_t PitchMotor, int16_t ChainMotor);
extern void MotorProcess(uint32_t MotorID, CAN_HandleTypeDef *hcan, uint8_t* message);
extern void GimbalMotorMeasureUpdate(GimbalMotorMeasure_t* Gimbal);
extern void ShootMotorMeasureUpdate(ShootMotorMeasure_t* Shoot);
extern void AngleEncoderMeasureUpdate(DartRackEncoder_t* Encoder);


#endif
