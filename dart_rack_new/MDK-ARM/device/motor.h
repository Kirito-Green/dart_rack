/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
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

extern fp32 cal_angle(int16_t ecd);
extern void shoot_motor_control(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void gimbal_motor_control(int16_t yaw_motor, int16_t pitch_motor);
extern void motor_process(uint32_t MotorID, CAN_HandleTypeDef *hcan, uint8_t *message);
extern void gimbal_motor_measure_update(GimbalMotorMeasure_t *Gimbal);
extern void shoot_motor_measure_update(ShootMotorMeasure_t *Shoot);
extern void angle_encoder_measure_update(DartRackEncoder_t *Encoder);
extern void motor_update_dist_ecd(int32_t *dist_ecd);
extern void get_motor_dist_ecd(int32_t *dist_ecd);
extern void motor_reset_dist_ecd(void);

#endif
