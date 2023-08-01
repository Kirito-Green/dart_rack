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

#include "motor.h"
#include "bsp_can.h"
#include "setting.h"
#include "launch_thread.h"
#include "struct_typedef.h"
#include "can_packet.h"
#include "user_lib.h"

extern DartRack_t dart_rack;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
static uint32_t send_mail_box;
static CAN_TxHeaderTypeDef can_tx_message;
static uint8_t MotorSendBuffer[16];

/**
 * @brief ������cal_angle����������ֵ����Ƕȣ��Զ�Ϊ��λ����
 *
 * @param ecd ������ecd��������Ϊint16_t������һ��16λ�з�������������ʾ������������ֵ��
 *
 * @return ��ʾ�Ƕȣ��Զ�Ϊ��λ���ĸ���ֵ (fp32)��
 */
fp32 cal_angle(int16_t ecd)
{
    return (fp32)(ecd / 8192.0f * 360.0f - 180.0f);
}

/**
 * @brief ������shoot_motor_control��ͨ�� CAN ���߷��͵���������ݡ�
 *
 * @param motor1 ������motor1����һ����int16_t����������ʾ���һ��ֵ��
 * @param motor2 ������motor2����һ�� int16_t ��������ʾ�ڶ��������ֵ��
 * @param motor3 ������motor3����һ��int16_t���ͱ�����������Ƶ����������ֵ��
 * @param motor4 ������motor4����һ��int16_t��������ʾ���ڿ��Ƶ��ĸ������ֵ��
 */
void shoot_motor_control(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;

    MotorSendBuffer[0] = motor1 >> 8;
    MotorSendBuffer[1] = motor1;
    MotorSendBuffer[2] = motor2 >> 8;
    MotorSendBuffer[3] = motor2;
    MotorSendBuffer[4] = motor3 >> 8;
    MotorSendBuffer[5] = motor3;
    MotorSendBuffer[6] = motor4 >> 8;
    MotorSendBuffer[7] = motor4;

    can_tx_message.StdId = 0x200;
    HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, MotorSendBuffer, &send_mail_box);
}

/**
 * @brief ������gimbal_motor_control��ʹ�� CAN Э����ƫ���͸���������Ϳ����źš�
 *
 * @param yaw_motor yaw_motor ������һ�� int16_t ��������ʾ�����ϵͳ��ƫ�����������λ�û��ٶȡ�
 * @param pitch_motor ������pitch_motor����һ�� int16_t ��������ʾ�����ϵͳ�ĸ����������Ŀ���ֵ��
 */
void gimbal_motor_control(int16_t yaw_motor, int16_t pitch_motor)
{
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;

    MotorSendBuffer[0] = yaw_motor >> 8;
    MotorSendBuffer[1] = yaw_motor;
    MotorSendBuffer[2] = pitch_motor >> 8;
    MotorSendBuffer[3] = pitch_motor;

    can_tx_message.StdId = 0x1FF;
    HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, MotorSendBuffer, &send_mail_box);
}

static motor_measure_t drag_left_motor_measure;
static motor_measure_t drag_right_motor_measure;
static motor_measure_t load_motor_measure;
static motor_measure_t adjust_motor_measure;
static motor_measure_t yaw_motor_measure;
static motor_measure_t pitch_motor_measure;
static angle_encoder_t yaw_angle_encoder;
// static angle_encoder_t pitch_angle_encoder;

/* `get_motor_measure` �����ڸ��ݽ��յ��� CAN ���ݸ��µ������ֵ������������������ptr����ָ���������ṹ��ָ�룩�͡�data�������յ��� CAN ���ݣ��� */
#define get_motor_measure(ptr, data)                                                             \
    {                                                                                            \
        (ptr)->last_ecd = (ptr)->ecd;                                                            \
        (ptr)->ecd      = (uint16_t)((data)[0] << 8 | (data)[1]);                                \
        if ((ptr)->flag) {                                                                       \
            (ptr)->dist_ecd += choose_shortest_path((ptr)->ecd, (ptr)->last_ecd, MOTOR_MAX_ECD); \
        }                                                                                        \
        (ptr)->speed_rpm   = (uint16_t)((data)[2] << 8 | (data)[3]);                             \
        (ptr)->torque      = (uint16_t)((data)[4] << 8 | (data)[5]);                             \
        (ptr)->temperature = (data)[6];                                                          \
        (ptr)->flag        = 1;                                                                  \
    }

/* `get_angle_measure` �����ڸ��ݽ��յ��� CAN ���ݸ��½ǶȲ���ֵ������������������ptr����ָ��ǶȲ����ṹ��ָ�룩�͡�data�������յ��� CAN ���ݣ��� */
#define get_angle_measure(ptr, data)                                     \
    {                                                                    \
        (ptr)->angle            = (int16_t)((data)[3] << 8 | (data)[2]); \
        (ptr)->loops            = (int16_t)((data)[5] << 8 | (data)[4]); \
        (ptr)->angular_velocity = (int16_t)((data)[7] << 8 | (data)[6]); \
    }

/**
 * @brief ������motor_process�����ݸ����� MotorID ���������������
 *
 * @param MotorID ����ı�ʶ�������������� uint32_t��
 * @param hcan ������hcan����ָ��CAN_HandleTypeDef���ṹ��ָ�롣�ýṹ���� CAN ���߽ӿڵ����ú�״̬��Ϣ����������CAN����ͨ�Ų�����/����CAN��Ϣ��
 * @param message
 * ��message��������һ��ָ��uint8_t�����������ָ�롣�����ڸ��ݡ�MotorID����ֵ����Ϣ���ݴ��ݸ���get_motor_measure����get_angle_measure��������
 */
void motor_process(uint32_t MotorID, CAN_HandleTypeDef *hcan, uint8_t *message)
{
    switch (MotorID) {
        case DRAG_LEFT_MOTOR_ID:
            get_motor_measure(&drag_left_motor_measure, message);
            break;
        case DRAG_RIGHT_MOTOR_ID:
            get_motor_measure(&drag_right_motor_measure, message);
            break;
        case LOAD_MOTOR_ID:
            get_motor_measure(&load_motor_measure, message);
            break;
        case ADJUST_MOTOR_ID:
            get_motor_measure(&adjust_motor_measure, message);
            break;
        case YAW_MOTOR_ID:
            get_motor_measure(&yaw_motor_measure, message);
            break;
        case PITCH_MOTOR_ID:
            get_motor_measure(&pitch_motor_measure, message);
            break;
        case YAW_ANGLE_ENCODER_ID:
            get_angle_measure(&yaw_angle_encoder, message);
            break;
        case PITCH_ANGLE_ENCODER_ID:
            // get_angle_measure(&pitch_angle_encoder, message);
            get_motor_measure(&yaw_motor_measure, message);
            break;
        default:
            break;
    }
}

static uint32_t time_count = 0;
/**
 * @brief ������angle_encoder_measure_update�����½Ǳ������ĽǶȺͽ��ٶȡ�
 *
 * @param encoder
 * ������encoder����ָ��DartRackEncoder_t�����ͽṹ��ָ�롣�ýṹ�����й�ƫ���Ǻ͸����Ǳ���������Ϣ��������ǰ�Ƕȡ�ѭ�����ͽ��ٶȡ�
 * ������angle_encoder_measure_update�����½ǶȺͽǶ�
 */
void angle_encoder_measure_update(DartRackEncoder_t *encoder)
{
    time_count++;

    encoder->yaw_angle_encoder.angle = yaw_angle_encoder.angle * CIRCLE_ANGLE / ANGLE_ENCODER_MEASURE_MAX;
    encoder->yaw_angle_encoder.loops = yaw_angle_encoder.loops;
    if (time_count >= 10) { // 0.1s��һ�ν��ٶ�
        encoder->yaw_angle_encoder.angular_velocity = (fp32)((encoder->yaw_angle_encoder.angle - encoder->yaw_angle_encoder.last_angle) /
                                                             ANGLE_ENCODER_SAMPLING_TIME);
        if (is_zero(fabsf(encoder->yaw_angle_encoder.angular_velocity) - ANGLE_ENCODER_ZERO_VALOCITY)) { // ��ֹ������΢С�Ŷ�
            encoder->yaw_angle_encoder.angular_velocity = ZERO_POINT;
        }
        encoder->yaw_angle_encoder.last_angle = encoder->yaw_angle_encoder.angle;
    }
    //	encoder->yaw_angle_encoder.angular_velocity = yaw_angle_encoder.angular_velocity * CIRCLE_ANGLE
    //																		/ ANGLE_ENCODER_MEASURE_MAX / ANGLE_ENCODER_SAMPLING_TIME;

    /* ------------------------------- pitchδʹ�ñ����� ------------------------------ */
    // encoder->pitch_angle_encoder.angle = pitch_angle_encoder.angle * CIRCLE_ANGLE / ANGLE_ENCODER_MEASURE_MAX;
    // encoder->pitch_angle_encoder.loops = pitch_angle_encoder.loops;
    // if (time_count >= 10) {
    //     encoder->pitch_angle_encoder.angular_velocity = (fp32)((encoder->pitch_angle_encoder.angle - encoder->pitch_angle_encoder.last_angle) /
    //                                                            ANGLE_ENCODER_SAMPLING_TIME);
    //     if (is_zero(fabsf(encoder->pitch_angle_encoder.angular_velocity) - ANGLE_ENCODER_ZERO_VALOCITY)) { // ��ֹ������΢С�Ŷ�
    //         encoder->pitch_angle_encoder.angular_velocity = ZERO_POINT;
    //     }
    //     encoder->pitch_angle_encoder.last_angle = encoder->pitch_angle_encoder.angle;
    //     time_count                              = 0;
    // }
    //	encoder->pitch_angle_encoder.angular_velocity = pitch_angle_encoder.angular_velocity * CIRCLE_ANGLE
    //																		/ ANGLE_ENCODER_MEASURE_MAX / ANGLE_ENCODER_SAMPLING_TIME;
}

/**
 * @brief �ù��ܸ�����̨����Ĳ���ֵ������ƫ���ǡ�ƫ���ٶȡ������ٶȺ͸������롣
 *
 * @param gimbal ָ�� GimbalMotorMeasure_t ���ͽṹ��ָ�룬���а�����̨����Ĳ���ֵ��
 */
void gimbal_motor_measure_update(GimbalMotorMeasure_t *gimbal)
{
    gimbal->yaw_angle   = cal_angle(yaw_motor_measure.ecd);
    gimbal->yaw_speed   = yaw_motor_measure.speed_rpm;
    gimbal->pitch_speed = pitch_motor_measure.speed_rpm;
    gimbal->pitch_dist  = pitch_motor_measure.dist_ecd / MOTOR_MAX_ECD / M2006_SLOW_RATE * LEAD_SCREW_PITCH;
}

/**
 * @brief �ú���ͨ�����������ĵ�ǰ�ٶȺ;���ֵ����� ShootMotorMeasure_t �ṹ�е���Ӧ�ֶ������µ������ֵ��
 *
 * @param shoot ָ�� ShootMotorMeasure_t ���ͽṹ��ָ�룬���а����뷢������صĸ��ֲ���ֵ��
 */
void shoot_motor_measure_update(ShootMotorMeasure_t *shoot)
{
    shoot->drag_left_speed  = drag_left_motor_measure.speed_rpm;
    shoot->drag_right_speed = drag_right_motor_measure.speed_rpm;
    shoot->load_speed       = load_motor_measure.speed_rpm;
    shoot->adjust_speed     = adjust_motor_measure.speed_rpm;
    shoot->drag_left_dist   = drag_left_motor_measure.dist_ecd / MOTOR_MAX_ECD / M3508_SLOW_RATE * M3508_CIRCUMFERENCE;
    shoot->drag_right_dist  = drag_right_motor_measure.dist_ecd / MOTOR_MAX_ECD / M3508_SLOW_RATE * M3508_CIRCUMFERENCE;
    shoot->load_dist        = load_motor_measure.dist_ecd / MOTOR_MAX_ECD / M2006_SLOW_RATE * M2006_CIRCUMFERENCE;
    shoot->adjust_dist      = adjust_motor_measure.dist_ecd / MOTOR_MAX_ECD / M2006_SLOW_RATE * LEAD_SCREW_PITCH;
}

/**
 * @brief motor_update_dist_ecd �������¶������ľ���ֵ��
 *
 * @param dist_ecd dist_ecd ��ָ�� int32_t ֵ�����ָ�롣
 */
void motor_update_dist_ecd(int32_t *dist_ecd)
{
    pitch_motor_measure.dist_ecd      = *dist_ecd;
    drag_left_motor_measure.dist_ecd  = *(dist_ecd + 1);
    drag_right_motor_measure.dist_ecd = *(dist_ecd + 2);
    load_motor_measure.dist_ecd       = *(dist_ecd + 3);
    adjust_motor_measure.dist_ecd     = *(dist_ecd + 4);
}

/**
 * @brief ������get_motor_dist_ecd�����������ͬ����ļ������벢����洢�������С�
 *
 * @param dist_ecd dist_ecd ��ָ�� int32_t ֵ�����ָ�롣
 */
void get_motor_dist_ecd(int32_t *dist_ecd)
{
    *dist_ecd       = pitch_motor_measure.dist_ecd;
    *(dist_ecd + 1) = drag_left_motor_measure.dist_ecd;
    *(dist_ecd + 2) = drag_right_motor_measure.dist_ecd;
    *(dist_ecd + 3) = load_motor_measure.dist_ecd;
    *(dist_ecd + 4) = adjust_motor_measure.dist_ecd;
}

/**
 * @brief motor_reset_dist_ecd ��������������ͬ����ľ���ֵ��
 */
void motor_reset_dist_ecd(void)
{
    // pitch_motor_measure.dist_ecd      = 0;
    drag_left_motor_measure.dist_ecd  = 0;
    drag_right_motor_measure.dist_ecd = 0;
    load_motor_measure.dist_ecd       = 0;
}
