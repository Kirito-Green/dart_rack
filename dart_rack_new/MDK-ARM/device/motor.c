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
 * @brief 函数“cal_angle”根据输入值计算角度(以度为单位)。
 *
 * @param ecd 参数“ecd”的类型为int16_t，它是一个16位有符号整数。它表示编码器计数的值。
 *
 * @return 表示角度(以度为单位)的浮点值 (fp32)。
 */
fp32 cal_angle(int16_t ecd)
{
    return (fp32)(ecd / 8192.0f * 360.0f - 180.0f);
}

/**
 * @brief 函数“shoot_motor_control”通过 CAN 总线发送电机控制数据。
 *
 * @param motor1 参数“motor1”是一个“int16_t”变量，表示电机一的值。
 * @param motor2 参数“motor2”是一个 int16_t 变量，表示第二个电机的值。
 * @param motor3 参数“motor3”是一个int16_t类型变量，代表控制第三个电机的值。
 * @param motor4 参数“motor4”是一个int16_t变量，表示用于控制第四个电机的值。
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
 * @brief 函数“gimbal_motor_control”使用 CAN 协议向偏航和俯仰电机发送控制信号。
 *
 * @param yaw_motor yaw_motor 参数是一个 int16_t 变量，表示万向节系统中偏航电机的所需位置或速度。
 * @param pitch_motor 参数“pitch_motor”是一个 int16_t 变量，表示万向节系统的俯仰电机所需的控制值。
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

/* `get_motor_measure` 宏用于根据接收到的 CAN 数据更新电机测量值。它有两个参数：“ptr”(指向电机测量结构的指针)和“data”(接收到的 CAN 数据)。 */
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

/* `get_angle_measure` 宏用于根据接收到的 CAN 数据更新角度测量值。它有两个参数：“ptr”(指向角度测量结构的指针)和“data”(接收到的 CAN 数据)。 */
#define get_angle_measure(ptr, data)                                     \
    {                                                                    \
        (ptr)->angle            = (int16_t)((data)[3] << 8 | (data)[2]); \
        (ptr)->loops            = (int16_t)((data)[5] << 8 | (data)[4]); \
        (ptr)->angular_velocity = (int16_t)((data)[7] << 8 | (data)[6]); \
    }

/**
 * @brief 函数“motor_process”根据给定的 MotorID 处理电机测量结果。
 *
 * @param MotorID 电机的标识符。它的类型是 uint32_t。
 * @param hcan 参数“hcan”是指向“CAN_HandleTypeDef”结构的指针。该结构包含 CAN 总线接口的配置和状态信息。它用于与CAN总线通信并发送/接收CAN消息。
 * @param message
 * “message”参数是一个指向“uint8_t”类型数组的指针。它用于根据“MotorID”的值将消息数据传递给“get_motor_measure”或“get_angle_measure”函数。
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
            break;
        default:
            break;
    }
}

static uint32_t time_count = 0;
/**
 * @brief 函数“angle_encoder_measure_update”更新角编码器的角度和角速度。
 *
 * @param encoder
 * 参数“encoder”是指向“DartRackEncoder_t”类型结构的指针。该结构包含有关偏航角和俯仰角编码器的信息，包括当前角度、循环数和角速度。
 * 函数“angle_encoder_measure_update”更新角度和角度
 */
void angle_encoder_measure_update(DartRackEncoder_t *encoder)
{
    time_count++;

    encoder->yaw_angle_encoder.angle = yaw_angle_encoder.angle * CIRCLE_ANGLE / ANGLE_ENCODER_MEASURE_MAX;
    encoder->yaw_angle_encoder.loops = yaw_angle_encoder.loops;
    if (time_count >= 10) { // 0.1s测一次角速度
        encoder->yaw_angle_encoder.angular_velocity = (fp32)((encoder->yaw_angle_encoder.angle - encoder->yaw_angle_encoder.last_angle) /
                                                             ANGLE_ENCODER_SAMPLING_TIME);
        if (is_zero(fabsf(encoder->yaw_angle_encoder.angular_velocity) - ANGLE_ENCODER_ZERO_VALOCITY)) { // 防止编码器微小扰动
            encoder->yaw_angle_encoder.angular_velocity = ZERO_POINT;
        }
        encoder->yaw_angle_encoder.last_angle = encoder->yaw_angle_encoder.angle;
    }
    //	encoder->yaw_angle_encoder.angular_velocity = yaw_angle_encoder.angular_velocity * CIRCLE_ANGLE
    //																		/ ANGLE_ENCODER_MEASURE_MAX / ANGLE_ENCODER_SAMPLING_TIME;

    /* ------------------------------- pitch未使用编码器 ------------------------------ */
    // encoder->pitch_angle_encoder.angle = pitch_angle_encoder.angle * CIRCLE_ANGLE / ANGLE_ENCODER_MEASURE_MAX;
    // encoder->pitch_angle_encoder.loops = pitch_angle_encoder.loops;
    // if (time_count >= 10) {
    //     encoder->pitch_angle_encoder.angular_velocity = (fp32)((encoder->pitch_angle_encoder.angle - encoder->pitch_angle_encoder.last_angle) /
    //                                                            ANGLE_ENCODER_SAMPLING_TIME);
    //     if (is_zero(fabsf(encoder->pitch_angle_encoder.angular_velocity) - ANGLE_ENCODER_ZERO_VALOCITY)) { // 防止编码器微小扰动
    //         encoder->pitch_angle_encoder.angular_velocity = ZERO_POINT;
    //     }
    //     encoder->pitch_angle_encoder.last_angle = encoder->pitch_angle_encoder.angle;
    //     time_count                              = 0;
    // }
    //	encoder->pitch_angle_encoder.angular_velocity = pitch_angle_encoder.angular_velocity * CIRCLE_ANGLE
    //																		/ ANGLE_ENCODER_MEASURE_MAX / ANGLE_ENCODER_SAMPLING_TIME;
}

/**
 * @brief 该功能更新云台电机的测量值，包括偏航角、偏航速度、俯仰速度和俯仰距离。
 *
 * @param gimbal 指向 GimbalMotorMeasure_t 类型结构的指针，其中包含云台电机的测量值。
 */
void gimbal_motor_measure_update(GimbalMotorMeasure_t *gimbal)
{
    gimbal->yaw_angle   = cal_angle(yaw_motor_measure.ecd);
    gimbal->yaw_speed   = yaw_motor_measure.speed_rpm;
    gimbal->pitch_speed = pitch_motor_measure.speed_rpm;
    gimbal->pitch_dist  = pitch_motor_measure.dist_ecd / MOTOR_MAX_ECD / M2006_SLOW_RATE * LEAD_SCREW_PITCH;
}

/**
 * @brief 该函数通过将发射电机的当前速度和距离值分配给 ShootMotorMeasure_t 结构中的相应字段来更新电机测量值。
 *
 * @param shoot 指向 ShootMotorMeasure_t 类型结构的指针，其中包含与发射电机相关的各种测量值。
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
 * @brief motor_update_dist_ecd 函数更新多个电机的距离值。
 *
 * @param dist_ecd dist_ecd 是指向 int32_t 值数组的指针。
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
 * @brief 函数“get_motor_dist_ecd”检索五个不同电机的计数距离并将其存储在数组中。
 *
 * @param dist_ecd dist_ecd 是指向 int32_t 值数组的指针。
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
 * @brief motor_reset_dist_ecd 函数重置三个不同电机的距离值。
 */
void motor_reset_dist_ecd(void)
{
    // pitch_motor_measure.dist_ecd      = 0;
    drag_left_motor_measure.dist_ecd  = 0;
    drag_right_motor_measure.dist_ecd = 0;
    load_motor_measure.dist_ecd       = 0;
}
