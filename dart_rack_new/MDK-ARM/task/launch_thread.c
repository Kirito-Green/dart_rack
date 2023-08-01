#include "launch_thread.h"
#include "pid.h"
#include "adrc.h"
#include "efc.h"
#include "combination.h"
#include "motor.h"
#include "cmsis_os.h"
#include "dart_Rack_parameter.h"
#include "setting.h"
#include "remote.h"
#include "referee.h"
#include "interrupt_service.h"
#include "string.h"
#include "user_lib.h"
#include "tim.h"
#include "stdio.h"
#include "tfcard.h"
#include "ff.h"
#include "beep.h"
#include "tfcard_transmit_thread.h"

static DartRack_t dart_rack;         // 飞镖架状态
static RC_ctrl_t remote;             // 遥控器状态
static OfflineMonitor_t offline;     // 模块离线状态
static RefereeInformation_t referee; // 裁判系统数据

/* 飞镖状态 */
static bool_t dart_rack_cmd_flag;    // 客户端命令标志
static bool_t dart_rack_reset_flag;  // 飞镖架复位标志
static int8_t dart_rack_shoot_state; // 飞镖发射状态
static int8_t motor_load_state;      // 装填电机状态机
static int8_t motor_drag_state;      // 拖拽电机状态机
static int8_t motor_adjust_state;    // 调节电机状态机
static int8_t servo_load_state;      // 装填舵机状态机
static int8_t servo_launch_state;    // 发射舵机状态机
static int8_t dart_rack_client_cmd;  // 客户端命令
static int8_t dart_rack_aim_shoot;   // 飞镖架瞄准点
static int8_t dart_rack_num_count;   // 飞镖架发射次数

/* 事件记录 */
static uint32_t dart_rack_shoot_count; // 飞镖架发射计时
static uint32_t motor_load_count;      // 装填电机计时
static uint32_t motor_drag_count;      // 拖拽电机计时
static uint32_t motor_adjust_count;    // 调节电机计时
static uint32_t servo_load_count;      // 装填舵机计时
static uint32_t servo_launch_count;    // 发射舵机计时

/* 云台位置 */
static fp32 yaw_cmd_temp;
static fp32 pitch_cmd_temp;

/* 舵机角度 */
static int16_t tim_right_first_compare  = TIM_RIGHT_FIRST_COMPARE_UP;
static int16_t tim_right_second_compare = TIM_RIGHT_SECOND_COMPARE_UP;
static int16_t tim_left_first_compare   = TIM_LEFT_FIRST_COMPARE_UP;
static int16_t tim_left_second_compare  = TIM_LEFT_SECOND_COMPARE_UP;
static int16_t tim_bottom_compare       = TIM_BOTTOM_COMPARE_UP;

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

void servo_init(void);
void servo_start(void);
void servo_no_force(void);
void servo_update(void);
void servo_load_up(void);
void servo_load_down(void);
void servo_launch_up(void);
void servo_launch_down(void);
void gimbal_no_force(void);
void shoot_no_force(void);
void tfcard_read(void);
void dart_rack_shoot(void);
void dart_rack_state_update(void);
void motor_measure_update(void);
void motor_target_update(void);
void encoder_measure_update(void);
void motor_algorithm_update(void);

/**
 * @brief 函数“launch_thread”不断更新各种数据并执行与飞镖射击系统相关的动作。
 *
 * @param argument “argument”参数是指向要传递给线程函数的任何附加数据的指针。它的类型为“void const
 * *”，这意味着它可以是指向任何类型数据的指针。在这种情况下，它没有在函数中使用，所以你可以忽略
 */
void launch_thread(void const *argument)
{
    tfcard_read();
    for (;;) {
        remote = *get_remote_control_point();    // 获取遥控器数据
        device_offline_monitor_update(&offline); // 获取模块离线数据
        get_referee_information(&referee);       // 获取裁判系统数据

        dart_rack_state_update(); // 飞镖架状态转化
        motor_measure_update();   // 电机数据更新
        encoder_measure_update(); // 编码器更新
        motor_algorithm_update(); // 电机PID更新
        gimbal_command_update();  // 云台指令更新
        shoot_command_update();   // 发射指令更新
        dart_rack_shoot();        // 飞镖发射

        osDelay(1);
    }
}

/**
 * @brief 函数“servo_init”在中断模式下启动 TIM4 和 TIM5 的基本定时器。
 *  TIM4 和 TIM5用于控制舵机的装填和发射
 */
void servo_init(void)
{
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
}

/**
 * @brief 函数“servo_start”启动两个不同定时器的多个通道的 PWM 输出。
 *  舵机启动pwm输出
 */
void servo_start(void)
{
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

/**
 * @brief 函数“servo_no_force”停止两个不同定时器的多个通道的 PWM 信号。
 * 舵机关闭pwm输出
 */
void servo_no_force(void)
{
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_4);
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
}

/**
 * @brief 函数“servo_update”更新舵机的角度。
 */
void servo_update(void)
{
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, tim_right_first_compare);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, tim_right_second_compare);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, tim_left_first_compare);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, tim_left_second_compare);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, tim_bottom_compare);
}

/**
 * @brief 函数“servo_load_up”将舵机角度设置为待装填状态
 */
void servo_load_up(void)
{
    tim_right_first_compare  = TIM_RIGHT_FIRST_COMPARE_UP;
    tim_right_second_compare = TIM_RIGHT_SECOND_COMPARE_UP;
    tim_left_first_compare   = TIM_LEFT_FIRST_COMPARE_UP;
    tim_left_second_compare  = TIM_LEFT_SECOND_COMPARE_UP;
}

/**
 * 函数“servo_load_down”将舵机角度设置为装填状态
 */
void servo_load_down(void)
{
    tim_right_first_compare  = TIM_RIGHT_FIRST_COMPARE_DOWN;
    tim_right_second_compare = TIM_RIGHT_SECOND_COMPARE_DOWN;
    tim_left_first_compare   = TIM_LEFT_FIRST_COMPARE_DOWN;
    tim_left_second_compare  = TIM_LEFT_SECOND_COMPARE_DOWN;
}

/**
 * 函数“servo_launch_up”将发射舵机角度设置为待发射状态
 */
void servo_launch_up(void)
{
    tim_bottom_compare = TIM_BOTTOM_COMPARE_UP;
}

/**
 * 函数“servo_launch_down”将发射舵机角度设置为发射状态
 */
void servo_launch_down(void)
{
    tim_bottom_compare = TIM_BOTTOM_COMPARE_DOWN;
}

/**
 * 函数“gimbal_no_force”将云台电机无力输出。
 */
void gimbal_no_force(void)
{
    dart_rack.output.yaw   = ZERO_POINT;
    dart_rack.output.pitch = ZERO_POINT;
}

/**
 * 函数“shoot_no_force”将发射电机无力输出。
 */
void shoot_no_force(void)
{
    dart_rack.output.drag_left  = ZERO_POINT;
    dart_rack.output.drag_right = ZERO_POINT;
    dart_rack.output.load       = ZERO_POINT;
    dart_rack.output.adjust     = ZERO_POINT;
}

/**
 * @brief 函数“tfcard_read”尝试多次从 TF 卡读取数据，如果成功则更新电机测量值和目标。
 *
 * @return 什么都没有（无效）。
 */
void tfcard_read(void)
{
    uint8_t flag;
    for (uint8_t i = 0; i < TFCARD_READ_TIMES_MAX; i++) {
        flag = tfcard_read_motor(&dart_rack.motor_measure);
        if (flag == FR_OK) {
            motor_measure_update();
            motor_target_update();
            beep_ready();
            // printf("tfcard read success\r\n");
            return;
        }
    }
    // printf("tfcard read error: %d\r\n", flag);
}

/**
 * @brief 函数“dart_rack_shoot”控制从飞镖架发射飞镖的伺服和电机运动。
 */
void dart_rack_shoot(void)
{
    servo_update();
    shoot_motor_control(dart_rack.output.drag_left,
                        dart_rack.output.drag_right,
                        dart_rack.output.load,
                        dart_rack.output.adjust);
    gimbal_motor_control(ZERO_POINT,  //*dart_rack.output.yaw,
                         ZERO_POINT); // PITCH_MOTOR_DIRECTION * dart_rack.output.pitch);
}

/**
 * @brief 函数“dart_rack_state_update”根据各种条件和输入更新飞镖架的状态。
 *
 * @return 函数 `dart_rack_state_update` 没有显式返回值。
 */
DartRackStateMachine_e drslast;
DartRackStateMachine_e drsthis;
void dart_rack_state_update(void)
{
    drslast = dart_rack.state; // 记录之前状态

    /* 电机离线保护 */
    if ( // offline.yaw_motor == DEVICE_OFFLINE ||
         // offline.pitch_motor == DEVICE_OFFLINE ||
        offline.drag_left_motor == DEVICE_OFFLINE ||
        offline.drag_right_motor == DEVICE_OFFLINE ||
        offline.load_motor == DEVICE_OFFLINE ||
        offline.adjust_motor == DEVICE_OFFLINE
        // offline.yaw_angle_encoder == DEVICE_OFFLINE
        //  offline.pitch_angle_encoder == DEVICE_OFFLINE
    ) {
        dart_rack.state = DART_RACK_NO_FORCE;
        return;
    }

    /* 遥控器离线保护 */
    if (offline.remote == DEVICE_OFFLINE) {
        dart_rack.state = DART_RACK_NO_FORCE;
        return;
    }

    /* yaw PITCH轴超位保护 */
    // if (dart_rack.encoder.yaw_angle_encoder.angle < YAW_MIN_ANGLE ||
    // dart_rack.encoder.yaw_angle_encoder.angle > YAW_MAX_ANGLE
    //		 dart_rack.encoder.pitch_angle_encoder.angle < PITCH_MIN_ANGLE ||
    // dart_rack.encoder.pitch_angle_encoder.angle > PITCH_MAX_ANGLE
    // ) {
    //     dart_rack.state = DART_RACK_NO_FORCE;
    //     return;
    // }

    /* 飞镖架状态机更新 */
    switch (remote.rc.s[0]) {
            /* 右拨杆打到最上，飞镖架进入比赛模式,该模式下摩擦轮参数已调好 */
        case RC_SW_UP:                     // 电机状态更新
            if (dart_rack.state == DART_RACK_MATCH) { // 不要重复设参数
                return;
            }
            servo_start();
            dart_rack_client_cmd = 0;                   // 客户端默认前哨站
            yaw_cmd_temp         = 0;                   // YAW轴指令发送记录
            dart_rack.target.yaw = YAW_TOWER_ANGLE;     // 默认瞄准前哨站
            dart_rack_aim_shoot  = DART_RACK_AIM_TOWER; // 瞄准标志

            break;
            /* 右拨杆打到中间，飞镖架进入调试模式,该模式下摩擦轮速度可调节 */
        case RC_SW_MID:
            if (dart_rack.state == DART_RACK_TEST) { // 不要重复设参数
                return;
            }
            dart_rack_reset_flag = 0;
            servo_start();
            dart_rack.state = DART_RACK_TEST;
            break;
            /* 右拨杆打到最下，或遥控器数据出错，飞镖架进入无力模式 */
        case RC_SW_DOWN:
            dart_rack.state = DART_RACK_NO_FORCE;
            break;
        default:
            dart_rack.state = DART_RACK_NO_FORCE;
            break;
    }

    drsthis = dart_rack.state; // 记录之后状态
}

/**
 * @brief 函数“motor_measure_update”更新飞镖架中云台和射击电机的测量值。
 */
void motor_measure_update(void)
{
    gimbal_motor_measure_update(&dart_rack.motor_measure.gimbal_motor_measure);
    shoot_motor_measure_update(&dart_rack.motor_measure.shoot_motor_measure);
}

/**
 * @brief 函数“encoder_measure_update”更新角度编码器的测量值。
 */
void encoder_measure_update(void)
{
    angle_encoder_measure_update(&dart_rack.encoder);
}

/**
 * @brief motor_target_update 函数根据其他电机的测量值更新飞镖架系统中各个电机的目标位置。
 */
void motor_target_update(void)
{
    dart_rack.target.pitch      = dart_rack.motor_measure.gimbal_motor_measure.pitch_dist;
    dart_rack.target.drag_left  = dart_rack.motor_measure.shoot_motor_measure.drag_left_dist;
    dart_rack.target.drag_right = dart_rack.motor_measure.shoot_motor_measure.drag_right_dist;
    dart_rack.target.load       = dart_rack.motor_measure.shoot_motor_measure.load_dist;
    dart_rack.target.adjust     = dart_rack.motor_measure.shoot_motor_measure.adjust_dist;
}

/**
 * @brief 函数“motor_algorithm_update”根据系统的当前状态初始化并更新各种电机算法的 PID 控制器。
 *
 * @return 该函数不返回任何值。
 */
void motor_algorithm_update(void)
{
    /* 避免重复设置算法 */
    if (drsthis == drslast && remote.rc.s[1] == remote.rc.s_last[1]) {
        return;
    }

    if (dart_rack.state == DART_RACK_INIT) {
        cascade_PID_init(&dart_rack.pid.cascade_drag_left,
                         DRAG_LEFT_POSITION_OPERATE,
                         DRAG_LEFT_SPEED_OPERATE,
                         POSITION_INPUT_ALPHA,
                         POSITION_OUTPUT_ALPHA,
                         SPEED_INPUT_ALPHA,
                         SPEED_OUTPUT_ALPHA,
                         POSITION_DELTA_INCREASE,
                         POSITION_ALPHA_INCREASE,
                         SPEED_DELTA,
                         SPEED_ALPHA,
                         DRAG_LEFT_MAX_SPEED,
                         DRAG_LEFT_MAX_ISPEED,
                         DRAG_LEFT_MAX_OUTPUT,
                         DRAG_LEFT_MAX_IOUTPUT,
                         POSITION_DEAD_AREA,
                         SPEED_DEAD_AREA);
        cascade_PID_init(&dart_rack.pid.cascade_drag_right,
                         DRAG_RIGHT_POSITION_OPERATE,
                         DRAG_RIGHT_SPEED_OPERATE,
                         POSITION_INPUT_ALPHA,
                         POSITION_OUTPUT_ALPHA,
                         SPEED_INPUT_ALPHA,
                         SPEED_OUTPUT_ALPHA,
                         POSITION_DELTA_INCREASE,
                         POSITION_ALPHA_INCREASE,
                         SPEED_DELTA,
                         SPEED_ALPHA,
                         DRAG_RIGHT_MAX_SPEED,
                         DRAG_RIGHT_MAX_ISPEED,
                         DRAG_RIGHT_MAX_OUTPUT,
                         DRAG_RIGHT_MAX_IOUTPUT,
                         POSITION_DEAD_AREA,
                         SPEED_DEAD_AREA);
        cascade_PID_init(&dart_rack.pid.cascade_load,
                         LOAD_POSITION_OPERATE,
                         LOAD_SPEED_OPERATE,
                         POSITION_INPUT_ALPHA,
                         POSITION_OUTPUT_ALPHA,
                         SPEED_INPUT_ALPHA,
                         SPEED_OUTPUT_ALPHA,
                         POSITION_DELTA_DECREASE,
                         POSITION_ALPHA_DECREASE,
                         SPEED_DELTA,
                         SPEED_ALPHA,
                         LOAD_MAX_SPEED,
                         LOAD_MAX_ISPEED,
                         LOAD_MAX_OUTPUT,
                         LOAD_MAX_IOUTPUT,
                         POSITION_DEAD_AREA,
                         SPEED_DEAD_AREA);
        cascade_PID_init(&dart_rack.pid.cascade_adjust,
                         ADJUST_POSITION_OPERATE,
                         ADJUST_SPEED_OPERATE,
                         POSITION_INPUT_ALPHA,
                         POSITION_OUTPUT_ALPHA,
                         SPEED_INPUT_ALPHA,
                         SPEED_OUTPUT_ALPHA,
                         POSITION_DELTA_DECREASE,
                         POSITION_ALPHA_DECREASE,
                         SPEED_DELTA,
                         SPEED_ALPHA,
                         ADJUST_MAX_SPEED,
                         ADJUST_MAX_ISPEED,
                         ADJUST_MAX_OUTPUT,
                         ADJUST_MAX_IOUTPUT,
                         POSITION_DEAD_AREA,
                         SPEED_DEAD_AREA);
        cascade_PID_init(&dart_rack.pid.yaw,
                         YAW_ANGLE_OPERATE,
                         YAW_SPEED_OPERATE,
                         ANGLE_INPUT_ALPHA,
                         ANGLE_OUTPUT_ALPHA,
                         SPEED_INPUT_ALPHA,
                         SPEED_OUTPUT_ALPHA,
                         ANGLE_DELTA,
                         ANGLE_ALPHA,
                         SPEED_DELTA,
                         SPEED_ALPHA,
                         YAW_MAX_ANGULAR_VELOCITY,
                         YAW_MAX_ANGULAR_IVELOCITY,
                         YAW_MAX_SPEED_OUTPUT,
                         YAW_MAX_SPEED_IOUTPUT,
                         ANGLE_DEAD_AREA,
                         SPEED_DEAD_AREA);
        cascade_PID_init(&dart_rack.pid.pitch,
                         PITCH_ANGLE_OPERATE,
                         PITCH_SPEED_OPERATE,
                         ANGLE_INPUT_ALPHA,
                         ANGLE_OUTPUT_ALPHA,
                         SPEED_INPUT_ALPHA,
                         SPEED_OUTPUT_ALPHA,
                         ANGLE_DELTA,
                         ANGLE_ALPHA,
                         SPEED_DELTA,
                         SPEED_ALPHA,
                         PITCH_MAX_ANGULAR_VELOCITY,
                         PITCH_MAX_ANGULAR_IVELOCITY,
                         PITCH_MAX_SPEED_OUTPUT,
                         PITCH_MAX_SPEED_IOUTPUT,
                         ANGLE_DEAD_AREA,
                         SPEED_DEAD_AREA);
    }

    if (dart_rack.state == DART_RACK_MATCH) {
        cascade_PID_init(&dart_rack.pid.cascade_drag_left,
                         DRAG_LEFT_POSITION_OPERATE,
                         DRAG_LEFT_SPEED_OPERATE,
                         POSITION_INPUT_ALPHA,
                         POSITION_OUTPUT_ALPHA,
                         SPEED_INPUT_ALPHA,
                         SPEED_OUTPUT_ALPHA,
                         POSITION_DELTA_INCREASE,
                         POSITION_ALPHA_INCREASE,
                         SPEED_DELTA,
                         SPEED_ALPHA,
                         DRAG_LEFT_MAX_SPEED,
                         DRAG_LEFT_MAX_ISPEED,
                         DRAG_LEFT_MAX_OUTPUT,
                         DRAG_LEFT_MAX_IOUTPUT,
                         POSITION_DEAD_AREA,
                         SPEED_DEAD_AREA);
        cascade_PID_init(&dart_rack.pid.cascade_drag_right,
                         DRAG_RIGHT_POSITION_OPERATE,
                         DRAG_RIGHT_SPEED_OPERATE,
                         POSITION_INPUT_ALPHA,
                         POSITION_OUTPUT_ALPHA,
                         SPEED_INPUT_ALPHA,
                         SPEED_OUTPUT_ALPHA,
                         POSITION_DELTA_INCREASE,
                         POSITION_ALPHA_INCREASE,
                         SPEED_DELTA,
                         SPEED_ALPHA,
                         DRAG_RIGHT_MAX_SPEED,
                         DRAG_RIGHT_MAX_ISPEED,
                         DRAG_RIGHT_MAX_OUTPUT,
                         DRAG_RIGHT_MAX_IOUTPUT,
                         POSITION_DEAD_AREA,
                         SPEED_DEAD_AREA);
        cascade_PID_init(&dart_rack.pid.cascade_load,
                         LOAD_POSITION_OPERATE,
                         LOAD_SPEED_OPERATE,
                         POSITION_INPUT_ALPHA,
                         POSITION_OUTPUT_ALPHA,
                         SPEED_INPUT_ALPHA,
                         SPEED_OUTPUT_ALPHA,
                         POSITION_DELTA_DECREASE,
                         POSITION_ALPHA_DECREASE,
                         SPEED_DELTA,
                         SPEED_ALPHA,
                         LOAD_MAX_SPEED,
                         LOAD_MAX_ISPEED,
                         LOAD_MAX_OUTPUT,
                         LOAD_MAX_IOUTPUT,
                         POSITION_DEAD_AREA,
                         SPEED_DEAD_AREA);
        cascade_PID_init(&dart_rack.pid.cascade_adjust,
                         ADJUST_POSITION_OPERATE,
                         ADJUST_SPEED_OPERATE,
                         POSITION_INPUT_ALPHA,
                         POSITION_OUTPUT_ALPHA,
                         SPEED_INPUT_ALPHA,
                         SPEED_OUTPUT_ALPHA,
                         POSITION_DELTA_DECREASE,
                         POSITION_ALPHA_DECREASE,
                         SPEED_DELTA,
                         SPEED_ALPHA,
                         ADJUST_MAX_SPEED,
                         ADJUST_MAX_ISPEED,
                         ADJUST_MAX_OUTPUT,
                         ADJUST_MAX_IOUTPUT,
                         POSITION_DEAD_AREA,
                         SPEED_DEAD_AREA);
        cascade_PID_init(&dart_rack.pid.yaw,
                         YAW_ANGLE_OPERATE,
                         YAW_SPEED_OPERATE,
                         ANGLE_INPUT_ALPHA,
                         ANGLE_OUTPUT_ALPHA,
                         SPEED_INPUT_ALPHA,
                         SPEED_OUTPUT_ALPHA,
                         ANGLE_DELTA,
                         ANGLE_ALPHA,
                         SPEED_DELTA,
                         SPEED_ALPHA,
                         YAW_MAX_ANGULAR_VELOCITY,
                         YAW_MAX_ANGULAR_IVELOCITY,
                         YAW_MAX_SPEED_OUTPUT,
                         YAW_MAX_SPEED_IOUTPUT,
                         ANGLE_DEAD_AREA,
                         SPEED_DEAD_AREA);
        cascade_PID_init(&dart_rack.pid.pitch,
                         PITCH_ANGLE_OPERATE,
                         PITCH_SPEED_OPERATE,
                         ANGLE_INPUT_ALPHA,
                         ANGLE_OUTPUT_ALPHA,
                         SPEED_INPUT_ALPHA,
                         SPEED_OUTPUT_ALPHA,
                         ANGLE_DELTA,
                         ANGLE_ALPHA,
                         SPEED_DELTA,
                         SPEED_ALPHA,
                         PITCH_MAX_ANGULAR_VELOCITY,
                         PITCH_MAX_ANGULAR_IVELOCITY,
                         PITCH_MAX_SPEED_OUTPUT,
                         PITCH_MAX_SPEED_IOUTPUT,
                         ANGLE_DEAD_AREA,
                         SPEED_DEAD_AREA);
    }
    if (dart_rack.state == DART_RACK_TEST) {
        switch (remote.rc.s[1]) {
            case GIMBAL_CONTROL:
                cascade_PID_init(&dart_rack.pid.yaw,
                                 YAW_ANGLE_OPERATE,
                                 YAW_SPEED_OPERATE,
                                 ANGLE_INPUT_ALPHA,
                                 ANGLE_OUTPUT_ALPHA,
                                 SPEED_INPUT_ALPHA,
                                 SPEED_OUTPUT_ALPHA,
                                 ANGLE_DELTA,
                                 ANGLE_ALPHA,
                                 SPEED_DELTA,
                                 SPEED_ALPHA,
                                 YAW_MAX_ANGULAR_VELOCITY,
                                 YAW_MAX_ANGULAR_IVELOCITY,
                                 YAW_MAX_SPEED_OUTPUT,
                                 YAW_MAX_SPEED_IOUTPUT,
                                 ANGLE_DEAD_AREA,
                                 SPEED_DEAD_AREA);
                cascade_PID_init(&dart_rack.pid.pitch,
                                 PITCH_ANGLE_OPERATE,
                                 PITCH_SPEED_OPERATE,
                                 ANGLE_INPUT_ALPHA,
                                 ANGLE_OUTPUT_ALPHA,
                                 SPEED_INPUT_ALPHA,
                                 SPEED_OUTPUT_ALPHA,
                                 ANGLE_DELTA,
                                 ANGLE_ALPHA,
                                 SPEED_DELTA,
                                 SPEED_ALPHA,
                                 1000,
                                 1000,
                                 500,
                                 500,
                                 ANGLE_DEAD_AREA,
                                 SPEED_DEAD_AREA);
                break;
            case SHOOT_SPEED_CONTROL:
                PID_init(&dart_rack.pid.drag_left,
                         PID_POSITION,
                         DRAG_LEFT_OPERATE,
                         SPEED_INPUT_ALPHA,
                         SPEED_OUTPUT_ALPHA,
                         SPEED_DELTA,
                         SPEED_ALPHA,
                         8000.0f,
                         4000.0f,
                         ZERO_POINT);
                PID_init(&dart_rack.pid.drag_right,
                         PID_POSITION,
                         DRAG_RIGHT_OPERATE,
                         SPEED_INPUT_ALPHA,
                         SPEED_OUTPUT_ALPHA,
                         SPEED_DELTA,
                         SPEED_ALPHA,
                         8000.0f,
                         4000.0f,
                         ZERO_POINT);
                PID_init(&dart_rack.pid.load,
                         PID_POSITION,
                         DRAG_LEFT_OPERATE,
                         SPEED_INPUT_ALPHA,
                         SPEED_OUTPUT_ALPHA,
                         SPEED_DELTA,
                         SPEED_ALPHA,
                         5000.0f,
                         2000.0f,
                         ZERO_POINT);
                PID_init(&dart_rack.pid.adjust,
                         PID_POSITION,
                         DRAG_LEFT_OPERATE,
                         SPEED_INPUT_ALPHA,
                         SPEED_OUTPUT_ALPHA,
                         SPEED_DELTA,
                         SPEED_ALPHA,
                         8000.0f,
                         4000.0f,
                         ZERO_POINT);
                break;
            case SHOOT_POSITION_CONTROL:
                cascade_PID_init(&dart_rack.pid.cascade_drag_left,
                                 DRAG_LEFT_POSITION_OPERATE,
                                 DRAG_LEFT_SPEED_OPERATE,
                                 POSITION_INPUT_ALPHA,
                                 POSITION_OUTPUT_ALPHA,
                                 SPEED_INPUT_ALPHA,
                                 SPEED_OUTPUT_ALPHA,
                                 POSITION_DELTA_DECREASE,
                                 POSITION_ALPHA_DECREASE,
                                 SPEED_DELTA,
                                 SPEED_ALPHA,
                                 DRAG_LEFT_MAX_SPEED,
                                 DRAG_LEFT_MAX_ISPEED,
                                 DRAG_LEFT_MAX_OUTPUT,
                                 DRAG_LEFT_MAX_IOUTPUT,
                                 POSITION_DEAD_AREA,
                                 SPEED_DEAD_AREA);
                cascade_PID_init(&dart_rack.pid.cascade_drag_right,
                                 DRAG_RIGHT_POSITION_OPERATE,
                                 DRAG_RIGHT_SPEED_OPERATE,
                                 POSITION_INPUT_ALPHA,
                                 POSITION_OUTPUT_ALPHA,
                                 SPEED_INPUT_ALPHA,
                                 SPEED_OUTPUT_ALPHA,
                                 POSITION_DELTA_DECREASE,
                                 POSITION_ALPHA_DECREASE,
                                 SPEED_DELTA,
                                 SPEED_ALPHA,
                                 DRAG_RIGHT_MAX_SPEED,
                                 DRAG_RIGHT_MAX_ISPEED,
                                 DRAG_RIGHT_MAX_OUTPUT,
                                 DRAG_RIGHT_MAX_IOUTPUT,
                                 POSITION_DEAD_AREA,
                                 SPEED_DEAD_AREA);
                cascade_PID_init(&dart_rack.pid.cascade_load,
                                 LOAD_POSITION_OPERATE,
                                 LOAD_SPEED_OPERATE,
                                 POSITION_INPUT_ALPHA,
                                 POSITION_OUTPUT_ALPHA,
                                 SPEED_INPUT_ALPHA,
                                 SPEED_OUTPUT_ALPHA,
                                 POSITION_DELTA_DECREASE,
                                 POSITION_ALPHA_DECREASE,
                                 SPEED_DELTA,
                                 SPEED_ALPHA,
                                 LOAD_MAX_SPEED,
                                 LOAD_MAX_ISPEED,
                                 LOAD_MAX_OUTPUT,
                                 LOAD_MAX_IOUTPUT,
                                 POSITION_DEAD_AREA,
                                 SPEED_DEAD_AREA);
                cascade_PID_init(&dart_rack.pid.cascade_adjust,
                                 ADJUST_POSITION_OPERATE,
                                 ADJUST_SPEED_OPERATE,
                                 POSITION_INPUT_ALPHA,
                                 POSITION_OUTPUT_ALPHA,
                                 SPEED_INPUT_ALPHA,
                                 SPEED_OUTPUT_ALPHA,
                                 POSITION_DELTA_DECREASE,
                                 POSITION_ALPHA_DECREASE,
                                 SPEED_DELTA,
                                 SPEED_ALPHA,
                                 ADJUST_MAX_SPEED,
                                 ADJUST_MAX_ISPEED,
                                 ADJUST_MAX_OUTPUT,
                                 ADJUST_MAX_IOUTPUT,
                                 POSITION_DEAD_AREA,
                                 SPEED_DEAD_AREA);
                break;
            default:
                break;
        }
    }
}

/**
 * @brief 函数“gimbal_command_update”根据不同的状态和控制输入更新飞镖架系统的偏航角和俯仰角的命令和输出。
 */
void gimbal_command_update(void)
{
    if (dart_rack.state == DART_RACK_INIT) {
        dart_rack.command.yaw = YAW_ZERO_ANGLE;
        dart_rack.output.yaw  = cascade_PID_calc(&dart_rack.pid.yaw,
                                                 dart_rack.encoder.yaw_angle_encoder.angle,
                                                 dart_rack.encoder.yaw_angle_encoder.angular_velocity,
                                                 dart_rack.command.yaw,
                                                 CIRCLE_ANGLE,
                                                 ZERO_POINT);

        /* pitch角度编码器反馈控制 */
        // dart_rack.command.pitch = PITCH_TARGET_ANGLE;
        // dart_rack.output.pitch  = cascade_PID_calc(&dart_rack.pid.pitch,
        //                                            dart_rack.encoder.pitch_angle_encoder.angle,
        //                                            dart_rack.encoder.pitch_angle_encoder.angular_velocity,
        //                                            dart_rack.command.pitch,
        //                                            CIRCLE_ANGLE,
        //                                            ZERO_POINT);
        /* pitch电机反馈+读卡控制*/
        dart_rack.command.pitch = PITCH_TARGET_DIST;
        dart_rack.output.pitch  = cascade_PID_calc(&dart_rack.pid.pitch,
                                                   dart_rack.motor_measure.gimbal_motor_measure.pitch_dist,
                                                   dart_rack.motor_measure.gimbal_motor_measure.pitch_speed,
                                                   dart_rack.command.pitch,
                                                   ZERO_POINT,
                                                   ZERO_POINT);
    }

    if (dart_rack.state == DART_RACK_MATCH) {
        /* 粗调角度 */
        dart_rack_cmd_flag   = referee.dart_rack_client_cmd.dart_cmd_flag;
        dart_rack_client_cmd = referee.dart_rack_client_cmd.dart_attack_target;
        if (dart_rack_aim_shoot != DART_RACK_AIM_TOWER &&
            (GIMBAL_CMD_YAW_COARSE_KEYMAP == DART_RACK_AIM_TOWER ||
             (dart_rack_cmd_flag && dart_rack_client_cmd == DART_CLIENT_TARGET_TOWER))) {
            dart_rack.target.yaw = YAW_TOWER_ANGLE;
            dart_rack_aim_shoot  = DART_RACK_AIM_TOWER;
        }
        if (dart_rack_aim_shoot != DART_RACK_AIM_BASE &&
            (GIMBAL_CMD_YAW_COARSE_KEYMAP == DART_RACK_AIM_BASE ||
             (dart_rack_cmd_flag && dart_rack_client_cmd == DART_CLIENT_TARGET_BASE))) {
            dart_rack.target.yaw = YAW_BASE_ANGLE;
            dart_rack_aim_shoot  = DART_RACK_AIM_BASE;
        }
        /* 细调角度 */
        if (!is_zero(GIMBAL_CMD_YAW_SLIGHT_KEYMAP)) {
            yaw_cmd_temp = GIMBAL_CMD_YAW_SLIGHT_KEYMAP;
        }
        if (!is_zero(yaw_cmd_temp) && is_zero(GIMBAL_CMD_YAW_SLIGHT_KEYMAP)) {
            dart_rack.target.yaw += yaw_cmd_temp;
            yaw_cmd_temp = ZERO_POINT;
        }
        constrain(dart_rack.target.yaw, YAW_MIN_ANGLE, YAW_MAX_ANGLE);
        /* 角度保护 */
        if (fabsf(dart_rack.target.yaw) < YAW_JUDGE_ANGLE) {
            dart_rack.target.yaw = YAW_TOWER_ANGLE;
        }
        dart_rack.output.yaw = cascade_PID_calc(&dart_rack.pid.yaw,
                                                dart_rack.encoder.yaw_angle_encoder.angle,
                                                dart_rack.encoder.yaw_angle_encoder.angular_velocity,
                                                dart_rack.target.yaw,
                                                CIRCLE_ANGLE,
                                                ZERO_POINT);
        /* pitch角度编码器反馈控制 */
        // dart_rack.target.pitch = PITCH_TARGET_ANGLE;
        // dart_rack.output.pitch  = cascade_PID_calc(&dart_rack.pid.pitch,
        //                                            dart_rack.encoder.pitch_angle_encoder.angle,
        //                                            dart_rack.encoder.pitch_angle_encoder.angular_velocity,
        //                                            dart_rack.target.pitch,
        //                                            CIRCLE_ANGLE,
        //                                            ZERO_POINT);
        /* pitch电机反馈+读卡控制*/
        dart_rack.target.pitch = PITCH_TARGET_DIST;
        dart_rack.output.pitch = cascade_PID_calc(&dart_rack.pid.pitch,
                                                  dart_rack.motor_measure.gimbal_motor_measure.pitch_dist,
                                                  dart_rack.motor_measure.gimbal_motor_measure.pitch_speed,
                                                  dart_rack.target.pitch,
                                                  ZERO_POINT,
                                                  ZERO_POINT);
    }
    if (dart_rack.state == DART_RACK_TEST) {
        /* 角度粗调 */
        switch (remote.rc.s[1]) {
            case GIMBAL_CONTROL:
                if (handle_gimbal_left()) { // 前哨站
                    dart_rack.target.yaw = YAW_TOWER_ANGLE;
                }
                if (handle_gimbal_right()) { // 基地
                    dart_rack.target.yaw = YAW_BASE_ANGLE;
                }
                if (handle_gimbal_down()) { // 复位
                    dart_rack.target.yaw = YAW_ZERO_ANGLE;
                }
                /* 设置零点 */
                if (fabsf(dart_rack.target.yaw) < YAW_JUDGE_ANGLE) dart_rack.target.yaw = YAW_ZERO_ANGLE;
                /* 角度微调*/
                if (!is_zero(GIMBAL_CMD_YAW_SLIGHT_KEYMAP)) {
                    yaw_cmd_temp = GIMBAL_CMD_YAW_SLIGHT_KEYMAP;
                }
                if (!is_zero(yaw_cmd_temp) && is_zero(GIMBAL_CMD_YAW_SLIGHT_KEYMAP)) {
                    dart_rack.target.yaw += yaw_cmd_temp;
                    yaw_cmd_temp = ZERO_POINT;
                }
                loop_constrain(dart_rack.target.yaw, ZERO_POINT, CIRCLE_ANGLE);
                dart_rack.output.yaw = cascade_PID_calc(&dart_rack.pid.yaw,
                                                        dart_rack.encoder.yaw_angle_encoder.angle,
                                                        dart_rack.encoder.yaw_angle_encoder.angular_velocity,
                                                        dart_rack.target.yaw,
                                                        CIRCLE_ANGLE,
                                                        ZERO_POINT);
                /* pitch编码器反馈控制 */
                // if (fabsf(dart_rack.target.pitch) < 10.0f) dart_rack.target.pitch = PITCH_TARGET_ANGLE;
                // dart_rack.target.pitch += GIMBAL_CMD_TEST_PITCH_KEYMAP;
                // constrain(dart_rack.target.pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
                // dart_rack.output.pitch = cascade_PID_calc(&dart_rack.pid.pitch,
                //                                           dart_rack.encoder.pitch_angle_encoder.angle,
                //                                           dart_rack.encoder.pitch_angle_encoder.angular_velocity,
                //                                           dart_rack.target.pitch,
                //                                           CIRCLE_ANGLE,
                //                                           ZERO_POINT);
                /* pitch电机+读卡控制*/
                /* 设置零点 */
                if (fabsf(dart_rack.target.pitch) < 90.0f) dart_rack.target.pitch = PITCH_TARGET_DIST;
                /* 角度微调*/
                if (!is_zero(GIMBAL_CMD_PITCH_SLIGHT_KEYMAP)) {
                    pitch_cmd_temp = GIMBAL_CMD_PITCH_SLIGHT_KEYMAP;
                }
                if (!is_zero(pitch_cmd_temp) && is_zero(GIMBAL_CMD_PITCH_SLIGHT_KEYMAP)) {
                    dart_rack.target.pitch += pitch_cmd_temp;
                    pitch_cmd_temp = ZERO_POINT;
                }
                constrain(dart_rack.target.pitch, PITCH_MIN_DIST, PITCH_MAX_DIST);
                dart_rack.output.pitch = cascade_PID_calc(&dart_rack.pid.pitch,
                                                          dart_rack.motor_measure.gimbal_motor_measure.pitch_dist,
                                                          dart_rack.motor_measure.gimbal_motor_measure.pitch_speed,
                                                          dart_rack.target.pitch,
                                                          ZERO_POINT,
                                                          ZERO_POINT);
                /* 记录结果 */
                // if (handle_gimbal_up()) {
                //     if (in_range(dart_rack.target.yaw, YAW_TOWER_MIN_ANGLE, YAW_TOWER_MAX_ANGLE)) {
                //         YAW_TOWER_ANGLE = dart_rack.target.yaw;
                //     }
                //     if (in_range(dart_rack.target.yaw, YAW_BASE_MIN_ANGLE, YAW_BASE_MAX_ANGLE)) {
                //         YAW_BASE_ANGLE = dart_rack.target.yaw;
                //     }
                // }
                break;
            case SHOOT_SPEED_CONTROL:
                gimbal_no_force();
                break;
            case SHOOT_POSITION_CONTROL:
                gimbal_no_force();
            default:
                break;
        }
    }
    if (dart_rack.state == DART_RACK_NO_FORCE) {
        gimbal_no_force();
    }
}

/**
 * @brief 函数“shoot_command_update”更新命令并控制飞镖架系统的发射电机和伺服系统。
 */
void shoot_command_update(void)
{
    if (dart_rack.state == DART_RACK_MATCH) {
        /* ------------------------------- 行程调节电机提前移动 ------------------------------- */
        if (dart_rack_shoot_state == DART_RACK_SHOOT_READY ||
            dart_rack_shoot_state == DART_RACK_SHOOT_ON_FIRE) {
            if (motor_adjust_state == MOTOR_ADJUST_READY ||
                (motor_adjust_state == MOTOR_ADJUST_TOWER && dart_rack_aim_shoot != DART_RACK_AIM_TOWER) ||
                (motor_adjust_state == MOTOR_ADJUST_BASE && dart_rack_aim_shoot != DART_RACK_AIM_BASE)) {
                switch (dart_rack_aim_shoot) {
                    case DART_RACK_AIM_TOWER:
                        motor_adjust_state      = MOTOR_ADJUST_BUSY;
                        motor_adjust_count      = get_system_time();
                        dart_rack.target.adjust = ADJUST_TOWER_POSITION;
                        break;
                    case DART_RACK_AIM_BASE:
                        motor_adjust_state      = MOTOR_ADJUST_BUSY;
                        motor_adjust_count      = get_system_time();
                        dart_rack.target.adjust = ADJUST_BASE_POSITION;
                        break;
                    default:
                        break;
                }
            }
        }
        if (motor_adjust_state == MOTOR_ADJUST_BUSY) {
            if (get_close(dart_rack.motor_measure.shoot_motor_measure.adjust_dist, dart_rack.target.adjust, ADJUST_POSITION_THRESHOLD) ||
                get_system_time() - motor_adjust_count > MOTOR_ADJUST_MAX_TIME) {
                switch (dart_rack_aim_shoot) {
                    case DART_RACK_AIM_TOWER:
                        motor_adjust_state    = MOTOR_ADJUST_TOWER;
                        dart_rack_shoot_state = DART_RACK_SHOOT_ADJUST;
                        break;
                    case DART_RACK_AIM_BASE:
                        motor_adjust_state    = MOTOR_ADJUST_BASE;
                        dart_rack_shoot_state = DART_RACK_SHOOT_ADJUST;
                        break;
                    default:
                        break;
                }
            }
        }
        // 按键检测
        if (dart_rack_shoot_state == DART_RACK_SHOOT_ADJUST && SHOOT_CMD_MATCH_LOAD_KEYMAP) {
            dart_rack_shoot_state = DART_RACK_SHOOT_LOAD;
        }
        /* ---------------------------------- 飞镖架装填 --------------------------------- */
        if (dart_rack_shoot_state == DART_RACK_SHOOT_LOAD) {
            // 拖拽电机等待
            if (motor_drag_state == MOTOR_DRAG_READY) {
                motor_drag_state            = MOTOR_DRAG_BUSY;
                motor_drag_count            = get_system_time();
                dart_rack.target.drag_left  = DRAG_LEFT_WAIT_POSITION;
                dart_rack.target.drag_right = DRAG_RIGHT_WAIT_POSITION;
            }
            if (motor_drag_state == MOTOR_DRAG_BUSY) {
                if ((get_close(dart_rack.motor_measure.shoot_motor_measure.drag_left_dist, dart_rack.target.drag_left, DRAG_LEFT_POSITION_THRESHOLD) &&
                     get_close(dart_rack.motor_measure.shoot_motor_measure.drag_right_dist, dart_rack.target.drag_right, DRAG_RIGHT_POSITION_THRESHOLD)) ||
                    get_system_time() - motor_drag_count > MOTOR_DRAG_MAX_TIME) {
                    motor_drag_state = MOTOR_DRAG_WAIT;
                }
            }
            if (motor_drag_state == MOTOR_DRAG_WAIT) {
                // 装填舵机下降
                if (servo_load_state == SERVO_LOAD_READY) {
                    servo_load_state = SERVO_LOAD_DOWN;
                    servo_load_count = get_system_time();
                    servo_load_down();
                }
                // 等待飞镖滑落
                if (servo_load_state == SERVO_LOAD_DOWN) {
                    if (get_system_time() - servo_load_count > SERVO_LOAD_MAX_TIME) {
                        servo_load_state = SERVO_LOAD_WAIT;
                        servo_load_count = get_system_time();
                    }
                }
                // 装填舵机上升 飞镖架就位
                if (servo_load_state == SERVO_LOAD_WAIT) {
                    if (get_system_time() - servo_load_count > DART_RACK_SLIP_MAX_TIME) {
                        dart_rack_shoot_state = DART_RACK_SHOOT_MOVE;
                        servo_load_state      = SERVO_LOAD_UP;
                        servo_load_count      = get_system_time();
                        servo_load_up();

                        motor_drag_state = MOTOR_DRAG_BUSY;
                        servo_load_count = get_system_time();
                        switch (dart_rack_aim_shoot) {
                            case DART_RACK_AIM_TOWER:
                                dart_rack.target.drag_left  = DRAG_LEFT_TOWER_POSITION;
                                dart_rack.target.drag_right = DRAG_RIGHT_TOWER_POSITION;
                                break;
                            case DART_RACK_AIM_BASE:
                                dart_rack.target.drag_left  = DRAG_LEFT_BASE_POSITION;
                                dart_rack.target.drag_right = DRAG_RIGHT_BASE_POSITION;
                                break;
                            default:
                                break;
                        }
                    }
                }
            }
        }
        /* ---------------------------------- 飞镖架就位 --------------------------------- */
        if (dart_rack_shoot_state == DART_RACK_SHOOT_MOVE) {
            // 装填舵机上升
            if (servo_load_state == SERVO_LOAD_UP) {
                if (get_system_time() - servo_load_count > SERVO_LOAD_MAX_TIME) {
                    servo_load_state = SERVO_LOAD_READY;
                }
            }
            // 装填电机移动
            if (servo_load_state == SERVO_LOAD_READY) {
                motor_load_state      = MOTOR_LOAD_BUSY;
                motor_load_count      = get_system_time();
                dart_rack.target.load = LOAD_POSITION[dart_rack_num_count];
            }
            if (motor_load_state == MOTOR_LOAD_BUSY) {
                if (get_close(dart_rack.motor_measure.shoot_motor_measure.load_dist, dart_rack.target.load, LOAD_POSITION_THRESHOLD) ||
                    get_system_time() - motor_load_count > MOTOR_LOAD_MAX_TIME) {
                    motor_load_state = MOTOR_LOAD_READY;
                }
            }

            // 拖拽电机移动
            if (motor_drag_state == MOTOR_DRAG_BUSY) {
                if ((get_close(dart_rack.motor_measure.shoot_motor_measure.drag_left_dist, dart_rack.target.drag_left, DRAG_LEFT_POSITION_THRESHOLD) &&
                     get_close(dart_rack.motor_measure.shoot_motor_measure.drag_right_dist, dart_rack.target.drag_right, DRAG_RIGHT_POSITION_THRESHOLD)) ||
                    get_system_time() - motor_drag_count > MOTOR_DRAG_MAX_TIME) {
                    motor_drag_state            = MOTOR_DRAG_RETURN;
                    motor_drag_count            = get_system_time();
                    dart_rack.target.drag_left  = DRAG_LEFT_SAFE_POSITION;
                    dart_rack.target.drag_right = DRAG_RIGHT_SAFE_POSITION;
                }
            }
            if (motor_drag_state == MOTOR_DRAG_RETURN) {
                if ((get_close(dart_rack.motor_measure.shoot_motor_measure.drag_left_dist, dart_rack.target.drag_left, DRAG_LEFT_POSITION_THRESHOLD) &&
                     get_close(dart_rack.motor_measure.shoot_motor_measure.drag_right_dist, dart_rack.target.drag_right, DRAG_RIGHT_POSITION_THRESHOLD)) ||
                    get_system_time() - motor_drag_count > MOTOR_DRAG_MAX_TIME) {
                    motor_drag_state = MOTOR_DRAG_READY;
                }
            }

            if (motor_load_state == MOTOR_LOAD_READY &&
                motor_drag_state == MOTOR_DRAG_READY) {
                dart_rack_shoot_state = DART_RACK_SHOOT_ON_FIRE;
            }
        }

        /* ---------------------------------- 电机控制 ---------------------------------- */
        dart_rack.output.drag_left  = cascade_PID_calc(&dart_rack.pid.cascade_drag_left,
                                                       dart_rack.motor_measure.shoot_motor_measure.drag_left_dist,
                                                       dart_rack.motor_measure.shoot_motor_measure.drag_left_speed,
                                                       dart_rack.target.drag_left,
                                                       ZERO_POINT,
                                                       ZERO_POINT);
        dart_rack.output.drag_right = cascade_PID_calc(&dart_rack.pid.cascade_drag_right,
                                                       dart_rack.motor_measure.shoot_motor_measure.drag_right_dist,
                                                       dart_rack.motor_measure.shoot_motor_measure.drag_right_speed,
                                                       dart_rack.target.drag_right,
                                                       ZERO_POINT,
                                                       ZERO_POINT);
        dart_rack.output.load       = cascade_PID_calc(&dart_rack.pid.cascade_load,
                                                       dart_rack.motor_measure.shoot_motor_measure.load_dist,
                                                       dart_rack.motor_measure.shoot_motor_measure.load_speed,
                                                       dart_rack.target.load,
                                                       ZERO_POINT,
                                                       ZERO_POINT);
        dart_rack.output.adjust     = cascade_PID_calc(&dart_rack.pid.cascade_adjust,
                                                       dart_rack.motor_measure.shoot_motor_measure.adjust_dist,
                                                       dart_rack.motor_measure.shoot_motor_measure.adjust_speed,
                                                       dart_rack.target.adjust,
                                                       ZERO_POINT,
                                                       ZERO_POINT);

        /* ---------------------------------- 发射飞镖 ---------------------------------- */
        switch (remote.rc.s[1]) {
            case SHOOT_HALF_CONTROL:
                // 发射飞镖但不装填
                if (SHOOT_CMD_MATCH_LAUNCH_KEYMAP &&
                    dart_rack_shoot_state == DART_RACK_SHOOT_ON_FIRE) {
                    dart_rack_shoot_state = DART_RACK_SHOOT_LAUNCH;
                    dart_rack_shoot_count = get_system_time();
                    servo_launch_down();
                    break;
                }
                if (dart_rack_shoot_state == DART_RACK_SHOOT_LAUNCH) {
                    if (get_system_time() - dart_rack_shoot_count > DART_RACK_SHOOT_MAX_TIME) {
                        servo_launch_up();
                        // 恢复到准备状态
                        dart_rack_shoot_state = DART_RACK_SHOOT_READY;
                    }
                }
            case SHOOT_FULL_CONTROL:
                // 发射飞镖并装填
                if (SHOOT_CMD_MATCH_LAUNCH_KEYMAP &&
                    dart_rack_shoot_state == DART_RACK_SHOOT_ON_FIRE) {
                    dart_rack_shoot_state = DART_RACK_SHOOT_LAUNCH;
                    dart_rack_shoot_count = get_system_time();
                    servo_launch_down();
                    break;
                }
                if (dart_rack_shoot_state == DART_RACK_SHOOT_LAUNCH) {
                    if (get_system_time() - dart_rack_shoot_count > DART_RACK_SHOOT_MAX_TIME) {
                        servo_launch_up();
                        // 直接到装填状态
                        dart_rack_shoot_state = DART_RACK_SHOOT_LOAD;
                    }
                }
            default:
                break;
        }
    }
    if (dart_rack.state == DART_RACK_TEST) {
        switch (remote.rc.s[1]) {
            case GIMBAL_CONTROL:
                servo_load_up();
                servo_launch_up();
                shoot_no_force();
                break;
            case SHOOT_SPEED_CONTROL:
                if (SHOOT_CMD_TEST_SERVO_LOAD_KEYMAP == SERVO_UP) {
                    servo_load_up();
                }
                if (SHOOT_CMD_TEST_SERVO_LOAD_KEYMAP == SERVO_DOWN) {
                    servo_load_down();
                }

                if (SHOOT_CMD_TEST_SERVO_LAUNCH_KEYMAP == SERVO_UP) {
                    servo_launch_up();
                }
                if (SHOOT_CMD_TEST_SERVO_LAUNCH_KEYMAP == SERVO_DOWN) {
                    servo_launch_down();
                }

                dart_rack.command.drag_left  = DRAG_LEFT_MOTOR_DIRECTION * MOTOR_DRAG_MAX_TEST_SPEED * SHOOT_CMD_TEST_DRAG_KEYMAP;
                dart_rack.command.drag_right = DRAG_RIGHT_MOTOR_DIRECTION * MOTOR_DRAG_MAX_TEST_SPEED * SHOOT_CMD_TEST_DRAG_KEYMAP;
                dart_rack.command.load       = LOAD_MOTOR_DIRECTION * MOTOR_DRAG_MAX_TEST_SPEED * SHOOT_CMD_TEST_LOAD_KEYMAP;
                dart_rack.command.adjust     = ADJUST_MOTOR_DIRECTION * MOTOR_DRAG_MAX_TEST_SPEED * SHOOT_CMD_TEST_ADJUST_KEYMAP;

                dart_rack.output.drag_left  = PID_calc(&dart_rack.pid.drag_left,
                                                       dart_rack.motor_measure.shoot_motor_measure.drag_left_speed,
                                                       dart_rack.command.drag_left,
                                                       ZERO_POINT);
                dart_rack.output.drag_right = PID_calc(&dart_rack.pid.drag_right,
                                                       dart_rack.motor_measure.shoot_motor_measure.drag_right_speed,
                                                       dart_rack.command.drag_right,
                                                       ZERO_POINT);
                dart_rack.output.load       = PID_calc(&dart_rack.pid.load,
                                                       dart_rack.motor_measure.shoot_motor_measure.load_speed,
                                                       dart_rack.command.load,
                                                       ZERO_POINT);
                dart_rack.output.adjust     = PID_calc(&dart_rack.pid.adjust,
                                                       dart_rack.motor_measure.shoot_motor_measure.adjust_speed,
                                                       dart_rack.command.adjust,
                                                       ZERO_POINT);
                break;
            case SHOOT_POSITION_CONTROL:
                /* ---------------------------------- 舵机移动 ---------------------------------- */
                if (SHOOT_CMD_TEST_SERVO_LOAD_KEYMAP == SERVO_UP) {
                    servo_load_up();
                }
                if (SHOOT_CMD_TEST_SERVO_LOAD_KEYMAP == SERVO_DOWN) {
                    servo_load_down();
                }

                if (SHOOT_CMD_TEST_SERVO_LAUNCH_KEYMAP == SERVO_UP) {
                    servo_launch_up();
                }
                if (SHOOT_CMD_TEST_SERVO_LAUNCH_KEYMAP == SERVO_DOWN) {
                    servo_launch_down();
                }

                dart_rack.target.drag_left += DRAG_LEFT_MOTOR_DIRECTION * REMOTE_CONTROL_SENSE * MOTOR_DRAG_MAX_TEST_DIST * SHOOT_CMD_TEST_DRAG_KEYMAP;
                dart_rack.target.drag_right += DRAG_RIGHT_MOTOR_DIRECTION * REMOTE_CONTROL_SENSE * MOTOR_DRAG_MAX_TEST_DIST * SHOOT_CMD_TEST_DRAG_KEYMAP;
                dart_rack.target.load += LOAD_MOTOR_DIRECTION * REMOTE_CONTROL_SENSE * MOTOR_DRAG_MAX_TEST_DIST * SHOOT_CMD_TEST_LOAD_KEYMAP;
                dart_rack.target.adjust += ADJUST_MOTOR_DIRECTION * REMOTE_CONTROL_SENSE * REMOTE_CONTROL_SENSE * MOTOR_DRAG_MAX_TEST_DIST * SHOOT_CMD_TEST_ADJUST_KEYMAP;
                /* ----------------------------------- 复位 ----------------------------------- */
                if (SHOOT_CMD_RESET_KEYMAP) {
                    dart_rack_reset_flag = 1;
                }
                if (dart_rack_reset_flag && SHOOT_CMD_ZERO_CONTROL) {
                    dart_rack_reset_flag        = 0;
                    dart_rack.target.drag_left  = ZERO_POINT;
                    dart_rack.target.drag_right = ZERO_POINT;
                    dart_rack.target.load       = ZERO_POINT;
                }
                constrain(dart_rack.target.drag_left, DRAG_LEFT_MIN_POSIITON, DRAG_LEFT_MAX_POSIION);
                constrain(dart_rack.target.drag_right, DRAG_RIGHT_MIN_POSITION, DRAG_RIGHT_MAX_POSITION);
                constrain(dart_rack.target.load, LOAD_MIN_POSITION, LOAD_MAX_POSITION);
                constrain(dart_rack.target.adjust, ADJUST_MIN_POSITION, ADJUST_MAX_POSIITON);
                dart_rack.output.drag_left  = cascade_PID_calc(&dart_rack.pid.cascade_drag_left,
                                                               dart_rack.motor_measure.shoot_motor_measure.drag_left_dist,
                                                               dart_rack.motor_measure.shoot_motor_measure.drag_left_speed,
                                                               dart_rack.target.drag_left,
                                                               ZERO_POINT,
                                                               ZERO_POINT);
                dart_rack.output.drag_right = cascade_PID_calc(&dart_rack.pid.cascade_drag_right,
                                                               dart_rack.motor_measure.shoot_motor_measure.drag_right_dist,
                                                               dart_rack.motor_measure.shoot_motor_measure.drag_right_speed,
                                                               dart_rack.target.drag_right,
                                                               ZERO_POINT,
                                                               ZERO_POINT);
                dart_rack.output.load       = cascade_PID_calc(&dart_rack.pid.cascade_load,
                                                               dart_rack.motor_measure.shoot_motor_measure.load_dist,
                                                               dart_rack.motor_measure.shoot_motor_measure.load_speed,
                                                               dart_rack.target.load,
                                                               ZERO_POINT,
                                                               ZERO_POINT);
                dart_rack.output.adjust     = cascade_PID_calc(&dart_rack.pid.cascade_adjust,
                                                               dart_rack.motor_measure.shoot_motor_measure.adjust_dist,
                                                               dart_rack.motor_measure.shoot_motor_measure.adjust_speed,
                                                               dart_rack.target.adjust,
                                                               ZERO_POINT,
                                                               ZERO_POINT);
                break;
            default:
                break;
        }
    }
    if (dart_rack.state == DART_RACK_NO_FORCE) {
        servo_no_force();
        shoot_no_force();
    }
}

/**
 * @brief 函数“get_target”将“dart_rack.target”结构的值复制到“target”结构中。
 *
 * @param target 指向 DartRackTarget_t 结构的指针，该结构将填充目标参数的值。
 */
void get_target(DartRackTarget_t *target)
{
    target->yaw        = dart_rack.target.yaw;
    target->pitch      = dart_rack.target.pitch;
    target->drag_left  = dart_rack.target.drag_left;
    target->drag_right = dart_rack.target.drag_right;
    target->load       = dart_rack.target.load;
    target->adjust     = dart_rack.target.adjust;
}

/**
 * @brief 函数“get_encoder_angle”将角度值从“dart_rack”编码器复制到“encoder”结构。
 *
 * @param encoder 指向 DartRackEncoder_t 类型结构的指针。
 */
void get_encoder_angle(DartRackEncoder_t *encoder)
{
    encoder->yaw_angle_encoder.angle   = dart_rack.encoder.yaw_angle_encoder.angle;
    encoder->pitch_angle_encoder.angle = dart_rack.encoder.pitch_angle_encoder.angle;
}

/**
 * @brief 函数“get_state”检索 DartRack 状态机的当前状态。
 *
 * @param state 参数“state”是指向“DartRackStateMachine_e”类型变量的指针。
 */
void get_state(DartRackStateMachine_e *state)
{
    *state = dart_rack.state;
}
