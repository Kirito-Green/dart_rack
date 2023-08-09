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
#include "lazer.h"

static DartRack_t dart_rack;         // ���ڼ�״??
static RC_ctrl_t remote;             // ң����״??
static OfflineMonitor_t offline;     // ģ������״??
static RefereeInformation_t referee; // ����ϵͳ����

/* ����״?? */
static bool_t dart_rack_cmd_flag;                  // �ͻ��������??
static bool_t dart_rack_reset_flag;                // ���ڼܸ�λ��??
static int8_t dart_rack_client_cmd;                // �ͻ�����??
static int8_t dart_rack_aim_shoot;                 // ���ڼ���׼��
static int8_t dart_rack_num_count;                 // ���ڼܷ����??
static DartRackShootState_e dart_rack_shoot_state; // ���ڷ���״??
static MotorLoadState_e motor_load_state;          // װ����״̬��
static MotorDragState_e motor_drag_state;          // ��ק���״̬��
static MotorAdjustState_e motor_adjust_state;      // ���ڵ��״̬��
static ServoLoadState_e servo_load_state;          // װ����״̬��
static ServoLaunchState_e servo_launch_state;      // ������״̬��
static ServoBlockState_e servo_block_state;        // ������״̬��

/* �¼���¼ */
static uint32_t dart_rack_shoot_count; // ���ڼܷ����??
static uint32_t motor_load_count;      // װ������ʱ
static uint32_t motor_drag_count;      // ��ק�����ʱ
static uint32_t motor_adjust_count;    // ���ڵ����ʱ
static uint32_t servo_load_count;      // װ������ʱ
static uint32_t servo_launch_count;    // ��������ʱ
static uint32_t servo_block_count;     // ��������ʱ

static uint32_t motor_drag_max_time; // �ɱ�ʱ��
static uint32_t motor_load_max_time;
static uint32_t motor_adjust_max_time;

/* ��̨λ�� */
static fp32 temp_cmd;
static fp32 temp_yaw_cmd;
static fp32 temp_pitch_cmd;

/* ���λ�� */
static fp32 temp_adjust_cmd;
static fp32 adjust_tower_position = ADJUST_TOWER_POSITION;
static fp32 adjust_base_position  = ADJUST_BASE_POSITION;

/* ����Ƕ� */
static int16_t tim_right_first_compare  = TIM_RIGHT_FIRST_COMPARE_UP;
static int16_t tim_right_second_compare = TIM_RIGHT_SECOND_COMPARE_UP;
static int16_t tim_left_first_compare   = TIM_LEFT_FIRST_COMPARE_UP;
static int16_t tim_left_second_compare  = TIM_LEFT_SECOND_COMPARE_UP;
static int16_t tim_bottom_compare       = TIM_BOTTOM_COMPARE_UP;
static int16_t tim_top_compare          = TIM_TOP_COMPARE_UP;

/* ������־ */
static bool_t dart_rack_double_shoot_flag;
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
void servo_block_up(void);
void servo_block_mid(void);
void servo_block_down(void);
void gimbal_no_force(void);
void shoot_no_force(void);
void tfcard_read(void);
void dart_rack_shoot(void);
void dart_rack_state_update(void);
void dart_rack_match_state_reset(void);
void motor_measure_update(void);
void motor_target_update(void);
void encoder_measure_update(void);
void motor_algorithm_update(void);
int16_t dist_cal_max_time(fp32 dist, int16_t time_measure);

/**
 * @brief ������launch_thread�����ϸ��¸������ݲ�ִ����������ϵͳ��صĶ���??
 *
 * @param argument ��argument��������ָ��Ҫ���ݸ��̺߳������κθ������ݵ�ָ�롣��������Ϊ��void const
 * *��������ζ����������ָ���κ��������ݵ�ָ�롣����������£���û���ں�����ʹ�ã���������Ժ���
 */
void launch_thread(void const *argument)
{
    tfcard_read();
    for (;;) {
        remote = *get_remote_control_point();    // ��ȡң������??
        device_offline_monitor_update(&offline); // ��ȡģ����������
        get_referee_information(&referee);       // ��ȡ����ϵͳ����

        dart_rack_state_update(); // ���ڼ�״̬ת??
        motor_measure_update();   // ������ݸ���
        encoder_measure_update(); // ��������??
        motor_algorithm_update(); // ���PID����
        gimbal_command_update();  // ��ָ̨�����
        shoot_command_update();   // ����ָ�����
        dart_rack_shoot();        // ���ڷ���

        osDelay(1);
    }
}

/**
 * @brief ������servo_init�����ж�ģʽ����?? TIM4 ?? TIM5 �Ļ�����ʱ��??
 *  TIM4 ?? TIM5���ڿ��ƶ����װ��ͷ���
 */
void servo_init(void)
{
    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_Base_Start(&htim5);
}

/**
 * @brief ������servo_start������������ͬ��ʱ���Ķ��ͨ��?? PWM ���??
 *  �������pwm���
 */
void servo_start(void)
{
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}

/**
 * @brief ������servo_no_force��ֹͣ������ͬ��ʱ���Ķ��ͨ��?? PWM �ź�??
 * ����ر�pwm���
 */
void servo_no_force(void)
{
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_4);
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
}

/**
 * @brief ������servo_update�����¶���ĽǶ�??
 */
void servo_update(void)
{
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, tim_right_first_compare);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, tim_right_second_compare);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, tim_left_first_compare);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, tim_left_second_compare);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, tim_bottom_compare);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, tim_top_compare);
}

/**
 * @brief ������servo_load_up��������Ƕ�����Ϊ��װ��״??
 */
void servo_load_up(void)
{
    tim_right_first_compare  = TIM_RIGHT_FIRST_COMPARE_UP;
    tim_right_second_compare = TIM_RIGHT_SECOND_COMPARE_UP;
    tim_left_first_compare   = TIM_LEFT_FIRST_COMPARE_UP;
    tim_left_second_compare  = TIM_LEFT_SECOND_COMPARE_UP;
}

/**
 * ������servo_load_down��������Ƕ�����Ϊװ��״??
 */
void servo_load_down(void)
{
    tim_right_first_compare  = TIM_RIGHT_FIRST_COMPARE_DOWN;
    tim_right_second_compare = TIM_RIGHT_SECOND_COMPARE_DOWN;
    tim_left_first_compare   = TIM_LEFT_FIRST_COMPARE_DOWN;
    tim_left_second_compare  = TIM_LEFT_SECOND_COMPARE_DOWN;
}

/**
 * ������servo_launch_up�����������Ƕ�����Ϊ������״??
 */
void servo_launch_up(void)
{
    tim_bottom_compare = TIM_BOTTOM_COMPARE_UP;
}

/**
 * ������servo_launch_down�����������Ƕ�����Ϊ����״??
 */
void servo_launch_down(void)
{
    tim_bottom_compare = TIM_BOTTOM_COMPARE_DOWN;
}

/**
 * ������servo_block_up�����������Ƕ�����Ϊ׼��״??
 */
void servo_block_up(void)
{
    tim_top_compare = TIM_TOP_COMPARE_UP;
}

/**
 * ������servo_block_mid�����������Ƕ�����Ϊб���赲״??
 */
void servo_block_mid(void)
{
    tim_top_compare = TIM_TOP_COMPARE_MID;
}

/**
 * ����servo_block_down���������Ƕ�����Ϊ��ֱ�赲״??
 */
void servo_block_down(void)
{
    tim_top_compare = TIM_TOP_COMPARE_DOWN;
}

/**
 * ������gimbal_no_force������̨����������??
 */
void gimbal_no_force(void)
{
    dart_rack.output.yaw   = ZERO_POINT;
    dart_rack.output.pitch = ZERO_POINT;
}

/**
 * ������shoot_no_force�����������������??
 */
void shoot_no_force(void)
{
    dart_rack.output.drag_left  = ZERO_POINT;
    dart_rack.output.drag_right = ZERO_POINT;
    dart_rack.output.load       = ZERO_POINT;
    dart_rack.output.adjust     = ZERO_POINT;
}

/**
 * @brief ������tfcard_read�����Զ�δ� TF ����ȡ���ݣ�����ɹ�����µ������ֵ��Ŀ��??
 *
 * @return ʲô��û��(��Ч)??
 */
void tfcard_read(void)
{
    uint8_t flag;
    tfcard_start();
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
 * @brief ������dart_rack_shoot�����ƴӷ��ڼܷ�����ڵ��ŷ��͵���˶�??
 */
void dart_rack_shoot(void)
{
    servo_update();
    shoot_motor_control(dart_rack.output.drag_left,
                        dart_rack.output.drag_right,
                        dart_rack.output.load,
                        dart_rack.output.adjust);
    gimbal_motor_control(dart_rack.output.yaw,
                         dart_rack.output.pitch);
}

/**
 * @brief ������dart_rack_state_update�����ݸ���������������·��ڼܵ�״̬??
 *
 * @return ���� `dart_rack_state_update` û����ʽ����ֵ??
 */
DartRackStateMachine_e drslast;
DartRackStateMachine_e drsthis;
void dart_rack_state_update(void)
{
    drslast = dart_rack.state; // ��¼֮ǰ״??

    /* ������߱��� */
    if (offline.yaw_motor == DEVICE_OFFLINE ||
        offline.pitch_motor == DEVICE_OFFLINE ||
        offline.drag_left_motor == DEVICE_OFFLINE ||
        offline.drag_right_motor == DEVICE_OFFLINE ||
        offline.load_motor == DEVICE_OFFLINE ||
        offline.adjust_motor == DEVICE_OFFLINE ||
        offline.yaw_angle_encoder == DEVICE_OFFLINE
        //  offline.pitch_angle_encoder == DEVICE_OFFLINE
    ) {
        dart_rack.state = DART_RACK_NO_FORCE;
        return;
    }

    /* ң�������߱�?? */
    if (offline.remote == DEVICE_OFFLINE) {
        dart_rack.state = DART_RACK_NO_FORCE;
        return;
    }

    /* yaw PITCH�ᳬλ��?? */
    // if (dart_rack.encoder.yaw_angle_encoder.angle < YAW_MIN_ANGLE ||
    // dart_rack.encoder.yaw_angle_encoder.angle > YAW_MAX_ANGLE
    //		 dart_rack.encoder.pitch_angle_encoder.angle < PITCH_MIN_ANGLE ||
    // dart_rack.encoder.pitch_angle_encoder.angle > PITCH_MAX_ANGLE
    // ) {
    //     dart_rack.state = DART_RACK_NO_FORCE;
    //     return;
    // }

    /* ���ڼ�״̬������ */
    switch (remote.rc.s[0]) {
            /* �Ҳ��˴����ϣ����ڼܽ������ģ??,��ģʽ��Ħ���ֲ����ѵ��� */
        case RC_SW_UP:                                // ���״̬��??
            if (dart_rack.state == DART_RACK_MATCH) { // ��Ҫ�ظ����??
                return;
            }
            servo_start();
            dart_rack_match_state_reset();
            dart_rack_client_cmd = 0;                   // �ͻ���Ĭ��ǰ��վ
            dart_rack_num_count  = 0;                   // ���ڷ���������������
            temp_yaw_cmd         = ZERO_POINT;          // YAW��ָ��ͼ�??
            dart_rack.target.yaw = YAW_TOWER_ANGLE;     // Ĭ����׼ǰ��??
            dart_rack_aim_shoot  = DART_RACK_AIM_TOWER; // ��׼��־
            lazer_on();                                 // ������??
            dart_rack.state = DART_RACK_MATCH;

            break;
            /* �Ҳ��˴��м䣬���ڼܽ������ģ??,��ģʽ��Ħ�����ٶȿɵ�?? */
        case RC_SW_MID:
            if (dart_rack.state == DART_RACK_TEST) { // ��Ҫ�ظ����??
                return;
            }
            dart_rack_reset_flag = 0;
            servo_start();
            lazer_on(); // ������??
            dart_rack.state = DART_RACK_TEST;
            break;
            /* �Ҳ��˴����£���ң�������ݳ������ڼܽ�������ģʽ */
        case RC_SW_DOWN:
            lazer_off(); // �رռ�??
            dart_rack.state = DART_RACK_NO_FORCE;
            break;
        default:
            lazer_off(); // �رռ�??
            dart_rack.state = DART_RACK_NO_FORCE;
            break;
    }

    drsthis = dart_rack.state; // ��¼֮��״??
}

/**
 * @brief ������motor_measure_update�����·��ڼ�����̨���������Ĳ���ֵ??
 */
void motor_measure_update(void)
{
    gimbal_motor_measure_update(&dart_rack.motor_measure.gimbal_motor_measure);
    shoot_motor_measure_update(&dart_rack.motor_measure.shoot_motor_measure);
}

/**
 * @brief ������encoder_measure_update�����½Ƕȱ������Ĳ���ֵ??
 */
void encoder_measure_update(void)
{
    angle_encoder_measure_update(&dart_rack.encoder);
}

/**
 * @brief motor_target_update ����������������Ĳ���ֵ���·��ڼ�ϵͳ�и��������Ŀ��λ��??
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
 * @brief ������motor_algorithm_update������ϵͳ�ĵ�ǰ״̬��ʼ�������¸��ֵ���㷨�� PID ������??
 *
 * @return �ú����������κ�ֵ??
 */
void motor_algorithm_update(void)
{
    /* �����ظ������㷨 */
    if (drsthis == drslast) {
        return;
    }

    PID_init(&dart_rack.pid.drag_left,
             PID_POSITION,
             DRAG_LEFT_OPERATE,
             SPEED_INPUT_ALPHA,
             SPEED_OUTPUT_ALPHA,
             SPEED_DELTA,
             SPEED_ALPHA,
             DRAG_LEFT_MAX_OUTPUT,
             DRAG_LEFT_MAX_IOUTPUT,
             SPEED_DEAD_AREA);
    PID_init(&dart_rack.pid.drag_right,
             PID_POSITION,
             DRAG_RIGHT_OPERATE,
             SPEED_INPUT_ALPHA,
             SPEED_OUTPUT_ALPHA,
             SPEED_DELTA,
             SPEED_ALPHA,
             DRAG_RIGHT_MAX_OUTPUT,
             DRAG_RIGHT_MAX_IOUTPUT,
             SPEED_DEAD_AREA);
    PID_init(&dart_rack.pid.load,
             PID_POSITION,
             LOAD_OPERATE,
             SPEED_INPUT_ALPHA,
             SPEED_OUTPUT_ALPHA,
             SPEED_DELTA,
             SPEED_ALPHA,
             LOAD_MAX_OUTPUT,
             LOAD_MAX_IOUTPUT,
             SPEED_DEAD_AREA);
    PID_init(&dart_rack.pid.adjust,
             PID_POSITION,
             ADJUST_OPERATE,
             SPEED_INPUT_ALPHA,
             SPEED_OUTPUT_ALPHA,
             SPEED_DELTA,
             SPEED_ALPHA,
             ADJUST_MAX_OUTPUT,
             ADJUST_MAX_IOUTPUT,
             SPEED_DEAD_AREA);

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
                     DRAG_LEFT_POSITION_THRESHOLD,
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
                     DRAG_RIGHT_POSITION_THRESHOLD,
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
                     LOAD_POSITION_THRESHOLD,
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
                     ADJUST_POSITION_THRESHOLD,
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
    // cascade_PID_init(&dart_rack.pid.pitch,
    //                  PITCH_ANGLE_OPERATE,
    //                  PITCH_SPEED_OPERATE,
    //                  ANGLE_INPUT_ALPHA,
    //                  ANGLE_OUTPUT_ALPHA,
    //                  SPEED_INPUT_ALPHA,
    //                  SPEED_OUTPUT_ALPHA,
    //                  ANGLE_DELTA,
    //                  ANGLE_ALPHA,
    //                  SPEED_DELTA,
    //                  SPEED_ALPHA,
    //                  PITCH_MAX_ANGULAR_VELOCITY,
    //                  PITCH_MAX_ANGULAR_IVELOCITY,
    //                  PITCH_MAX_SPEED_OUTPUT,
    //                  PITCH_MAX_SPEED_IOUTPUT,
    //                  ANGLE_DEAD_AREA,
    //                  SPEED_DEAD_AREA);
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
                     PITCH_MAX_SPEED,
                     PITCH_MAX_ISPEED,
                     PITCH_MAX_OUTPUT,
                     PITCH_MAX_IOUTPUT,
                     PITCH_POSITION_THRESHOLD,
                     SPEED_DEAD_AREA);
}

/**
 * @brief ������gimbal_command_update�����ݲ�ͬ��״̬�Ϳ���������·��ڼ�ϵͳ��ƫ���Ǻ͸����ǵ���������??
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

        /* pitch�Ƕȱ�����������?? */
        // dart_rack.command.pitch = PITCH_TARGET_ANGLE;
        // dart_rack.output.pitch  = cascade_PID_calc(&dart_rack.pid.pitch,
        //                                            dart_rack.encoder.pitch_angle_encoder.angle,
        //                                            dart_rack.encoder.pitch_angle_encoder.angular_velocity,
        //                                            dart_rack.command.pitch,
        //                                            CIRCLE_ANGLE,
        //                                            ZERO_POINT);
        /* pitch�������+��������*/
        dart_rack.command.pitch = PITCH_TARGET_DIST;
        dart_rack.output.pitch  = cascade_PID_calc(&dart_rack.pid.pitch,
                                                   dart_rack.motor_measure.gimbal_motor_measure.pitch_dist,
                                                   dart_rack.motor_measure.gimbal_motor_measure.pitch_speed,
                                                   dart_rack.command.pitch,
                                                   ZERO_POINT,
                                                   ZERO_POINT);
    }

    if (dart_rack.state == DART_RACK_MATCH) {
        /* �ֵ��Ƕ� */
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
        /* ϸ���Ƕ� */
        temp_cmd = GIMBAL_CMD_YAW_SLIGHT_KEYMAP;
        if (!is_zero(temp_cmd)) {
            temp_yaw_cmd = temp_cmd;
        }
        if (!is_zero(temp_yaw_cmd) && is_zero(GIMBAL_CMD_YAW_SLIGHT_KEYMAP)) {
            dart_rack.target.yaw += temp_yaw_cmd;
            temp_yaw_cmd = ZERO_POINT;
        }
        constrain(dart_rack.target.yaw, YAW_MIN_ANGLE, YAW_MAX_ANGLE);
        /* �Ƕȱ��� */
        if (fabsf(dart_rack.target.yaw) < YAW_JUDGE_ANGLE) {
            dart_rack.target.yaw = YAW_TOWER_ANGLE;
        }
        dart_rack.output.yaw = YAW_MOTOR_DIRECTION * cascade_PID_calc(&dart_rack.pid.yaw,
                                                                      dart_rack.encoder.yaw_angle_encoder.angle,
                                                                      dart_rack.encoder.yaw_angle_encoder.angular_velocity,
                                                                      dart_rack.target.yaw,
                                                                      CIRCLE_ANGLE,
                                                                      ZERO_POINT);

        /* pitch�Ƕȱ�����������?? */
        // dart_rack.target.pitch = PITCH_TARGET_ANGLE;
        // dart_rack.output.pitch  = cascade_PID_calc(&dart_rack.pid.pitch,
        //                                            dart_rack.encoder.pitch_angle_encoder.angle,
        //                                            dart_rack.encoder.pitch_angle_encoder.angular_velocity,
        //                                            dart_rack.target.pitch,
        //                                            CIRCLE_ANGLE,
        //                                            ZERO_POINT);
        /* pitch�������+��������*/
        dart_rack.target.pitch = PITCH_TARGET_DIST;
        dart_rack.output.pitch = cascade_PID_calc(&dart_rack.pid.pitch,
                                                  dart_rack.motor_measure.gimbal_motor_measure.pitch_dist,
                                                  dart_rack.motor_measure.gimbal_motor_measure.pitch_speed,
                                                  dart_rack.target.pitch,
                                                  ZERO_POINT,
                                                  ZERO_POINT);
    }
    if (dart_rack.state == DART_RACK_TEST) {
        /* �Ƕȴֵ� */
        switch (remote.rc.s[1]) {
            case GIMBAL_CONTROL:
                if (handle_gimbal_left()) { // ǰ��??
                    dart_rack.target.yaw = YAW_TOWER_ANGLE;
                }
                if (handle_gimbal_right()) { // ����
                    dart_rack.target.yaw = YAW_BASE_ANGLE;
                }
                if (handle_gimbal_down()) { // ��λ
                    dart_rack.target.yaw = YAW_ZERO_ANGLE;
                }
                /* ������� */
                if (fabsf(dart_rack.target.yaw) < YAW_JUDGE_ANGLE) dart_rack.target.yaw = YAW_ZERO_ANGLE;
                /* �Ƕ�΢��*/
                temp_cmd = GIMBAL_CMD_YAW_SLIGHT_KEYMAP;
                if (!is_zero(temp_cmd)) {
                    temp_yaw_cmd = temp_cmd;
                }
                if (!is_zero(temp_yaw_cmd) && is_zero(GIMBAL_CMD_YAW_SLIGHT_KEYMAP)) {
                    dart_rack.target.yaw += temp_yaw_cmd;
                    temp_yaw_cmd = ZERO_POINT;
                }
                loop_constrain(dart_rack.target.yaw, ZERO_POINT, CIRCLE_ANGLE);
                dart_rack.output.yaw = YAW_MOTOR_DIRECTION * cascade_PID_calc(&dart_rack.pid.yaw,
                                                                              dart_rack.encoder.yaw_angle_encoder.angle,
                                                                              dart_rack.encoder.yaw_angle_encoder.angular_velocity,
                                                                              dart_rack.target.yaw,
                                                                              CIRCLE_ANGLE,
                                                                              ZERO_POINT);
                /* pitch������������?? */
                // if (fabsf(dart_rack.target.pitch) < 10.0f) dart_rack.target.pitch = PITCH_TARGET_ANGLE;
                // dart_rack.target.pitch += GIMBAL_CMD_TEST_PITCH_KEYMAP;
                // constrain(dart_rack.target.pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
                // dart_rack.output.pitch = cascade_PID_calc(&dart_rack.pid.pitch,
                //                                           dart_rack.encoder.pitch_angle_encoder.angle,
                //                                           dart_rack.encoder.pitch_angle_encoder.angular_velocity,
                //                                           dart_rack.target.pitch,
                //                                           CIRCLE_ANGLE,
                //                                           ZERO_POINT);
                /* pitch���+��������*/
                /* ������� */
                if (fabsf(dart_rack.target.pitch) > PITCH_JUDGE_DIST) dart_rack.target.pitch = PITCH_TARGET_DIST;
                /* �Ƕ�΢��*/
                if (!is_zero(GIMBAL_CMD_PITCH_SLIGHT_KEYMAP)) {
                    temp_pitch_cmd = GIMBAL_CMD_PITCH_SLIGHT_KEYMAP;
                }
                if (!is_zero(temp_pitch_cmd) && is_zero(GIMBAL_CMD_PITCH_SLIGHT_KEYMAP)) {
                    dart_rack.target.pitch += temp_pitch_cmd;
                    temp_pitch_cmd = ZERO_POINT;
                }
                constrain(dart_rack.target.pitch, PITCH_MIN_DIST, PITCH_MAX_DIST);
                dart_rack.output.pitch = cascade_PID_calc(&dart_rack.pid.pitch,
                                                          dart_rack.motor_measure.gimbal_motor_measure.pitch_dist,
                                                          dart_rack.motor_measure.gimbal_motor_measure.pitch_speed,
                                                          dart_rack.target.pitch,
                                                          ZERO_POINT,
                                                          ZERO_POINT);
                /* ��¼��� */
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
 * @brief ������shoot_command_update������������Ʒ��ڼ�ϵͳ�ķ��������ŷ�ϵͳ??
 */
void shoot_command_update(void)
{
    if (dart_rack.state == DART_RACK_MATCH) {
        /* -------------------------------- ȫ�����������?? -------------------------------- */
        if (dart_rack_num_count >= DART_RACK_NUM) {
            return;
        }
        /* ------------------------------- �г̵��ڵ����ǰ�ƶ� ------------------------------- */
        if (motor_adjust_state == MOTOR_ADJUST_READY ||
            (motor_adjust_state != MOTOR_ADJUST_TOWER && dart_rack_aim_shoot == DART_RACK_AIM_TOWER) ||
            (motor_adjust_state == MOTOR_ADJUST_TOWER && dart_rack_aim_shoot == DART_RACK_AIM_TOWER && (!is_zero(dart_rack.target.adjust - adjust_tower_position))) ||
            (motor_adjust_state != MOTOR_ADJUST_BASE && dart_rack_aim_shoot == DART_RACK_AIM_BASE) ||
            (motor_adjust_state == MOTOR_ADJUST_BASE && dart_rack_aim_shoot == DART_RACK_AIM_BASE && (!is_zero(dart_rack.target.adjust - adjust_base_position)))) {
            switch (dart_rack_aim_shoot) {
                case DART_RACK_AIM_TOWER:
                    motor_adjust_state      = MOTOR_ADJUST_BUSY;
                    motor_adjust_count      = get_system_time();
                    dart_rack.target.adjust = adjust_tower_position;
                    motor_adjust_max_time   = dist_cal_max_time(dart_rack.motor_measure.shoot_motor_measure.adjust_dist - dart_rack.target.adjust,
                                                                MOTOR_ADJUST_TIME_MEASURE);
                    break;
                case DART_RACK_AIM_BASE:
                    motor_adjust_state      = MOTOR_ADJUST_BUSY;
                    motor_adjust_count      = get_system_time();
                    dart_rack.target.adjust = adjust_base_position;
                    motor_adjust_max_time   = dist_cal_max_time(dart_rack.motor_measure.shoot_motor_measure.adjust_dist - dart_rack.target.adjust,
                                                                MOTOR_ADJUST_TIME_MEASURE);
                    break;
                default:
                    break;
            }
        }
        if (motor_adjust_state == MOTOR_ADJUST_BUSY) {
            if (get_close(dart_rack.motor_measure.shoot_motor_measure.adjust_dist, dart_rack.target.adjust, ADJUST_POSITION_THRESHOLD) ||
                // get_system_time() - motor_adjust_count > MOTOR_ADJUST_MAX_TIME) {
                /* ��̬���ʱ?? */
                get_system_time() - motor_adjust_count > motor_adjust_max_time) {
                switch (dart_rack_aim_shoot) {
                    case DART_RACK_AIM_TOWER:
                        motor_adjust_state = MOTOR_ADJUST_TOWER;
                        if (dart_rack_shoot_state == DART_RACK_SHOOT_READY) { //? ��ֹ��װ���??
                            dart_rack_shoot_state = DART_RACK_SHOOT_ADJUST;
                        }
                        break;
                    case DART_RACK_AIM_BASE:
                        motor_adjust_state = MOTOR_ADJUST_BASE;
                        if (dart_rack_shoot_state == DART_RACK_SHOOT_READY) {
                            dart_rack_shoot_state = DART_RACK_SHOOT_ADJUST;
                        }
                        break;
                    default:
                        break;
                }
            }
        }
        // ������??
        if (dart_rack_shoot_state == DART_RACK_SHOOT_ADJUST && SHOOT_CMD_MATCH_LOAD_KEYMAP) {
            dart_rack_shoot_state = DART_RACK_SHOOT_LOAD;
        }
        /* ---------------------------------- ���ڼ�װ?? --------------------------------- */
        if (dart_rack_shoot_state == DART_RACK_SHOOT_LOAD) {
            /* --------------------------------- step 1 --------------------------------- */
            // ��ק����ȴ�
            if (motor_drag_state == MOTOR_DRAG_READY) {
                motor_drag_state            = MOTOR_DRAG_BUSY;
                motor_drag_count            = get_system_time();
                dart_rack.target.drag_left  = DRAG_LEFT_WAIT_POSITION;
                dart_rack.target.drag_right = DRAG_RIGHT_WAIT_POSITION;
                motor_drag_max_time         = get_max(dist_cal_max_time(dart_rack.motor_measure.shoot_motor_measure.drag_left_dist - dart_rack.target.drag_left,
                                                                        MOTOR_DRAG_TIME_MEASURE),
                                                      dist_cal_max_time(dart_rack.motor_measure.shoot_motor_measure.drag_right_dist - dart_rack.target.drag_right,
                                                                        MOTOR_DRAG_TIME_MEASURE));
            }
            if (motor_drag_state == MOTOR_DRAG_BUSY) {
                if ((get_close(dart_rack.motor_measure.shoot_motor_measure.drag_left_dist, dart_rack.target.drag_left, DRAG_LEFT_POSITION_THRESHOLD) &&
                     get_close(dart_rack.motor_measure.shoot_motor_measure.drag_right_dist, dart_rack.target.drag_right, DRAG_RIGHT_POSITION_THRESHOLD)) ||
                    // get_system_time() - motor_drag_count > MOTOR_DRAG_MAX_TIME) {
                    get_system_time() - motor_drag_count > motor_drag_max_time) {
                    motor_drag_state = MOTOR_DRAG_WAIT;
                }
            }
            // �������½�
            if (servo_block_state == SERVO_BLOCK_READY) {
                servo_block_state = SERVO_BLOCK_DOWN;
                servo_block_count = get_system_time();
                servo_block_down();
            }
            if (servo_block_state == SERVO_BLOCK_DOWN) {
                if (get_system_time() - servo_block_count > SERVO_BLOCK_MOVE_MAX_TIME) {
                    servo_block_state = SERVO_BLOCK_WAIT;
                    servo_block_count = get_system_time();
                }
            }
            /* --------------------------------- step 2 --------------------------------- */
            if (motor_drag_state == MOTOR_DRAG_WAIT &&
                servo_block_state == SERVO_BLOCK_WAIT) {
                // װ�����½�
                if (servo_load_state == SERVO_LOAD_READY) {
                    servo_load_state = SERVO_LOAD_DOWN;
                    servo_load_count = get_system_time();
                    servo_load_down();
                }
                // �ȴ����ڻ���
                if (servo_load_state == SERVO_LOAD_DOWN) {
                    if (get_system_time() - servo_load_count > SERVO_LOAD_MAX_TIME) {
                        servo_load_state = SERVO_LOAD_WAIT;
                        servo_load_count = get_system_time();
                    }
                }
                /* --------------------------------- step 3 --------------------------------- */
                // װ�������� ���������� ���ڼܾ�??
                if (servo_load_state == SERVO_LOAD_WAIT) {
                    if (get_system_time() - servo_load_count > DART_RACK_SLIP_MAX_TIME) {
                        dart_rack_shoot_state = DART_RACK_SHOOT_MOVE;

                        servo_load_state = SERVO_LOAD_UP;
                        servo_load_count = get_system_time();
                        servo_load_up();

                        // servo_block_state = SERVO_BLOCK_UP;
                        // servo_block_count = get_system_time();
                        // servo_block_up();
                        servo_block_state = SERVO_BLOCK_MID;
                        servo_block_count = get_system_time();
                        servo_block_mid();
                    }
                }
            }
        }
        /* ---------------------------------- ���ڼܾ�?? --------------------------------- */
        if (dart_rack_shoot_state == DART_RACK_SHOOT_MOVE) {
            /* -------------------------------- step 3.1 -------------------------------- */
            // װ��������
            if (servo_load_state == SERVO_LOAD_UP) {
                if (get_system_time() - servo_load_count > SERVO_LOAD_MAX_TIME) {
                    servo_load_state = SERVO_LOAD_READY;
                }
            }
            /* -------------------------------- step 4.1 -------------------------------- */
            // װ�����ƶ�
            //! ��������bug
            if (servo_load_state == SERVO_LOAD_READY &&
                motor_load_state == MOTOR_LOAD_READY) {
                motor_load_state      = MOTOR_LOAD_BUSY;
                motor_load_count      = get_system_time();
                dart_rack.target.load = LOAD_POSITION[dart_rack_num_count];
                motor_load_max_time   = dist_cal_max_time(dart_rack.motor_measure.shoot_motor_measure.load_dist - dart_rack.target.load,
                                                          MOTOR_LOAD_TIME_MEASURE);
            }
            if (motor_load_state == MOTOR_LOAD_BUSY) {
                if (get_close(dart_rack.motor_measure.shoot_motor_measure.load_dist, dart_rack.target.load, LOAD_POSITION_THRESHOLD) ||
                    // get_system_time() - motor_load_count > MOTOR_LOAD_MAX_TIME) {
                    get_system_time() - motor_load_count > motor_load_max_time) {
                    // motor_load_state = MOTOR_LOAD_READY;
                    motor_load_state = MOTOR_LOAD_RETURN;
                    dart_rack.target.load += LOAD_RETURN_DIST;
                    constrain(dart_rack.target.load, LOAD_MIN_POSITION, LOAD_MAX_POSITION);
                    motor_load_max_time = dist_cal_max_time(dart_rack.motor_measure.shoot_motor_measure.load_dist - dart_rack.target.load,
                                                            MOTOR_LOAD_TIME_MEASURE);
                }
            }
            //? װ������λ
            if (motor_load_state == MOTOR_LOAD_RETURN) {
                if (get_close(dart_rack.motor_measure.shoot_motor_measure.load_dist, dart_rack.target.load, LOAD_POSITION_THRESHOLD) ||
                    get_system_time() - motor_load_count > motor_load_max_time) {
                    motor_load_state = MOTOR_LOAD_READY;
                }
            }
            /* -------------------------------- step 3.2 -------------------------------- */
            // ����������
            // if (servo_block_state == SERVO_BLOCK_UP) {
            //     if (get_system_time() - servo_block_count > SERVO_BLOCK_MAX_TIME) {
            //         servo_block_state = SERVO_BLOCK_READY;
            //     }
            // }
            // �������ƶ�??45��б��
            if (servo_block_state == SERVO_BLOCK_MID) {
                if (get_system_time() - servo_block_count > SERVO_BLOCK_MOVE_MAX_TIME / 2) {
                    servo_block_state = SERVO_BLOCK_WAIT;
                    servo_block_count = get_system_time();
                }
            }
            // �������ȴ�
            if (servo_block_state == SERVO_BLOCK_WAIT) {
                if (get_system_time() - servo_block_count > SERVO_BLOCK_WAIT_MAX_TIME) {
                    servo_block_state = SERVO_BLOCK_UP;
                    servo_block_count = get_system_time();
                    servo_block_up();
                }
            }
            if (servo_block_state == SERVO_BLOCK_UP) {
                if (get_system_time() - servo_block_count > SERVO_BLOCK_MOVE_MAX_TIME / 2) {
                    servo_block_state = SERVO_BLOCK_READY;
                }
            }
            /* -------------------------------- step 4.2 -------------------------------- */
            // ��ק����½�
            //! Ŀ��ֵ������??
            if (servo_block_state == SERVO_BLOCK_READY &&
                motor_drag_state == MOTOR_DRAG_WAIT) {
                motor_drag_state = MOTOR_DRAG_BUSY;
                motor_drag_count = get_system_time();
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
                motor_drag_max_time = get_max(dist_cal_max_time(dart_rack.motor_measure.shoot_motor_measure.drag_left_dist - dart_rack.target.drag_left,
                                                                MOTOR_DRAG_TIME_MEASURE),
                                              dist_cal_max_time(dart_rack.motor_measure.shoot_motor_measure.drag_right_dist - dart_rack.target.drag_right,
                                                                MOTOR_DRAG_TIME_MEASURE));
            }
            // ��ק����ƶ�
            if (motor_drag_state == MOTOR_DRAG_BUSY) {
                if ((get_close(dart_rack.motor_measure.shoot_motor_measure.drag_left_dist, dart_rack.target.drag_left, DRAG_LEFT_POSITION_THRESHOLD) &&
                     get_close(dart_rack.motor_measure.shoot_motor_measure.drag_right_dist, dart_rack.target.drag_right, DRAG_RIGHT_POSITION_THRESHOLD)) ||
                    // get_system_time() - motor_drag_count > MOTOR_DRAG_MAX_TIME) {
                    get_system_time() - motor_drag_count > motor_drag_max_time) {
                    motor_drag_state            = MOTOR_DRAG_RETURN;
                    motor_drag_count            = get_system_time();
                    dart_rack.target.drag_left  = DRAG_LEFT_SAFE_POSITION;
                    dart_rack.target.drag_right = DRAG_RIGHT_SAFE_POSITION;
                    motor_drag_max_time         = get_max(dist_cal_max_time(dart_rack.motor_measure.shoot_motor_measure.drag_left_dist - dart_rack.target.drag_left,
                                                                            MOTOR_DRAG_TIME_MEASURE),
                                                          dist_cal_max_time(dart_rack.motor_measure.shoot_motor_measure.drag_right_dist - dart_rack.target.drag_right,
                                                                            MOTOR_DRAG_TIME_MEASURE));
                }
            }
            /* -------------------------------- step 5.2 -------------------------------- */
            if (motor_drag_state == MOTOR_DRAG_RETURN) {
                if ((get_close(dart_rack.motor_measure.shoot_motor_measure.drag_left_dist, dart_rack.target.drag_left, DRAG_LEFT_POSITION_THRESHOLD) &&
                     get_close(dart_rack.motor_measure.shoot_motor_measure.drag_right_dist, dart_rack.target.drag_right, DRAG_RIGHT_POSITION_THRESHOLD)) ||
                    // get_system_time() - motor_drag_count > MOTOR_DRAG_MAX_TIME) {
                    get_system_time() - motor_drag_count > motor_drag_max_time) {
                    motor_drag_state = MOTOR_DRAG_READY;
                }
            }

            if (motor_load_state == MOTOR_LOAD_READY &&
                motor_drag_state == MOTOR_DRAG_READY) {
                dart_rack_shoot_state = DART_RACK_SHOOT_ON_FIRE;
            }
        }

        /* ---------------------------------- ������� ---------------------------------- */
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
        /* ---------------------------------- ����΢�� ---------------------------------- */
        temp_cmd = SHOOT_CMD_MATCH_ADJUST_KEYMAP;
        switch (remote.rc.s[1]) {
            case DIST_CONTROL_FIRST_LEVEL:
                if (!is_zero(temp_cmd)) {
                    temp_adjust_cmd = DIST_CONTROL_FIRST_LEVEL_SENSE * temp_cmd;
                }
                if (!is_zero(temp_adjust_cmd) && (!SHOOT_CMD_MATCH_ADJUST_KEYMAP)) {
                    if (dart_rack_aim_shoot == DART_RACK_AIM_TOWER) {
                        adjust_tower_position += temp_adjust_cmd;
                        constrain(adjust_tower_position, ADJUST_MIN_POSITION, ADJUST_MAX_POSIITON);
                    }
                    if (dart_rack_aim_shoot == DART_RACK_AIM_BASE) {
                        adjust_base_position += temp_adjust_cmd;
                        constrain(adjust_base_position, ADJUST_MIN_POSITION, ADJUST_MAX_POSIITON);
                    }
                    temp_adjust_cmd = ZERO_POINT;
                }
                break;
            case DIST_CONTROL_SECOND_LEVEL:
                if (!is_zero(SHOOT_CMD_MATCH_ADJUST_KEYMAP)) {
                    temp_adjust_cmd = DIST_CONTROL_SECOND_LEVEL_SENSE * SHOOT_CMD_MATCH_ADJUST_KEYMAP;
                }
                if (!is_zero(temp_adjust_cmd) && (!SHOOT_CMD_MATCH_ADJUST_KEYMAP)) {
                    if (dart_rack_aim_shoot == DART_RACK_AIM_TOWER) {
                        adjust_tower_position += temp_adjust_cmd;
                        constrain(adjust_tower_position, ADJUST_MIN_POSITION, ADJUST_MAX_POSIITON);
                    }
                    if (dart_rack_aim_shoot == DART_RACK_AIM_BASE) {
                        adjust_base_position += temp_adjust_cmd;
                        constrain(adjust_base_position, ADJUST_MIN_POSITION, ADJUST_MAX_POSIITON);
                    }
                    temp_adjust_cmd = ZERO_POINT;
                }
                break;
            default:
                break;
        }
        /* ---------------------------------- ������� ---------------------------------- */
        switch (remote.rc.s[1]) {
            case SHOOT_FULL_CONTROL:
                // ������ڲ�װ�����(�ڶ����Զ���??)
                if ((SHOOT_CMD_MATCH_LAUNCH_KEYMAP ||
                     dart_rack_num_count % 2 == 1) && // ˫��ģʽ
                    dart_rack_shoot_state == DART_RACK_SHOOT_ON_FIRE) {
                    dart_rack_shoot_state = DART_RACK_SHOOT_LAUNCH;
                    dart_rack_shoot_count = get_system_time();
                    servo_launch_down();
                    break;
                }
                if (dart_rack_shoot_state == DART_RACK_SHOOT_LAUNCH) {
                    if (get_system_time() - dart_rack_shoot_count > DART_RACK_SHOOT_MAX_TIME) {
                        servo_launch_up();
                        // ֱ�ӵ�װ��״??
                        dart_rack_shoot_state = DART_RACK_SHOOT_LOAD;
                        dart_rack_num_count++;
                    }
                }
            case SHOOT_HALF_CONTROL:
                // printf("state: %u %u %u\r\n", motor_load_state, motor_drag_state, dart_rack_shoot_state);
                // ������ڵ���װ��
                if (SHOOT_CMD_MATCH_LAUNCH_KEYMAP &&
                    dart_rack_shoot_state == DART_RACK_SHOOT_ON_FIRE) {
                    // printf("launch ready\r\n");
                    dart_rack_shoot_state = DART_RACK_SHOOT_LAUNCH;
                    dart_rack_shoot_count = get_system_time();
                    servo_launch_down();
                    break;
                }
                if (dart_rack_shoot_state == DART_RACK_SHOOT_LAUNCH) {
                    if (get_system_time() - dart_rack_shoot_count > DART_RACK_SHOOT_MAX_TIME) {
                        servo_launch_up();
                        // �ָ���׼��״??
                        // printf("launch done\r\n");
                        dart_rack_shoot_state = DART_RACK_SHOOT_ADJUST;
                        dart_rack_num_count++;
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

                if (SHOOT_CMD_TEST_SERVO_BLOCK_KEYMAP == SERVO_UP) {
                    servo_block_up();
                }
                if (SHOOT_CMD_TEST_SERVO_BLOCK_KEYMAP == SERVO_MID) {
                    servo_block_mid();
                }
                if (SHOOT_CMD_TEST_SERVO_BLOCK_KEYMAP == SERVO_DOWN) {
                    servo_block_down();
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
                /* ---------------------------------- ����ƶ� ---------------------------------- */
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

                if (SHOOT_CMD_TEST_SERVO_BLOCK_KEYMAP == SERVO_UP) {
                    servo_block_up();
                }
                if (SHOOT_CMD_TEST_SERVO_BLOCK_KEYMAP == SERVO_MID) {
                    servo_block_mid();
                }
                if (SHOOT_CMD_TEST_SERVO_BLOCK_KEYMAP == SERVO_DOWN) {
                    servo_block_down();
                }

                dart_rack.target.drag_left += DRAG_LEFT_MOTOR_DIRECTION * REMOTE_CONTROL_SENSE * MOTOR_DRAG_MAX_TEST_DIST * SHOOT_CMD_TEST_DRAG_KEYMAP;
                dart_rack.target.drag_right += DRAG_RIGHT_MOTOR_DIRECTION * REMOTE_CONTROL_SENSE * MOTOR_DRAG_MAX_TEST_DIST * SHOOT_CMD_TEST_DRAG_KEYMAP;
                dart_rack.target.load += LOAD_MOTOR_DIRECTION * REMOTE_CONTROL_SENSE * MOTOR_DRAG_MAX_TEST_DIST * SHOOT_CMD_TEST_LOAD_KEYMAP;
                dart_rack.target.adjust += ADJUST_MOTOR_DIRECTION * REMOTE_CONTROL_SENSE * REMOTE_CONTROL_SENSE * MOTOR_DRAG_MAX_TEST_DIST * SHOOT_CMD_TEST_ADJUST_KEYMAP;
                /* ----------------------------------- ��λ ----------------------------------- */
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
 * �ú������������������������ʱ��??
 *
 * @param dist ������dist��������Ϊ��fp32����������??32λ�������Ǵ����������ı���??
 * @param time_measure ��λ����ĳ���ʱ��(�Ժ���Ϊ��λ)??
 *
 * @return ����������һ?? int16_t ֵ??
 */
int16_t dist_cal_max_time(fp32 dist, int16_t time_measure)
{
    return roundf(fabs(dist) * time_measure);
}

/**
 * @brief ������get_target������dart_rack.target���ṹ��ֵ���Ƶ���target���ṹ��??
 *
 * @param target ָ�� DartRackTarget_t �ṹ��ָ�룬�ýṹ�����Ŀ�������ֵ??
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
 * ������motor_reset_target�����÷��ڼ����϶������϶��͸��ص�Ŀ��ֵ??
 */
void motor_reset_target(void)
{
    dart_rack.target.drag_left  = ZERO_POINT;
    dart_rack.target.drag_right = ZERO_POINT;
    dart_rack.target.load       = ZERO_POINT;
}

/**
 * @brief ������get_encoder_angle�����Ƕ�ֵ�ӡ�dart_rack�����������Ƶ���encoder���ṹ??
 *
 * @param encoder ָ�� DartRackEncoder_t ���ͽṹ��ָ��??
 */
void get_encoder_angle(DartRackEncoder_t *encoder)
{
    encoder->yaw_angle_encoder.angle   = dart_rack.encoder.yaw_angle_encoder.angle;
    encoder->pitch_angle_encoder.angle = dart_rack.encoder.pitch_angle_encoder.angle;
}

/**
 * @brief ������get_state����?? DartRack ״̬���ĵ�ǰ״̬??
 *
 * @param state ������state����ָ��DartRackStateMachine_e�����ͱ�����ָ��??
 */
void get_state(DartRackStateMachine_e *state)
{
    *state = dart_rack.state;
}

void dart_rack_match_state_reset(void)
{
    dart_rack_shoot_state = DART_RACK_SHOOT_READY;
    motor_load_state      = MOTOR_LOAD_READY;
    motor_drag_state      = MOTOR_DRAG_READY;
    motor_adjust_state    = MOTOR_ADJUST_READY;
    servo_load_state      = SERVO_LOAD_READY;
    servo_launch_state    = SERVO_LAUNCH_READY;
    servo_block_state     = SERVO_BLOCK_READY;
}
