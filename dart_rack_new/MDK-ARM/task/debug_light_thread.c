#include "debug_light_thread.h"
#include "interrupt_service.h"
#include "cmsis_os.h"

static OfflineMonitor_t offline; // ģ��������Ϣ

void debug_light_show(void);

/**
 * @brief  debug_light_thread�������ϸ����豸���������ݣ���ͨ��LED����ʾ����״̬��
 *
 * @param argument ��argument��������ָ����Ҫ���ݸ��̺߳������κθ������ݵ�ָ�롣����������£������ᱻʹ�ò��ҿ��Ա����ԡ�
 */
void debug_light_thread(void const *argument)
{
    for (;;) {
        device_offline_monitor_update(&offline); // ��ȡģ����������
        debug_light_show();                      // LEDչʾģ�鹤��״��

        osDelay(100);
    }
}

/**
 * @brief  ������debug_light_show�������ֵ���ʹ�������״̬����Ӧ��������Ӧ�� LED������κ��豸���ߣ���������������� LED��
 */
void debug_light_show(void)
{
    bool_t flag_problem = 0;
    /* DRAG_LEFT */
    if (offline.drag_left_motor == DEVICE_OFFLINE) {
        flag_problem = 1;
        LED_DRAG_LEFT_RESET;
    } else {
        LED_DRAG_LEFT_SET;
    }
    /* DRAG_RIGHT */
    if (offline.drag_right_motor == DEVICE_OFFLINE) {
        flag_problem = 1;
        LED_DRAG_RIGHT_RESET;
    } else {
        LED_DRAG_RIGHT_SET;
    }
    /* LOAD */
    if (offline.load_motor == DEVICE_OFFLINE) {
        flag_problem = 1;
        LED_LOAD_RESET;
    } else {
        LED_LOAD_SET;
    }
    /* ADJUST */
    if (offline.adjust_motor == DEVICE_OFFLINE) {
        flag_problem = 1;
        LED_ADJUST_RESET;
    } else {
        LED_ADJUST_SET;
    }
    /* YAW */
    if (offline.yaw_motor == DEVICE_OFFLINE) {
        flag_problem = 1;
        LED_YAW_RESET;
    } else {
        LED_YAW_SET;
    }
    /* PITCH */
    if (offline.pitch_motor == DEVICE_OFFLINE) {
        flag_problem = 1;
        LED_PITCH_RESET;
    } else {
        LED_PITCH_SET;
    }
    /* YAW ANGLE ENCODER */
    if (offline.yaw_angle_encoder == DEVICE_OFFLINE) {
        flag_problem = 1;
        LED_YAW_ANGLE_ENCODER_RESET;
    } else {
        LED_YAW_ANGLE_ENCODER_SET;
    }
    /* PITCH ANGLE ENCODER */
    // if (offline.pitch_angle_encoder == DEVICE_OFFLINE) {
    //     flag_problem = 1;
    //     LED_PITCH_ANGLE_ENCODER_RESET;
    // } else {
    //     LED_PITCH_ANGLE_ENCODER_SET;
    // }
    /* REMOTE */
    if (offline.remote == DEVICE_OFFLINE) {
        flag_problem = 1;
        LED_REMOTE_RESET;
    } else {
        LED_REMOTE_SET;
    }
    /* PROBLEM */
    if (flag_problem) {
        LED_PROBLEM_SET;
    } else {
        LED_PROBLEM_RESET;
    }
}
