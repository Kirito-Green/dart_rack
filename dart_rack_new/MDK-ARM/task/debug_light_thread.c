#include "debug_light_thread.h"
#include "interrupt_service.h"
#include "cmsis_os.h"

static OfflineMonitor_t offline; // 模块离线信息

void debug_light_show(void);

/**
 * @brief  debug_light_thread函数不断更新设备的离线数据，并通过LED灯显示工作状态。
 *
 * @param argument “argument”参数是指向需要传递给线程函数的任何附加数据的指针。在这种情况下，它不会被使用并且可以被忽略。
 */
void debug_light_thread(void const *argument)
{
    for (;;) {
        device_offline_monitor_update(&offline); // 获取模块离线数据
        debug_light_show();                      // LED展示模块工作状况

        osDelay(100);
    }
}

/**
 * @brief  函数“debug_light_show”检查各种电机和传感器的状态并相应地设置相应的 LED，如果任何设备离线，它会设置有问题的 LED。
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
    if (offline.pitch_angle_encoder == DEVICE_OFFLINE) {
        flag_problem = 1;
        LED_PITCH_ANGLE_ENCODER_RESET;
    } else {
        LED_PITCH_ANGLE_ENCODER_SET;
    }
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
