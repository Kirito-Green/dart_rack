#include "DebugLight.h"
#include "InterruptService.h"
#include "cmsis_os.h"

static OfflineMonitor_t Offline; // 模块离线信息

void DebugLightThread(void const *argument)
{
    for (;;) {
        DeviceOfflineMonitorUpdate(&Offline); // 获取模块离线数据
        DebugLightShow();                     // LED展示模块工作状况

        osDelay(1);
    }
}

void DebugLightShow(void)
{
    bool_t flag_problem = 0;
    /* SHOOT1 */
    if (Offline.ShootMotor[0] == DEVICE_OFFLINE) {
        flag_problem = 1;
        LED_SHOOT1_RESET;
    } else {
        LED_SHOOT1_SET;
    }
    /* SHOOT2 */
    if (Offline.ShootMotor[1] == DEVICE_OFFLINE) {
        flag_problem = 1;
        LED_SHOOT2_RESET;
    } else {
        LED_SHOOT2_SET;
    }
    /* SHOOT3 */
    if (Offline.ShootMotor[2] == DEVICE_OFFLINE) {
        flag_problem = 1;
        LED_SHOOT3_RESET;
    } else {
        LED_SHOOT3_SET;
    }
    /* SHOOT4 */
    if (Offline.ShootMotor[3] == DEVICE_OFFLINE) {
        flag_problem = 1;
        LED_SHOOT4_RESET;
    } else {
        LED_SHOOT4_SET;
    }
    /* YAW */
    if (Offline.YawMotor == DEVICE_OFFLINE) {
        flag_problem = 1;
        LED_YAW_RESET;
    } else {
        LED_YAW_SET;
    }
    /* PITCH */
    if (Offline.PitchMotor == DEVICE_OFFLINE) {
        flag_problem = 1;
        LED_PITCH_RESET;
    } else {
        LED_PITCH_SET;
    }
    /* CHAIN */
    if (Offline.ChainMotor == DEVICE_OFFLINE) {
        flag_problem = 1;
        LED_CHAIN_RESET;
    } else {
        LED_CHAIN_SET;
    }
    /* YAW ANGLE ENCODER */
    if (Offline.YawAngleEncoder == DEVICE_OFFLINE) {
        flag_problem = 1;
        LED_YAW_ANGLE_ENCODER_RESET;
    } else {
        LED_YAW_ANGLE_ENCODER_SET;
        ;
    }
    /* PITCH ANGLE ENCODER */
    if (Offline.PitchAngleEncoder == DEVICE_OFFLINE) {
        flag_problem = 1;
        LED_PITCH_ANGLE_ENCODER_RESET;
    } else {
        LED_PITCH_ANGLE_ENCODER_SET;
    }
    /* REMOTE */
    if (Offline.Remote == DEVICE_OFFLINE) {
        flag_problem = 1;
    }
    /* PROBLEM */
    if (flag_problem) {
        LED_PROBLEM_SET;
    } else {
        LED_PROBLEM_RESET;
    }
}
