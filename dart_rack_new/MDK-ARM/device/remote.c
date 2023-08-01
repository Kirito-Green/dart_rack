/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "remote.h"

#include "main.h"

#include "bsp_usart.h"
#include "string.h"
#include "stdlib.h"

// 遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

// 取正函数
static int16_t rc_abs(int16_t value);
/**
 * @brief          remote control protocol resolution
 * @param[in]      sbus_buf: raw data point
 * @param[out]     rc_ctrl: remote control data struct point
 * @retval         none
 */
/**
 * @brief          遥控器协议解析
 * @param[in]      sbus_buf: 原生数据指针
 * @param[out]     rc_ctrl: 遥控器数据指
 * @retval         none
 */

// remote control data
// 遥控器控制变量
RC_ctrl_t rc_ctrl;
// 接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
uint8_t sbus_rx_buf[SBUS_RX_BUF_NUM];

// 边沿触发检测
uint16_t key_former_channel = 0;
uint16_t key_jump_channel   = 0;
uint16_t key_used           = 0;

/**
 * @brief          remote control init
 * @param[in]      none
 * @retval         none
 */
/**
 * @brief          遥控器初始化
 * @param[in]      none
 * @retval         none
 */
void remote_control_init(void)
{
    usart1_rx_dma_init(sbus_rx_buf, SBUS_RX_BUF_NUM);
}
/**
 * @brief          get remote control data point
 * @param[in]      none
 * @retval         remote control data point
 */
/**
 * @brief          获取遥控器数据指针
 * @param[in]      none
 * @retval         遥控器数据指针
 */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

/**
 * @brief该函数检查是否有任何 RC 数据值或开关高于某个错误阈值，如果有错误则返回 1，否则返回 0。
 * 用于判断遥控器数据是否出错，
 *
 * @return 无符号 8 位整数 (uint8_t)。
 */
uint8_t RC_data_is_error(void)
{
    // 禁止使用go to语句！！！！！！
    if (rc_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE) {
        memset(&rc_ctrl, 0, sizeof(rc_ctrl));
        return 1;
    }
    if (rc_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE) {
        memset(&rc_ctrl, 0, sizeof(rc_ctrl));
        return 1;
    }
    if (rc_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE) {
        memset(&rc_ctrl, 0, sizeof(rc_ctrl));
        return 1;
    }
    if (rc_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE) {
        memset(&rc_ctrl, 0, sizeof(rc_ctrl));
        return 1;
    }
    if (rc_ctrl.rc.s[0] == 0) {
        memset(&rc_ctrl, 0, sizeof(rc_ctrl));
        return 1;
    }
    if (rc_ctrl.rc.s[1] == 0) {
        memset(&rc_ctrl, 0, sizeof(rc_ctrl));
        return 1;
    }
    return 0;
}

void slove_RC_lost(void)
{
    usart1_rx_dma_restart(SBUS_RX_BUF_NUM);
}
void slove_data_error(void)
{
    usart1_rx_dma_restart(SBUS_RX_BUF_NUM);
}

/**
 * @brief 函数“rc_abs”返回输入整数的绝对值。
 *
 * @param value 参数“value”的类型为int16_t，它是一个有符号的16位整数。
 *
 * @return 函数“rc_abs”返回输入“value”的绝对值。
 */
static int16_t rc_abs(int16_t value)
{
    if (value > 0) {
        return value;
    } else {
        return -value;
    }
}
/**
 * @brief          remote control protocol resolution
 * @param[in]      sbus_buf: raw data point
 * @param[out]     rc_ctrl: remote control data struct point
 * @retval         none
 */
/**
 * @brief          遥控器协议解析
 * @param[in]      sbus_buf: 原生数据指针
 * @param[out]     rc_ctrl: 遥控器数据指
 * @retval         none
 */
void sbus_to_rc(void)
{
    key_former_channel   = rc_ctrl.key.v;
    rc_ctrl.rc.s_last[0] = rc_ctrl.rc.s[0];
    rc_ctrl.rc.s_last[1] = rc_ctrl.rc.s[1];

    rc_ctrl.rc.ch[0] = (sbus_rx_buf[0] | (sbus_rx_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl.rc.ch[1] = ((sbus_rx_buf[1] >> 3) | (sbus_rx_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl.rc.ch[2] = ((sbus_rx_buf[2] >> 6) | (sbus_rx_buf[3] << 2) |          //!< Channel 2
                        (sbus_rx_buf[4] << 10)) &
                       0x07ff;
    rc_ctrl.rc.ch[3]      = ((sbus_rx_buf[4] >> 1) | (sbus_rx_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl.rc.s[0]       = ((sbus_rx_buf[5] >> 4) & 0x0003);                         //!< Switch left
    rc_ctrl.rc.s[1]       = ((sbus_rx_buf[5] >> 4) & 0x000C) >> 2;                    //!< Switch right
    rc_ctrl.mouse.y       = -(sbus_rx_buf[6] | (sbus_rx_buf[7] << 8));                //!< Mouse X axis
    rc_ctrl.mouse.x       = -(sbus_rx_buf[8] | (sbus_rx_buf[9] << 8));                //!< Mouse Y axis
    rc_ctrl.mouse.z       = sbus_rx_buf[10] | (sbus_rx_buf[11] << 8);                 //!< Mouse Z axis
    rc_ctrl.mouse.press_l = sbus_rx_buf[12];                                          //!< Mouse Left Is Press ?
    rc_ctrl.mouse.press_r = sbus_rx_buf[13];                                          //!< Mouse Right Is Press ?
    rc_ctrl.key.v         = sbus_rx_buf[14] | (sbus_rx_buf[15] << 8);                 //!< KeyBoard value
    rc_ctrl.rc.ch[4]      = sbus_rx_buf[16] | (sbus_rx_buf[17] << 8);                 // NULL

    rc_ctrl.rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[4] -= RC_CH_VALUE_OFFSET;

    /* 防止微小扰动 */
    if (rc_ctrl.rc.ch[0] <= 10 && rc_ctrl.rc.ch[0] >= -10) {
        rc_ctrl.rc.ch[1] = 0;
    }
    if (rc_ctrl.rc.ch[2] <= 10 && rc_ctrl.rc.ch[2] >= -10) {
        rc_ctrl.rc.ch[2] = 0;
    }
    if (rc_ctrl.rc.ch[3] <= 10 && rc_ctrl.rc.ch[3] >= -10) {
        rc_ctrl.rc.ch[3] = 0;
    }
    if (rc_ctrl.rc.ch[1] <= 10 && rc_ctrl.rc.ch[1] >= -10) {
        rc_ctrl.rc.ch[1] = 0;
    }

    // 防止数据溢出
    if ((abs(rc_ctrl.rc.ch[1]) > 660) ||
        (abs(rc_ctrl.rc.ch[2]) > 660) ||
        (abs(rc_ctrl.rc.ch[3]) > 660) ||
        (abs(rc_ctrl.rc.ch[0]) > 660)) {
        memset(&rc_ctrl.rc, 0, sizeof(rc_ctrl.rc));
    }

    key_jump_channel = (rc_ctrl.key.v ^ key_former_channel);
}

// 键盘应用功能
// 键盘区域分为长按触发的功能区和点按触发的功能区
// 其中 W、A、S、D、SHIFT、CTRL、F 为长按触发的功能区
// 其他按键为点按功能区

// 长按触发区域的键值检测

bool_t cheak_key_press(uint16_t key)
{
    if ((rc_ctrl.key.v & key) == 0)
        return 0;

    return 1;
}

bool_t cheak_key_press_once(uint16_t key)
{
    if ((rc_ctrl.key.v & key) == 0) {
        key_used &= (~key);
        return 0;
    }

    if ((key_jump_channel & key) == 0) {
        return 0;
    } else {
        if ((key_used & key) == 0) {
            key_used |= key;
            return 1;
        }
        return 0;
    }
}

// 归一化摇杆值
fp32 remote_channel_rightX(void)
{
    return (rc_ctrl.rc.ch[1] / 660.0f);
}
fp32 remote_channel_rightY(void)
{
    return -(rc_ctrl.rc.ch[0] / 660.0f);
}
fp32 remote_channel_leftX(void)
{
    return (rc_ctrl.rc.ch[3] / 660.0f);
}
fp32 remote_channel_leftY(void)
{
    return (rc_ctrl.rc.ch[2] / 660.0f);
}
fp32 remote_dial()
{
    return (rc_ctrl.rc.ch[4] / 660.0f);
}

// 归一化鼠标移动
fp32 mouse_moveX(void)
{
    return (rc_ctrl.mouse.x / 32768.0f);
}
fp32 mouse_moveY(void)
{
    return (rc_ctrl.mouse.y / 32768.0f);
}

// 鼠标左右键
bool_t mouse_press_left(void)
{
    return rc_ctrl.mouse.press_l;
}
bool_t mouse_press_right(void)
{
    return rc_ctrl.mouse.press_r;
}

// 拨杆位置检测
bool_t handle_chassis_up(void)
{ // 左半区域
    return (rc_ctrl.rc.ch[3] > 600);
}
bool_t handle_chassis_down(void)
{
    return (rc_ctrl.rc.ch[3] < -600);
}
bool_t handle_chassis_right(void)
{
    return (rc_ctrl.rc.ch[2] > 600);
}
bool_t handle_chassis_left(void)
{
    return (rc_ctrl.rc.ch[2] < -600);
}
bool_t handle_gimbal_up(void)
{ // 右半区域
    return (rc_ctrl.rc.ch[1] > 600);
}
bool_t handle_gimbal_down(void)
{
    return (rc_ctrl.rc.ch[1] < -600);
}
bool_t handle_gimbal_right(void)
{
    return (rc_ctrl.rc.ch[0] > 600);
}
bool_t handle_gimbal_left(void)
{
    return (rc_ctrl.rc.ch[0] < -600);
}
bool_t handle_dial_down(void)
{
    return (rc_ctrl.rc.ch[4] > 600);
}
bool_t handle_dial_up(void)
{
    return (rc_ctrl.rc.ch[4] < -600);
}
/* 归位 */
bool_t handle_chassis_resetX(void)
{
    return (rc_ctrl.rc.ch[3] > -50 && rc_ctrl.rc.ch[3] < 50);
}
bool_t handle_chassis_resetY(void)
{
    return (rc_ctrl.rc.ch[2] > -50 && rc_ctrl.rc.ch[2] < 50);
}
bool_t handle_gimbal_resetX(void)
{
    return (rc_ctrl.rc.ch[1] > -50 && rc_ctrl.rc.ch[1] < 50);
}
bool_t handle_gimbal_resetY(void)
{
    return (rc_ctrl.rc.ch[0] > -50 && rc_ctrl.rc.ch[0] < 50);
}
/* 挂挡 */
bool_t switch_right_up(void)
{
    return (rc_ctrl.rc.s[0] == RC_SW_UP);
}
bool_t switch_right_mid(void)
{
    return (rc_ctrl.rc.s[0] == RC_SW_MID);
}
bool_t switch_right_down(void)
{
    return (rc_ctrl.rc.s[0] == RC_SW_DOWN);
}
bool_t switch_left_up(void)
{
    return (rc_ctrl.rc.s[1] == RC_SW_UP);
}
bool_t switch_left_mid(void)
{
    return (rc_ctrl.rc.s[1] == RC_SW_MID);
}
bool_t switch_left_down(void)
{
    return (rc_ctrl.rc.s[1] == RC_SW_DOWN);
}

/**
 * @brief 函数 int_normalized_limit 接受一个输入整数，如果它在 -1 和 1 之间，则返回它，否则根据输入的符号返回 -1 或 1。
 *
 * @param input 输入参数是一个整数值，需要在-1到1的范围内进行归一化。
 *
 * @return 标准化后的输入值在-1到1的范围内。
 */
int int_normalized_limit(int input)
{
    if (input > 1) {
        input = 1;
    }
    if (input < -1) {
        input = -1;
    }
    return input;
}

/**
 * @brief 函数“fp32_normalized_limit”将输入值限制在-1.0到1.0的范围内。
 *
 * @param input 输入参数是单精度浮点数（fp32）。
 *
 * @return 检查并可能修改输入值以确保其落在 -1.0f 到 1.0f 的范围内。
 */
fp32 fp32_normalized_limit(fp32 input)
{
    if (input > 1.0f) {
        input = 1.0f;
    } else if (input < -1.0f) {
        input = -1.0f;
    }
    return input;
}
