/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      ң��������ң������ͨ������SBUS��Э�鴫�䣬����DMA���䷽ʽ��ԼCPU
  *             ��Դ�����ô��ڿ����ж�������������ͬʱ�ṩһЩ��������DMA������
  *             �ķ�ʽ��֤�Ȳ�ε��ȶ��ԡ�
  * @note       ��������ͨ�������ж�����������freeRTOS����
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

#include "Remote.h"

#include "main.h"

#include "bsp_usart.h"
#include "string.h"
#include "stdlib.h"

// ң����������������
#define RC_CHANNAL_ERROR_VALUE 700

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

// ȡ������
static int16_t RC_abs(int16_t value);
/**
 * @brief          remote control protocol resolution
 * @param[in]      sbus_buf: raw data point
 * @param[out]     rc_ctrl: remote control data struct point
 * @retval         none
 */
/**
 * @brief          ң����Э�����
 * @param[in]      sbus_buf: ԭ������ָ��
 * @param[out]     rc_ctrl: ң��������ָ
 * @retval         none
 */

// remote control data
// ң�������Ʊ���
RC_ctrl_t rc_ctrl;
// ����ԭʼ���ݣ�Ϊ18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��
uint8_t sbus_rx_buf[SBUS_RX_BUF_NUM];

// ���ش������
uint16_t KeyFormerChannal = 0;
uint16_t KeyJumpChannal   = 0;
uint16_t KeyUsed          = 0;

/**
 * @brief          remote control init
 * @param[in]      none
 * @retval         none
 */
/**
 * @brief          ң������ʼ��
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
 * @brief          ��ȡң��������ָ��
 * @param[in]      none
 * @retval         ң��������ָ��
 */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

// �ж�ң���������Ƿ����
uint8_t RC_data_is_error(void)
{
    // ��ֹʹ��go to��䣡����������
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE) {
        memset(&rc_ctrl, 0, sizeof(rc_ctrl));
        return 1;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE) {
        memset(&rc_ctrl, 0, sizeof(rc_ctrl));
        return 1;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE) {
        memset(&rc_ctrl, 0, sizeof(rc_ctrl));
        return 1;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE) {
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

// ȡ������
static int16_t RC_abs(int16_t value)
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
 * @brief          ң����Э�����
 * @param[in]      sbus_buf: ԭ������ָ��
 * @param[out]     rc_ctrl: ң��������ָ
 * @retval         none
 */
void sbus_to_rc(void)
{
    KeyFormerChannal     = rc_ctrl.key.v;
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

    /* ��ֹ΢С�Ŷ� */
    if (rc_ctrl.rc.ch[0] <= 5 && rc_ctrl.rc.ch[0] >= -5) {
        rc_ctrl.rc.ch[1] = 0;
    }
    if (rc_ctrl.rc.ch[2] <= 5 && rc_ctrl.rc.ch[2] >= -5) {
        rc_ctrl.rc.ch[2] = 0;
    }
    if (rc_ctrl.rc.ch[3] <= 5 && rc_ctrl.rc.ch[3] >= -5) {
        rc_ctrl.rc.ch[3] = 0;
    }
    if (rc_ctrl.rc.ch[1] <= 5 && rc_ctrl.rc.ch[1] >= -5) {
        rc_ctrl.rc.ch[1] = 0;
    }

    // ��ֹ�������
    if ((abs(rc_ctrl.rc.ch[1]) > 660) ||
        (abs(rc_ctrl.rc.ch[2]) > 660) ||
        (abs(rc_ctrl.rc.ch[3]) > 660) ||
        (abs(rc_ctrl.rc.ch[0]) > 660)) {
        memset(&rc_ctrl.rc, 0, sizeof(rc_ctrl.rc));
    }

    KeyJumpChannal = (rc_ctrl.key.v ^ KeyFormerChannal);
}

// ����Ӧ�ù���
// ���������Ϊ���������Ĺ������͵㰴�����Ĺ�����
// ���� W��A��S��D��SHIFT��CTRL��F Ϊ���������Ĺ�����
// ��������Ϊ�㰴������

// ������������ļ�ֵ���

bool_t CheakKeyPress(uint16_t Key)
{
    if ((rc_ctrl.key.v & Key) == 0)
        return 0;

    return 1;
}

bool_t CheakKeyPressOnce(uint16_t Key)
{
    if ((rc_ctrl.key.v & Key) == 0) {
        KeyUsed &= (~Key);
        return 0;
    }

    if ((KeyJumpChannal & Key) == 0) {
        return 0;
    } else {
        if ((KeyUsed & Key) == 0) {
            KeyUsed |= Key;
            return 1;
        }
        return 0;
    }
}

// ��һ��ҡ��ֵ
fp32 RemoteChannalRightX(void)
{
    return (rc_ctrl.rc.ch[1] / 660.0f);
}
fp32 RemoteChannalRightY(void)
{
    return -(rc_ctrl.rc.ch[0] / 660.0f);
}
fp32 RemoteChannalLeftX(void)
{
    return (rc_ctrl.rc.ch[3] / 660.0f);
}
fp32 RemoteChannalLeftY(void)
{
    return (rc_ctrl.rc.ch[2] / 660.0f);
}
fp32 RemoteDial()
{
    return (rc_ctrl.rc.ch[4] / 660.0f);
}

// ��һ������ƶ�
fp32 MouseMoveX(void)
{
    return (rc_ctrl.mouse.x / 32768.0f);
}
fp32 MouseMoveY(void)
{
    return (rc_ctrl.mouse.y / 32768.0f);
}

// ������Ҽ�
bool_t MousePressLeft(void)
{
    return rc_ctrl.mouse.press_l;
}
bool_t MousePressRight(void)
{
    return rc_ctrl.mouse.press_r;
}

// ����λ�ü��
bool_t HandleChassisUpside(void)
{ // �������
    return (rc_ctrl.rc.ch[3] > 500);
}
bool_t HandleChassisDownside(void)
{
    return (rc_ctrl.rc.ch[3] < -500);
}
bool_t HandleChassisRightside(void)
{
    return (rc_ctrl.rc.ch[2] > 500);
}
bool_t HandleChassisLeftside(void)
{
    return (rc_ctrl.rc.ch[2] < -500);
}
bool_t HandleGimbalUpside(void)
{ // �Ұ�����
    return (rc_ctrl.rc.ch[1] > 500);
}
bool_t HandleGimbalDownside(void)
{
    return (rc_ctrl.rc.ch[1] < -500);
}
bool_t HandleGimbalRightside(void)
{
    return (rc_ctrl.rc.ch[0] > 500);
}
bool_t HandleGimbalLeftside(void)
{
    return (rc_ctrl.rc.ch[0] < -500);
}
bool_t HandleDial(void)
{
    return (rc_ctrl.rc.ch[4] > 500);
}
/* ��λ */
bool_t HandleChassisResetX(void)
{
    return (rc_ctrl.rc.ch[3] > -50 && rc_ctrl.rc.ch[3]);
}
bool_t HandleChassisResetY(void)
{
    return (rc_ctrl.rc.ch[2] > -50 && rc_ctrl.rc.ch[2]);
}
bool_t HandleGimbalResetX(void)
{
    return (rc_ctrl.rc.ch[1] > -50 && rc_ctrl.rc.ch[1]);
}
bool_t HandleGimbalResetY(void)
{
    return (rc_ctrl.rc.ch[0] > -50 && rc_ctrl.rc.ch[0]);
}
/* �ҵ� */
bool_t SwitchRightUpSide(void)
{
    return (rc_ctrl.rc.s[0] == RC_SW_UP);
}
bool_t SwitchRightMidSide(void)
{
    return (rc_ctrl.rc.s[0] == RC_SW_MID);
}
bool_t SwitchRightDownSide(void)
{
    return (rc_ctrl.rc.s[0] == RC_SW_DOWN);
}
bool_t SwitchLeftUpSide(void)
{
    return (rc_ctrl.rc.s[1] == RC_SW_UP);
}
bool_t SwitchLeftMidSide(void)
{
    return (rc_ctrl.rc.s[1] == RC_SW_MID);
}
bool_t SwitchLeftDownSide(void)
{
    return (rc_ctrl.rc.s[1] == RC_SW_DOWN);
}

int intNormalizedLimit(int input)
{
    if (input > 1) {
        input = 1;
    }
    if (input < -1) {
        input = -1;
    }
    return input;
}

fp32 fp32NormalizedLimit(fp32 input)
{
    if (input > 1.0f) {
        input = 1.0f;
    } else if (input < -1.0f) {
        input = -1.0f;
    }
    return input;
}
