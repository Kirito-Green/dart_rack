#include "bsp_can.h"
#include "main.h"

static uint32_t send_mail_box;
static CAN_TxHeaderTypeDef can_tx_message;

/**
 * @brief 该函数初始化并配置两个 CAN 控制器的 CAN 过滤器。
 */
void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation     = ENABLE;
    can_filter_st.FilterMode           = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale          = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh         = 0x0000;
    can_filter_st.FilterIdLow          = 0x0000;
    can_filter_st.FilterMaskIdHigh     = 0x0000;
    can_filter_st.FilterMaskIdLow      = 0x0000;
    can_filter_st.FilterBank           = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank           = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief 函数 CanSendMessage 使用给定的 CAN 句柄发送具有指定 ID、数据长度代码和消息数据的 CAN 消息。
 *
 * @param hcan 指向 CAN 句柄结构的指针。该结构包含 CAN 外设的配置和状态信息。
 * @param id id 参数是 CAN 报文的标识符。它表示用于识别 CAN 总线上的消息的消息 ID 或地址。
 * @param dlc 参数“dlc”代表“数据长度代码”，它表示消息有效负载中的字节数。它指定将在 CAN 消息中传输的数据的长度。
 * @param message “message”参数是一个指向 uint8_t（无符号 8 位整数）值数组的指针。它代表您想要在 CAN
 * 消息中发送的数据有效负载。数组的长度由“dlc”参数决定，该参数指定数据长度代码
 */
void CanSendMessage(CAN_HandleTypeDef *hcan, uint32_t id, uint32_t dlc, uint8_t *message)
{
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = dlc;

    if (can_tx_message.DLC > 8) {
        can_tx_message.DLC = 8;
    }

    can_tx_message.StdId = id;

    HAL_CAN_AddTxMessage(hcan, &can_tx_message, message, &send_mail_box);
}
