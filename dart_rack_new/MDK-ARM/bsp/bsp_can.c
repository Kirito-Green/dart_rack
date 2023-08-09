#include "bsp_can.h"
#include "main.h"

static uint32_t send_mail_box;
static CAN_TxHeaderTypeDef can_tx_message;

/**
 * @brief �ú�����ʼ������������ CAN �������� CAN ��������
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
 * @brief ���� CanSendMessage ʹ�ø����� CAN ������;���ָ�� ID�����ݳ��ȴ������Ϣ���ݵ� CAN ��Ϣ��
 *
 * @param hcan ָ�� CAN ����ṹ��ָ�롣�ýṹ���� CAN ��������ú�״̬��Ϣ��
 * @param id id ������ CAN ���ĵı�ʶ��������ʾ����ʶ�� CAN �����ϵ���Ϣ����Ϣ ID ���ַ��
 * @param dlc ������dlc���������ݳ��ȴ��롱������ʾ��Ϣ��Ч�����е��ֽ�������ָ������ CAN ��Ϣ�д�������ݵĳ��ȡ�
 * @param message ��message��������һ��ָ�� uint8_t���޷��� 8 λ������ֵ�����ָ�롣����������Ҫ�� CAN
 * ��Ϣ�з��͵�������Ч���ء�����ĳ����ɡ�dlc�������������ò���ָ�����ݳ��ȴ���
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
