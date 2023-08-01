#include "referee.h"
#include "bsp_usart.h"
#include "string.h"

uint8_t referee_rx_buf[REFEREE_RX_BUF_NUM];
RefereeInformation_t referee_information;

/**
 * @brief 函数“referee_init”使用缓冲区和缓冲区大小初始化 USART6 接收 DMA。
 */
void referee_init(void)
{
    usart6_rx_dma_init(referee_rx_buf, REFEREE_RX_BUF_NUM);
}

uint16_t cmd_id;
/**
 * @brief 函数“dart_rack_get_message”检查从缓冲区接收到的命令 ID，并根据命令 ID 调用特定函数。
 * 可用于接收客户端飞镖架控制指令
 */
void dart_rack_get_message(void)
{
    // cmd_id = (uint16_t)(referee_rx_buf[5] << 8 | referee_rx_buf[6]);
    memcpy(&cmd_id, &referee_rx_buf[5], 2);
    switch (cmd_id) {
        case DART_RACK_CMD_ID:
            dart_client_cmd_update();
            break;
        default:
            break;
    }
}

/**
 * @brief 函数“dart_client_cmd_update”用于更新客户端指令。
 */
void dart_client_cmd_update(void)
{
    referee_information.dart_rack_client_cmd.dart_cmd_flag = 1;
    //	referee_information.dart_rack_client_cmd.dart_launch_opening_status = (uint8_t)referee_rx_buf[7];
    //	referee_information.dart_rack_client_cmd.dart_attack_target = (uint8_t)referee_rx_buf[8];
    //	referee_information.dart_rack_client_cmd.target_change_time = (uint16_t)(referee_rx_buf[9] << 8 | referee_rx_buf[10]);
    //	referee_information.dart_rack_client_cmd.operate_launch_cmd_time = (uint16_t)(referee_rx_buf[11] << 8 | referee_rx_buf[12]);
    memcpy(&referee_information.dart_rack_client_cmd.dart_launch_opening_status, &referee_rx_buf[7], 1);
    memcpy(&referee_information.dart_rack_client_cmd.dart_attack_target, &referee_rx_buf[8], 1);
    memcpy(&referee_information.dart_rack_client_cmd.target_change_time, &referee_rx_buf[9], 2);
    memcpy(&referee_information.dart_rack_client_cmd.operate_launch_cmd_time, &referee_rx_buf[11], 2);
}

/**
 * @brief 函数“get_referee_information”将“referee_information”结构的内容复制到“Inf”结构中。
 *
 * @param Inf 指向 RefereeInformation_t 类型的结构的指针。
 */
void get_referee_information(RefereeInformation_t *Inf)
{
    memcpy(Inf, &referee_information, sizeof(RefereeInformation_t));
}
