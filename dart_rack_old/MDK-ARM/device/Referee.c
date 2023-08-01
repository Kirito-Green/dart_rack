#include "Referee.h"
#include "bsp_usart.h"
#include "string.h"

uint8_t referee_rx_buf[REFEREE_RX_BUF_NUM];
RefereeInformation_t RefereeInformation;

void RefereeInit(void)
{
    usart6_rx_dma_init(referee_rx_buf, REFEREE_RX_BUF_NUM);
}

uint16_t cmd_id;
void DartRackGetMessage(void)
{
    // cmd_id = (uint16_t)(referee_rx_buf[5] << 8 | referee_rx_buf[6]);
    memcpy(&cmd_id, &referee_rx_buf[5], 2);
    switch (cmd_id) {
        case DART_RACK_CMD_ID:
            DartClientCmdUpdate();
            break;
        default:
            break;
    }
}

void DartClientCmdUpdate(void)
{
    //	RefereeInformation.DartClientCmd.DartLaunchOpeningStatus = (uint8_t)referee_rx_buf[7];
    //	RefereeInformation.DartClientCmd.DartAttackTarget = (uint8_t)referee_rx_buf[8];
    //	RefereeInformation.DartClientCmd.TargetChangeTime = (uint16_t)(referee_rx_buf[9] << 8 | referee_rx_buf[10]);
    //	RefereeInformation.DartClientCmd.OperateLaunchCmdTime = (uint16_t)(referee_rx_buf[11] << 8 | referee_rx_buf[12]);
    memcpy(&RefereeInformation.DartClientCmd.DartLaunchOpeningStatus, &referee_rx_buf[7], 1);
    memcpy(&RefereeInformation.DartClientCmd.DartAttackTarget, &referee_rx_buf[8], 1);
    memcpy(&RefereeInformation.DartClientCmd.TargetChangeTime, &referee_rx_buf[9], 2);
    memcpy(&RefereeInformation.DartClientCmd.OperateLaunchCmdTime, &referee_rx_buf[11], 2);
}

void GetRefereeInformation(RefereeInformation_t *Inf)
{
    memcpy(Inf, &RefereeInformation, sizeof(RefereeInformation_t));
}
