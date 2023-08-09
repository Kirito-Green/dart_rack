#ifndef REFEREE_H
#define REFEREE_H

#include "can_packet.h"
#include "struct_typedef.h"

#define REFEREE_RX_BUF_NUM       80u
#define DART_RACK_RX_START       7u
#define DART_RACK_RX_LENGTH      6u
#define DART_RACK_RX_END         (DART_RACK_RX_START + DART_RACK_RX_LENGTH - 1)
#define DART_RACK_CMD_ID         0x020A
#define DART_RACK_CMD_POS        5u
#define DART_RACK_CMD_LENGTH     2u
#define DART_CLIENT_TARGET_TOWER 0
#define DART_CLIENT_TARGET_BASE  1

extern uint8_t referee_rx_buf[REFEREE_RX_BUF_NUM];

typedef struct
{
    uint8_t dart_cmd_flag;
    uint8_t dart_launch_opening_status;
    uint8_t dart_attack_target; // 0 ǰվ 1
    uint16_t target_change_time;
    uint16_t operate_launch_cmd_time;
} dart_rack_client_cmd_t;

typedef struct
{
    dart_rack_client_cmd_t dart_rack_client_cmd;
} RefereeInformation_t;

extern void referee_init(void);
extern void dart_rack_get_message(void);
extern void dart_client_cmd_update(void);
extern void get_referee_information(RefereeInformation_t *Inf);

#endif
