#ifndef REFEREE_H
#define REFEREE_H

#include "CanPacket.h"


#define REFEREE_RX_BUF_NUM               80u
#define DART_RACK_RX_START               7u								 
#define DART_RACK_RX_LENGTH              6u
#define DART_RACK_RX_END								 (DART_RACK_RX_START+DART_RACK_RX_LENGTH-1)
#define DART_RACK_CMD_ID                 0x020A
#define DART_CLIENT_TARGET_TOWER                0
#define DART_CLIENT_TARGET_BASE                 1

extern uint8_t referee_rx_buf[REFEREE_RX_BUF_NUM];


typedef struct 
{ 
  uint8_t DartLaunchOpeningStatus; 
  uint8_t DartAttackTarget;              // 0 Ç°ÉÚÕ¾ 1 »ùµØ
  uint16_t TargetChangeTime; 
  uint16_t OperateLaunchCmdTime; 
} DartClientCmd_t; 

typedef struct
{
	DartClientCmd_t DartClientCmd;
} RefereeInformation_t;


extern void RefereeInit(void);
extern void DartRackGetMessage(void);
extern void DartClientCmdUpdate(void);
extern void GetRefereeInformation(RefereeInformation_t* Inf);


#endif

