#ifndef INTERUPT_SERVICE_H
#define INTERUPT_SERVICE_H
#include "struct_typedef.h"
#include "DartRAckKeyMap.h"
#define DEVICE_OFFLINE          0x01
#define DEVICE_ONLINE           0x00


typedef struct
{
    // Motor
		uint32_t ChainMotor;
		uint32_t ShootMotor[FRICTION_WHEEL_NUM];
		uint32_t PitchMotor;
    uint32_t YawMotor;
    // CAN Bus Node
		uint32_t YawAngleEncoder;
		uint32_t PitchAngleEncoder;
		uint32_t Referee;
    // Remote
    uint32_t Remote;
} OfflineCounter_t;


typedef struct
{
    // Motor
		uint8_t ChainMotor;   //���ʹ����
    uint8_t ShootMotor[FRICTION_WHEEL_NUM];   //Ħ���ֵ��
		uint8_t YawMotor;
    uint8_t PitchMotor;
    // CAN Bus Node
		uint8_t YawAngleEncoder;
		uint8_t PitchAngleEncoder;
		uint8_t Referee;
    // Remote
    uint8_t Remote;//ң����
} OfflineMonitor_t;


extern void YawMotorOfflineCounterUpdate(void);
extern void PitchMotorOfflineCounterUpdate(void);
extern void ChainMotorOfflineCounterUpdate(void);
extern void ShootMotorOfflineCounterUpdate(int id);
extern void RemoteOfflineCounterUpdate(void);
extern void YawAngleEncoderOfflineCounterUpdate(void);
extern void PitchAngleEncoderOfflineCounterUpdate(void);
extern void RefereeOfflineCounterUpdate(void);

extern void DeviceOfflineMonitorUpdate(OfflineMonitor_t *Monitor);
extern uint32_t GetSystemTimer(void);
extern void TimerTaskLoop1000Hz(void);

extern void adc_init(void);
extern void adc_unable(void);
extern fp32 GetADCValue(void);
extern bool_t GetLauch(void);
extern fp32 GetLauchSpeed(fp32 length);

extern void CommuniteOfflineCounterUpdate(void);
extern void CommuniteOfflineStateUpdate(void);

#endif
