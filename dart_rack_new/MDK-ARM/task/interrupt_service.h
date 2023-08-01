#ifndef INTERUPT_SERVICE_H
#define INTERUPT_SERVICE_H
#include "struct_typedef.h"
#include "dart_rack_key_map.h"
#define DEVICE_OFFLINE 0x01
#define DEVICE_ONLINE  0x00

typedef struct
{
    // Motor
    uint32_t drag_left_motor;
    uint32_t drag_right_motor;
    uint32_t load_motor;
    uint32_t adjust_motor;
    uint32_t pitch_motor;
    uint32_t yaw_motor;
    // CAN Bus Node
    uint32_t yaw_angle_encoder;
    uint32_t pitch_angle_encoder;
    uint32_t referee;
    // remote
    uint32_t remote;
} OfflineCounter_t;

typedef struct
{
    // Motor
    uint8_t drag_left_motor;
    uint8_t drag_right_motor;
    uint8_t load_motor;
    uint8_t adjust_motor;
    uint8_t yaw_motor;
    uint8_t pitch_motor;
    // CAN Bus Node
    uint8_t yaw_angle_encoder;
    uint8_t pitch_angle_encoder;
    uint8_t referee;
    // remote
    uint8_t remote; //
} OfflineMonitor_t;

extern void systime_init(void);

extern void yaw_motor_offline_counter_update(void);
extern void pitch_motor_offline_counter_update(void);
extern void drag_left_motor_offline_counter_update(void);
extern void drag_right_motor_offline_counter_update(void);
extern void load_motor_offline_counter_update(void);
extern void adjust_motor_offline_counter_update(void);
extern void remote_offline_counter_update(void);
extern void yaw_angle_encoder_offline_counter_update(void);
extern void pitch_angle_encoder_offline_counter_update(void);
extern void referee_offline_counter_update(void);

extern void device_offline_monitor_update(OfflineMonitor_t *Monitor);
extern uint32_t get_system_time(void);
extern void timer_task_loop_1000Hz(void);

extern void CommuniteOfflineCounterUpdate(void);
extern void CommuniteOfflineStateUpdate(void);

#endif
