#include "interrupt_service.h"
#include "main.h"
#include "motor.h"
#include "remote.h"
#include "referee.h"
#include "bsp_can.h"
#include "struct_typedef.h"
#include "can_packet.h"
#include "setting.h"
#include <string.h>
#include <stdio.h>
#include "launch_thread.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart8;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/**
 * @brief  该函数通过在中断模式下启动定时器2来初始化系统时间。
 */
void systime_init(void)
{
    HAL_TIM_Base_Start_IT(&htim2);
}

/**
 * @brief  函数“HAL_CAN_RxFifo0MsgPendingCallback”根据 ID 处理传入的 CAN 消息，并调用适当的电机处理函数。
 *
 * @param hcan 参数“hcan”是指向 CAN_HandleTypeDef 结构的指针，该结构包含 CAN 外设的配置和状态信息。它用于访问 CAN 外设并检索接收到的消息。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId) {
        case DRAG_LEFT_MOTOR_ID:
            drag_left_motor_offline_counter_update();
            motor_process(rx_header.StdId, hcan, rx_data);
            break;
        case DRAG_RIGHT_MOTOR_ID:
            drag_right_motor_offline_counter_update();
            motor_process(rx_header.StdId, hcan, rx_data);
            break;
        case LOAD_MOTOR_ID:
            load_motor_offline_counter_update();
            motor_process(rx_header.StdId, hcan, rx_data);
            break;
        case ADJUST_MOTOR_ID:
            adjust_motor_offline_counter_update();
            motor_process(rx_header.StdId, hcan, rx_data);
            break;
        case YAW_MOTOR_ID:
            yaw_motor_offline_counter_update();
            motor_process(rx_header.StdId, hcan, rx_data);
            break;
        case PITCH_MOTOR_ID:
            pitch_motor_offline_counter_update();
            motor_process(rx_header.StdId, hcan, rx_data);
            break;
        case YAW_ANGLE_ENCODER_ID:
            yaw_angle_encoder_offline_counter_update();
            motor_process(rx_header.StdId, hcan, rx_data);
            break;
        case PITCH_ANGLE_ENCODER_ID:
            pitch_angle_encoder_offline_counter_update();
            motor_process(rx_header.StdId, hcan, rx_data);
            break;
        default:
            break;
    }
}

/**
 * @brief 该函数处理 USART1 的中断并检查空闲标志是否设置，然后清除该标志，停止空闲传输，如果长度为 18，则处理接收到的数据。
 * 用于处理遥控器发送数据，更新遥控器离线计数器。
 */
int8_t rx_len;
void USART1_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET) {
        // clear idle flag
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        // stop idle transmition
        HAL_UART_DMAStop(&huart1);

        rx_len = SBUS_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
        if (rx_len == 18) {
            sbus_to_rc();
            remote_offline_counter_update();
        }
        HAL_UART_Receive_DMA(&huart1, sbus_rx_buf, SBUS_RX_BUF_NUM);
    }
}

/**
 * @brief 该函数处理 USART6 的中断并在空闲标志置位时执行操作。
 * 读裁判系统数据，更新裁判系统离线计数器。
 */
int8_t referee_rx_len;
void USART6_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) != RESET) {
        // clear idle flag
        __HAL_UART_CLEAR_IDLEFLAG(&huart6);
        // stop idle transmition
        HAL_UART_DMAStop(&huart6);

        referee_rx_len = REFEREE_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
        referee_offline_counter_update();
        dart_rack_get_message();
        HAL_UART_Receive_DMA(&huart6, referee_rx_buf, REFEREE_RX_BUF_NUM);
    }
}

uint32_t SystemTimer = 0;
/**
 * @brief 函数“get_system_time”以 32 位无符号整数形式返回系统时间。
 *
 * @return uint32_t 类型的值，它是一个无符号 32 位整数。
 */
uint32_t get_system_time(void)
{
    return SystemTimer;
}

/**
 * @brief 该函数增加系统计时器并更新通信离线计数器和状态。
 */
void timer_task_loop_1000Hz(void)
{
    SystemTimer++;
    CommuniteOfflineCounterUpdate();
    CommuniteOfflineStateUpdate();
}

/**
 * @brief TIM2_IRQHandler 函数处理定时器 2 的中断，并在将控制权传递给 HAL_TIM_IRQHandler 函数之前调用timer_task_loop_1000Hz 函数。
 */
void TIM2_IRQHandler(void)
{
    /* USER CODE BEGIN TIM3_IRQn 0 */
    timer_task_loop_1000Hz();
    /* USER CODE END TIM3_IRQn 0 */
    HAL_TIM_IRQHandler(&htim2);
    /* USER CODE BEGIN TIM3_IRQn 1 */

    /* USER CODE END TIM3_IRQn 1 */
}

OfflineCounter_t offline_counter;
OfflineMonitor_t offline_monitor;

/**
 * @brief 该功能更新与离线通信相关的各种计数器。
 */
void CommuniteOfflineCounterUpdate(void)
{
    offline_counter.yaw_motor++;
    offline_counter.pitch_motor++;
    offline_counter.drag_left_motor++;
    offline_counter.drag_right_motor++;
    offline_counter.load_motor++;
    offline_counter.adjust_motor++;
    offline_counter.remote++;
    offline_counter.yaw_angle_encoder++;
    offline_counter.pitch_angle_encoder++;
    offline_counter.referee++;
}

/**
 * @brief “CommuniteOfflineStateUpdate”函数更新电机、编码器、遥控器和裁判器等各种组件的离线监控状态。
 */
void CommuniteOfflineStateUpdate(void)
{
    // Motor
    if (offline_counter.drag_left_motor > MOTOR_OFFLINE_TIMEMAX) {
        offline_monitor.drag_left_motor = 1;
    } else {
        offline_monitor.drag_left_motor = 0;
    }
    if (offline_counter.drag_right_motor > MOTOR_OFFLINE_TIMEMAX) {
        offline_monitor.drag_right_motor = 1;
    } else {
        offline_monitor.drag_right_motor = 0;
    }
    if (offline_counter.load_motor > MOTOR_OFFLINE_TIMEMAX) {
        offline_monitor.load_motor = 1;
    } else {
        offline_monitor.load_motor = 0;
    }
    if (offline_counter.adjust_motor > MOTOR_OFFLINE_TIMEMAX) {
        offline_monitor.adjust_motor = 1;
    } else {
        offline_monitor.adjust_motor = 0;
    }
    if (offline_counter.yaw_motor > MOTOR_OFFLINE_TIMEMAX) {
        offline_monitor.yaw_motor = 1;
    } else {
        offline_monitor.yaw_motor = 0;
    }
    if (offline_counter.pitch_motor > MOTOR_OFFLINE_TIMEMAX) {
        offline_monitor.pitch_motor = 1;
    } else {
        offline_monitor.pitch_motor = 0;
    }

    // CAN Bus Node
    if (offline_counter.yaw_angle_encoder > ANGLE_MEASURE_OFFLINE_TIMEMAX) {
        offline_monitor.yaw_angle_encoder = 1;
    } else {
        offline_monitor.yaw_angle_encoder = 0;
    }
    if (offline_counter.pitch_angle_encoder > ANGLE_MEASURE_OFFLINE_TIMEMAX) {
        offline_monitor.pitch_angle_encoder = 1;
    } else {
        offline_monitor.pitch_angle_encoder = 0;
    }

    // remote
    if (offline_counter.remote > MOTOR_OFFLINE_TIMEMAX) {
        offline_monitor.remote = 1;
    } else {
        offline_monitor.remote = 0;
    }

    // referee
    if (offline_counter.referee > REFEREE_OFFLINE_TIMEMAX) {
        offline_monitor.referee = 1;
    } else {
        offline_monitor.referee = 0;
    }
}

/**
 * @brief 函数“EXTI2_IRQHandler”是一个中断处理程序，用于重置电机的距离和目标，然后调用 HAL_GPIO_EXTI_IRQHandler 函数。
 */
void EXTI2_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI2_IRQn 0 */
    // printf("EXTI2_IRQHandler\r\n");
    motor_reset_dist_ecd();
    motor_reset_target();
    /* USER CODE END EXTI2_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(KEY_Pin);
    /* USER CODE BEGIN EXTI2_IRQn 1 */

    /* USER CODE END EXTI2_IRQn 1 */
}

/**
 * @brief 函数device_offline_monitor_update 使用offline_monitor 结构中的值更新OfflineMonitor_t 结构。
 *
 * @param Monitor 指向 OfflineMonitor_t 结构的指针，该结构将使用offline_monitor 结构中的值进行更新。
 */
void device_offline_monitor_update(OfflineMonitor_t *Monitor)
{
    memcpy(Monitor, &offline_monitor, sizeof(OfflineMonitor_t));
}

/**
 * @brief 该函数更新偏航电机的离线计数器。
 */
void yaw_motor_offline_counter_update(void)
{
    offline_counter.yaw_motor = 0;
}

/**
 * @brief 该函数更新俯仰电机的离线计数器。
 */
void pitch_motor_offline_counter_update(void)
{
    offline_counter.pitch_motor = 0;
}

/**
 * @brief 该函数更新左拖动电机的离线计数器。
 */
void drag_left_motor_offline_counter_update(void)
{
    offline_counter.drag_left_motor = 0;
}

/**
 * @brief 该函数更新右拖动电机的离线计数器。
 */
void drag_right_motor_offline_counter_update(void)
{
    offline_counter.drag_right_motor = 0;
}

/**
 * @brief 函数“load_motor_offline_counter_update”更新推进电机的离线计数器。
 */
void load_motor_offline_counter_update(void)
{
    offline_counter.load_motor = 0;
}

/**
 * @brief 函数“adjust_motor_offline_counter_update”将调整电机的离线计数器更新为零。
 */
void adjust_motor_offline_counter_update(void)
{
    offline_counter.adjust_motor = 0;
}

/**
 * @brief 函数“yaw_angle_encoder_offline_counter_update”重置偏航角编码器的离线计数器。
 */
void yaw_angle_encoder_offline_counter_update(void)
{
    offline_counter.yaw_angle_encoder = 0;
}

/**
 * @brief 该函数通过将俯仰角编码器的离线计数器设置为0来更新该计数器。
 */
void pitch_angle_encoder_offline_counter_update(void)
{
    offline_counter.pitch_angle_encoder = 0;
}

/**
 * 函数“remote_offline_counter_update”将“遥控器的离线计数器”设置为0。
 */
void remote_offline_counter_update(void)
{
    offline_counter.remote = 0;
}

/**
 * @brief 函数“referee_offline_counter_update"将“裁判系统的离线计数器”设置为0。
 */
void referee_offline_counter_update(void)
{
    offline_counter.referee = 0;
}
