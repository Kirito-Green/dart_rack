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
 * @brief  �ú���ͨ�����ж�ģʽ��������ʱ��2����ʼ��ϵͳʱ�䡣
 */
void systime_init(void)
{
    HAL_TIM_Base_Start_IT(&htim2);
}

/**
 * @brief  ������HAL_CAN_RxFifo0MsgPendingCallback������ ID ������� CAN ��Ϣ���������ʵ��ĵ����������
 *
 * @param hcan ������hcan����ָ�� CAN_HandleTypeDef �ṹ��ָ�룬�ýṹ���� CAN ��������ú�״̬��Ϣ�������ڷ��� CAN ���貢�������յ�����Ϣ��
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
 * @brief �ú������� USART1 ���жϲ������б�־�Ƿ����ã�Ȼ������ñ�־��ֹͣ���д��䣬�������Ϊ 18��������յ������ݡ�
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
 * @brief �ú������� USART6 ���жϲ��ڿ��б�־��λʱִ�в�����
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
 * @brief ������get_system_time���� 32 λ�޷���������ʽ����ϵͳʱ�䡣
 *
 * @return uint32_t ���͵�ֵ������һ���޷��� 32 λ������
 */
uint32_t get_system_time(void)
{
    return SystemTimer;
}

/**
 * @brief �ú�������ϵͳ��ʱ��������ͨ�����߼�������״̬��
 */
void timer_task_loop_1000Hz(void)
{
    SystemTimer++;
    CommuniteOfflineCounterUpdate();
    CommuniteOfflineStateUpdate();
}

/**
 * @brief TIM2_IRQHandler ��������ʱ�� 2 ���жϣ����ڽ�����Ȩ���ݸ� HAL_TIM_IRQHandler ����֮ǰ����timer_task_loop_1000Hz ������
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
 * @brief �ù��ܸ���������ͨ����صĸ��ּ�������
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
 * @brief ��CommuniteOfflineStateUpdate���������µ������������ң�����Ͳ������ȸ�����������߼��״̬��
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
 * @brief ������EXTI2_IRQHandler����һ���жϴ�������������õ���ľ��룬Ȼ����� HAL_GPIO_EXTI_IRQHandler ������
 */
void EXTI2_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI2_IRQn 0 */
    // printf("EXTI2_IRQHandler\r\n");
    motor_reset_dist_ecd();
    /* USER CODE END EXTI2_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(KEY_Pin);
    /* USER CODE BEGIN EXTI2_IRQn 1 */

    /* USER CODE END EXTI2_IRQn 1 */
}

/**
 * @brief ����device_offline_monitor_update ʹ��offline_monitor �ṹ�е�ֵ����OfflineMonitor_t �ṹ��
 *
 * @param Monitor ָ�� OfflineMonitor_t �ṹ��ָ�룬�ýṹ��ʹ��offline_monitor �ṹ�е�ֵ���и��¡�
 */
void device_offline_monitor_update(OfflineMonitor_t *Monitor)
{
    memcpy(Monitor, &offline_monitor, sizeof(OfflineMonitor_t));
}

/**
 * @brief �ú�������ƫ����������߼�������
 */
void yaw_motor_offline_counter_update(void)
{
    offline_counter.yaw_motor = 0;
}

/**
 * @brief �ú������¸�����������߼�������
 */
void pitch_motor_offline_counter_update(void)
{
    offline_counter.pitch_motor = 0;
}

/**
 * @brief �ú����������϶���������߼�������
 */
void drag_left_motor_offline_counter_update(void)
{
    offline_counter.drag_left_motor = 0;
}

/**
 * @brief �ú����������϶���������߼�������
 */
void drag_right_motor_offline_counter_update(void)
{
    offline_counter.drag_right_motor = 0;
}

/**
 * @brief ������load_motor_offline_counter_update�������ƽ���������߼�������
 */
void load_motor_offline_counter_update(void)
{
    offline_counter.load_motor = 0;
}

/**
 * @brief ������adjust_motor_offline_counter_update����������������߼���������Ϊ�㡣
 */
void adjust_motor_offline_counter_update(void)
{
    offline_counter.adjust_motor = 0;
}

/**
 * @brief ������yaw_angle_encoder_offline_counter_update������ƫ���Ǳ����������߼�������
 */
void yaw_angle_encoder_offline_counter_update(void)
{
    offline_counter.yaw_angle_encoder = 0;
}

/**
 * @brief �ú���ͨ���������Ǳ����������߼���������Ϊ0�����¸ü�������
 */
void pitch_angle_encoder_offline_counter_update(void)
{
    offline_counter.pitch_angle_encoder = 0;
}

/**
 * ������remote_offline_counter_update������ң���������߼�����������Ϊ0��
 */
void remote_offline_counter_update(void)
{
    offline_counter.remote = 0;
}

/**
 * @brief ������referee_offline_counter_update"��������ϵͳ�����߼�����������Ϊ0��
 */
void referee_offline_counter_update(void)
{
    offline_counter.referee = 0;
}
