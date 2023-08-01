#include "InterruptService.h"
#include "main.h"
#include "Motor.h"
#include "Remote.h"
#include "Referee.h"
#include "bsp_can.h"
#include "struct_typedef.h"
#include "CanPacket.h"
#include "Setting.h"
#include <string.h>
#include <stdio.h>

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
// extern ADC_HandleTypeDef hadc1;
// extern ADC_HandleTypeDef hadc3;

CAN_RxHeaderTypeDef rx_header;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    int i = rx_header.StdId - SHOOT_MOTOR1_ID;

    switch (rx_header.StdId) {
        case SHOOT_MOTOR1_ID:
            ShootMotorOfflineCounterUpdate(i);
            MotorProcess(rx_header.StdId, hcan, rx_data);
            break;
        case SHOOT_MOTOR2_ID:
            ShootMotorOfflineCounterUpdate(i);
            MotorProcess(rx_header.StdId, hcan, rx_data);
            break;
        case SHOOT_MOTOR3_ID:
            ShootMotorOfflineCounterUpdate(i);
            MotorProcess(rx_header.StdId, hcan, rx_data);
            break;
        case SHOOT_MOTOR4_ID:
            ShootMotorOfflineCounterUpdate(i);
            MotorProcess(rx_header.StdId, hcan, rx_data);
            break;
        case YAW_MOTOR_ID:
            YawMotorOfflineCounterUpdate();
            MotorProcess(rx_header.StdId, hcan, rx_data);
            break;
        case PITCH_MOTOR_ID:
            PitchMotorOfflineCounterUpdate();
            MotorProcess(rx_header.StdId, hcan, rx_data);
            break;
        case CHAIN_MOTOR_ID:
            ChainMotorOfflineCounterUpdate();
            MotorProcess(rx_header.StdId, hcan, rx_data);
            break;
        case YAW_ANGLE_ENCODER_ID:
            YawAngleEncoderOfflineCounterUpdate();
            MotorProcess(rx_header.StdId, hcan, rx_data);
            break;
        case PITCH_ANGLE_ENCODER_ID:
            PitchAngleEncoderOfflineCounterUpdate();
            MotorProcess(rx_header.StdId, hcan, rx_data);
            break;
        default:
            break;
    }
}

// 串口中断
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
            RemoteOfflineCounterUpdate();
        }
        HAL_UART_Receive_DMA(&huart1, sbus_rx_buf, SBUS_RX_BUF_NUM);
    }
}

/* 裁判系统数据获取 */
int8_t referee_rx_len;
void USART6_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) != RESET) {
        // clear idle flag
        __HAL_UART_CLEAR_IDLEFLAG(&huart6);
        // stop idle transmition
        HAL_UART_DMAStop(&huart6);

        referee_rx_len = REFEREE_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
        RefereeOfflineCounterUpdate();
        DartRackGetMessage();
        HAL_UART_Receive_DMA(&huart6, referee_rx_buf, REFEREE_RX_BUF_NUM);
    }
}

/* 获取系统时间 */
uint32_t SystemTimer = 0;
uint32_t GetSystemTimer(void)
{
    return SystemTimer;
}

void TimerTaskLoop1000Hz(void)
{
    SystemTimer++;
    CommuniteOfflineCounterUpdate();
    CommuniteOfflineStateUpdate();
}

void TIM2_IRQHandler(void)
{
    /* USER CODE BEGIN TIM3_IRQn 0 */
    TimerTaskLoop1000Hz();
    /* USER CODE END TIM3_IRQn 0 */
    HAL_TIM_IRQHandler(&htim2);
    /* USER CODE BEGIN TIM3_IRQn 1 */

    /* USER CODE END TIM3_IRQn 1 */
}

/* ADC中断 */
// uint32_t count_times = 0;
// uint32_t measure_time = 0;
// uint32_t total_adc = 0;
// uint16_t ADC_value;
// uint8_t sampling_times = 0.0f;
// volatile float real_value = 0.0f;
// volatile float last_real_value = 0.0f;
// float voltage_vrefint_proportion = 0.0f;

// bool_t GetLauch(void){
//	return real_value > VOLTAGE_JUDEG_POINT ? 1 : 0;
// }

// float GetADCValue(void){
//	return real_value;
// }

// void adc_init(void){
//	HAL_ADC_Start_IT(&hadc1);
//	HAL_ADC_Start_IT(&hadc3);
// }

// void adc_unable(void){
//	HAL_ADC_Stop_IT(&hadc1);
//	HAL_ADC_Stop_IT(&hadc3);
// }

// fp32 GetLauchSpeed(fp32 length){
//	return measure_time < MIN_MEASURE_TIME? 0 : length / (measure_time * PHOTOELETRICITY_PERIOD);
// }

// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
//	if(hadc->Instance == ADC1){
//		if(sampling_times < 200){
//			sampling_times++;
//			total_adc += HAL_ADC_GetValue(hadc);
//		}
//		else{
//			voltage_vrefint_proportion = (fp32)(1.2f/(total_adc / sampling_times));
//		}
//	}
//	if(hadc->Instance == ADC3){
//		if(sampling_times == 200){
//			ADC_value = HAL_ADC_GetValue(hadc);
//			last_real_value = real_value;
//			real_value = voltage_vrefint_proportion * ADC_value * (22.0f + 200.0f) / (22.0f);
//			/* 记录时间 */
//			if(real_value < VOLTAGE_JUDEG_POINT) {
//				if(last_real_value > VOLTAGE_JUDEG_POINT && count_times > MIN_MEASURE_TIME){
//					measure_time = count_times;
//				}
//				count_times = 0;
//			}
//			else{
//				count_times++;
//			}
//		}
//	}
// }

OfflineCounter_t OfflineCounter;
OfflineMonitor_t OfflineMonitor;

void CommuniteOfflineCounterUpdate(void)
{
    OfflineCounter.YawMotor++;
    OfflineCounter.PitchMotor++;
    OfflineCounter.ChainMotor++;
    for (int i = 0; i < FRICTION_WHEEL_NUM; i++) {
        OfflineCounter.ShootMotor[i]++;
    }
    OfflineCounter.Remote++;
    OfflineCounter.YawAngleEncoder++;
    OfflineCounter.PitchAngleEncoder++;
    OfflineCounter.Referee++;
}

void CommuniteOfflineStateUpdate(void)
{
    // Motor
    if (OfflineCounter.ChainMotor > MOTOR_OFFLINE_TIMEMAX) {
        OfflineMonitor.ChainMotor = 1;
    } else {
        OfflineMonitor.ChainMotor = 0;
    }
    for (int i = 0; i < FRICTION_WHEEL_NUM; i++) {
        if (OfflineCounter.ShootMotor[i] > MOTOR_OFFLINE_TIMEMAX) {
            OfflineMonitor.ShootMotor[i] = 1;
        } else {
            OfflineMonitor.ShootMotor[i] = 0;
        }
    }
    if (OfflineCounter.YawMotor > MOTOR_OFFLINE_TIMEMAX) {
        OfflineMonitor.YawMotor = 1;
    } else {
        OfflineMonitor.YawMotor = 0;
    }
    if (OfflineCounter.PitchMotor > MOTOR_OFFLINE_TIMEMAX) {
        OfflineMonitor.PitchMotor = 1;
    } else {
        OfflineMonitor.PitchMotor = 0;
    }

    // CAN Bus Node
    if (OfflineCounter.YawAngleEncoder > ANGLE_MEASURE_OFFLINE_TIMEMAX) {
        OfflineMonitor.YawAngleEncoder = 1;
    } else {
        OfflineMonitor.YawAngleEncoder = 0;
    }
    if (OfflineCounter.PitchAngleEncoder > ANGLE_MEASURE_OFFLINE_TIMEMAX) {
        OfflineMonitor.PitchAngleEncoder = 1;
    } else {
        OfflineMonitor.PitchAngleEncoder = 0;
    }

    // Remote
    if (OfflineCounter.Remote > MOTOR_OFFLINE_TIMEMAX) {
        OfflineMonitor.Remote = 1;
    } else {
        OfflineMonitor.Remote = 0;
    }

    // Referee
    if (OfflineCounter.Referee > REFEREE_OFFLINE_TIMEMAX) {
        OfflineMonitor.Referee = 1;
    } else {
        OfflineMonitor.Referee = 0;
    }
}

void DeviceOfflineMonitorUpdate(OfflineMonitor_t *Monitor)
{
    memcpy(Monitor, &OfflineMonitor, sizeof(OfflineMonitor_t));
}

void YawMotorOfflineCounterUpdate(void)
{
    OfflineCounter.YawMotor = 0;
}

void PitchMotorOfflineCounterUpdate(void)
{
    OfflineCounter.PitchMotor = 0;
}

void ChainMotorOfflineCounterUpdate(void)
{
    OfflineCounter.ChainMotor = 0;
}

void ShootMotorOfflineCounterUpdate(int id)
{
    OfflineCounter.ShootMotor[id] = 0;
}

void YawAngleEncoderOfflineCounterUpdate(void)
{
    OfflineCounter.YawAngleEncoder = 0;
}

void PitchAngleEncoderOfflineCounterUpdate(void)
{
    OfflineCounter.PitchAngleEncoder = 0;
}

void RemoteOfflineCounterUpdate(void)
{
    OfflineCounter.Remote = 0;
}

void RefereeOfflineCounterUpdate(void)
{
    OfflineCounter.Referee = 0;
}
