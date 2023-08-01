#include "LauchThread.h"
#include "pid.h"
#include "ADRC.h"
#include "EFC.h"
#include "Combination.h"
#include "motor.h"
#include "cmsis_os.h"
#include "DartRackParameter.h"
#include "Setting.h"
#include "remote.h"
#include "Referee.h"
#include "InterruptService.h"
#include "string.h"
#include "User_lib.h"

DartRack_t DartRack;          // ���ڼ�״̬
RC_ctrl_t Remote;             // ң����״̬
OfflineMonitor_t Offline;     // ģ������״̬
RefereeInformation_t Referee; // ����ϵͳ����
extern TIM_HandleTypeDef htim3;

/* ���� */
int8_t DART_RACK_CLIENT_CMD;
int8_t DART_RACK_AIM_SHOOT;
bool_t DART_RACK_FINISH_FLAG;
int8_t DART_RACK_NUM_COUNT;
/* ��̨λ�� */
fp32 YAW_CMD_TEMP;
fp32 YAW_TARGET_ANGLE;
fp32 YAW_TOWER_ANGLE;
fp32 YAW_BASE_ANGLE;
/* Ħ���ֲ��� */
fp32 FRICTION_CMD_TEMP;
bool_t LAUCH_CMD_FLAG;
bool_t FRICTION_COARSE_INIT_FLAG;
bool_t FRICTION_COARSE_TUNE_FLAG;
/* ʱ���¼ */
uint32_t DART_RACK_RESET_TIME_COUNT;
uint32_t DART_RACK_LAUCH_TIME_COUNT;
uint32_t DART_RACK_SHELTER_TIME_COUNT;
uint32_t DART_RACK_CHAIN_START_TIME;
uint32_t RACK_LAUCH_TIME[4] = {RACK1_LAUCH_TIME, RACK2_LAUCH_TIME, RACK3_LAUCH_TIME, RACK4_LAUCH_TIME};
/* �ٶȼ�¼ */
fp32 CHAIN_MATCH_SPEED;
fp32 SHOOT_MATCH_FIRST_SPEED;
fp32 SHOOT_MATCH_SECOND_SPEED;

void LauchThread(void const *argument)
{
    DartRackInit();

    for (;;) {
        Remote = *get_remote_control_point(); // ��ȡң��������
        DeviceOfflineMonitorUpdate(&Offline); // ��ȡģ����������
        GetRefereeInformation(&Referee);      // ��ȡ����ϵͳ����

        DartRackStateMachineUpdate(); // ���ڼ�״̬ת��
        MotorMeasureUpdate();         // ������ݸ���
        EncoderMeasureUpdate();       // ����������
        MotorAlgorithmUpdate();       // ���PID����
        GimbalCommandUpdate();        // ��ָ̨�����
        ShootCommandUpdate();         // ����ָ�����
        ShootDiffCompensate();        // ͬ��Ħ�����ٶȲ���
        DartRackShoot();              // ���ڷ���
        // DartLauchSpeedUpdate();                        // ���ڷ����ٶȸ���

        osDelay(1);
    }
}

void DartRackInit(void)
{
    /* �ٶȲ��� */
    CHAIN_MATCH_SPEED = 500.0f;
    YAW_TOWER_ANGLE   = (YAW_ZERO_ANGLE + 6.5f);
    YAW_BASE_ANGLE    = (YAW_ZERO_ANGLE - 7.3f);
    /* Test */
    // DartRack.Target.Straight.FirstSpeed[0]  = 500.0f;
    // DartRack.Target.Straight.SecondSpeed[0] = 800.0f;
    // DartRack.Target.Straight.FirstSpeed[1]  = 500.0f;
    // DartRack.Target.Straight.SecondSpeed[1] = 800.0f;
    // DartRack.Target.Straight.FirstSpeed[2]  = 500.0f;
    // DartRack.Target.Straight.SecondSpeed[2] = 800.0f;
    // DartRack.Target.Straight.FirstSpeed[3]  = 500.0f;
    // DartRack.Target.Straight.SecondSpeed[3] = 800.0f;

    // DartRack.Target.Tower.FirstSpeed[0]  = 500.0f;
    // DartRack.Target.Tower.SecondSpeed[0] = 900.0f;
    // DartRack.Target.Tower.FirstSpeed[1]  = 500.0f;
    // DartRack.Target.Tower.SecondSpeed[1] = 900.0f;
    // DartRack.Target.Tower.FirstSpeed[2]  = 500.0f;
    // DartRack.Target.Tower.SecondSpeed[2] = 900.0f;
    // DartRack.Target.Tower.FirstSpeed[3]  = 500.0f;
    // DartRack.Target.Tower.SecondSpeed[3] = 900.0f;

    // DartRack.Target.Base.FirstSpeed[0]  = 336.0f;
    // DartRack.Target.Base.SecondSpeed[0] = 416.0f;
    // DartRack.Target.Base.FirstSpeed[1]  = 336.0f;
    // DartRack.Target.Base.SecondSpeed[1] = 416.0f;
    // DartRack.Target.Base.FirstSpeed[2]  = 336.0f;
    // DartRack.Target.Base.SecondSpeed[2] = 416.0f;
    // DartRack.Target.Base.FirstSpeed[3]  = 336.0f;
    // DartRack.Target.Base.SecondSpeed[3] = 416.0f;

    /* ���������� */
    /* ֱ�߲��� */
    DartRack.Target.Straight.FirstSpeed[0]  = 2910.0f;
    DartRack.Target.Straight.SecondSpeed[0] = 3710.0f;
    DartRack.Target.Straight.FirstSpeed[1]  = 2910.0f;
    DartRack.Target.Straight.SecondSpeed[1] = 3710.0f;
    DartRack.Target.Straight.FirstSpeed[2]  = 2910.0f;
    DartRack.Target.Straight.SecondSpeed[2] = 3710.0f;
    DartRack.Target.Straight.FirstSpeed[3]  = 2910.0f;
    DartRack.Target.Straight.SecondSpeed[3] = 3710.0f;
    /* ���������� */
    DartRack.Target.Tower.FirstSpeed[0]  = 2910.0f;
    DartRack.Target.Tower.SecondSpeed[0] = 3710.0f;
    DartRack.Target.Tower.FirstSpeed[1]  = 2910.0f;
    DartRack.Target.Tower.SecondSpeed[1] = 3710.0f;
    DartRack.Target.Tower.FirstSpeed[2]  = 2910.0f;
    DartRack.Target.Tower.SecondSpeed[2] = 3710.0f;
    DartRack.Target.Tower.FirstSpeed[3]  = 2910.0f;
    DartRack.Target.Tower.SecondSpeed[3] = 3710.0f;
    /* ���ز��� */
    DartRack.Target.Base.FirstSpeed[0]  = 3500.0f;
    DartRack.Target.Base.SecondSpeed[0] = 4500.0f;
    DartRack.Target.Base.FirstSpeed[1]  = 3500.0f;
    DartRack.Target.Base.SecondSpeed[1] = 4500.0f;
    DartRack.Target.Base.FirstSpeed[2]  = 3500.0f;
    DartRack.Target.Base.SecondSpeed[2] = 4500.0f;
    DartRack.Target.Base.FirstSpeed[3]  = 3500.0f;
    DartRack.Target.Base.SecondSpeed[3] = 4500.0f;

    /* �ڶ������׷���*/
    /* ֱ�߲��� */
    DartRack.Target.Straight.FirstSpeed[0]  = 2900.0f;
    DartRack.Target.Straight.SecondSpeed[0] = 3700.0f;
    DartRack.Target.Straight.FirstSpeed[1]  = 2900.0f;
    DartRack.Target.Straight.SecondSpeed[1] = 3700.0f;
    DartRack.Target.Straight.FirstSpeed[2]  = 2900.0f;
    DartRack.Target.Straight.SecondSpeed[2] = 3700.0f;
    DartRack.Target.Straight.FirstSpeed[3]  = 2900.0f;
    DartRack.Target.Straight.SecondSpeed[3] = 3700.0f;
    /* ���������� */
    DartRack.Target.Tower.FirstSpeed[0]  = 2900.0f;
    DartRack.Target.Tower.SecondSpeed[0] = 3700.0f;
    DartRack.Target.Tower.FirstSpeed[1]  = 2900.0f;
    DartRack.Target.Tower.SecondSpeed[1] = 3700.0f;
    DartRack.Target.Tower.FirstSpeed[2]  = 2900.0f;
    DartRack.Target.Tower.SecondSpeed[2] = 3700.0f;
    DartRack.Target.Tower.FirstSpeed[3]  = 2900.0f;
    DartRack.Target.Tower.SecondSpeed[3] = 3700.0f;
    /* ���ز��� */
    DartRack.Target.Base.FirstSpeed[0]  = 3500.0f;
    DartRack.Target.Base.SecondSpeed[0] = 4500.0f;
    DartRack.Target.Base.FirstSpeed[1]  = 3500.0f;
    DartRack.Target.Base.SecondSpeed[1] = 4500.0f;
    DartRack.Target.Base.FirstSpeed[2]  = 3500.0f;
    DartRack.Target.Base.SecondSpeed[2] = 4500.0f;
    DartRack.Target.Base.FirstSpeed[3]  = 3500.0f;
    DartRack.Target.Base.SecondSpeed[3] = 4500.0f;
    /* Ħ������ǰת�� */
    SHOOT_MATCH_FIRST_SPEED  = DartRack.Target.Tower.FirstSpeed[0];
    SHOOT_MATCH_SECOND_SPEED = DartRack.Target.Tower.SecondSpeed[0];
}

/* ͬ��Ħ���ֲ��ٲ��� */
void ShootDiffCompensate(void)
{
    if ((DartRack.MotorMeasure.ShootMotorMeasure.ShootMotorSpeed[0] - SHOOT_MATCH_FIRST_SPEED) *
            (DartRack.MotorMeasure.ShootMotorMeasure.ShootMotorSpeed[1] - SHOOT_MATCH_FIRST_SPEED) <
        ZERO_POINT) {
        DartRack.Output.Shoot[0] += -DIFF_FIRST_DEGREE * fal(DartRack.MotorMeasure.ShootMotorMeasure.ShootMotorSpeed[1] + DartRack.MotorMeasure.ShootMotorMeasure.ShootMotorSpeed[0],
                                                             DIFF_FIRST_ALPHA, DIFF_DELTA);
        DartRack.Output.Shoot[1] += -DIFF_FIRST_DEGREE * fal(DartRack.MotorMeasure.ShootMotorMeasure.ShootMotorSpeed[0] + DartRack.MotorMeasure.ShootMotorMeasure.ShootMotorSpeed[1],
                                                             DIFF_FIRST_ALPHA, DIFF_DELTA);
    }
    if ((DartRack.MotorMeasure.ShootMotorMeasure.ShootMotorSpeed[2] - SHOOT_MATCH_SECOND_SPEED) *
            (DartRack.MotorMeasure.ShootMotorMeasure.ShootMotorSpeed[3] - SHOOT_MATCH_SECOND_SPEED) <
        ZERO_POINT) {
        DartRack.Output.Shoot[2] += -DIFF_SECOND_DEGREE * fal(DartRack.MotorMeasure.ShootMotorMeasure.ShootMotorSpeed[3] + DartRack.MotorMeasure.ShootMotorMeasure.ShootMotorSpeed[2],
                                                              DIFF_SECOND_ALPHA, DIFF_DELTA);
        DartRack.Output.Shoot[3] += -DIFF_SECOND_DEGREE * fal(DartRack.MotorMeasure.ShootMotorMeasure.ShootMotorSpeed[2] + DartRack.MotorMeasure.ShootMotorMeasure.ShootMotorSpeed[3],
                                                              DIFF_SECOND_ALPHA, DIFF_DELTA);
    }
}

void DartRackShoot(void)
{
    GimbalMotorControl(YAW_MOTOR_DIRECTION * DartRack.Output.Yaw,
                       PITCH_MOTOR_DIRECTION * DartRack.Output.Pitch,
                       CHAIN_MOTOR_DIRECTION * DartRack.Output.Chain);
    ShootMotorControl(DartRack.Output.Shoot[0],
                      DartRack.Output.Shoot[1],
                      DartRack.Output.Shoot[2],
                      DartRack.Output.Shoot[3]);
}

DartRackStateMachine_e DRSlast;
DartRackStateMachine_e DRSthis;
void DartRackStateMachineUpdate(void)
{
    DRSlast = DartRack.StateMachine;
    // ��¼֮ǰ״̬

    /* ������߱��� */
    if (Offline.YawMotor == DEVICE_OFFLINE ||
        Offline.PitchMotor == DEVICE_OFFLINE ||
        Offline.ChainMotor == DEVICE_OFFLINE ||
        Offline.ShootMotor[0] == DEVICE_OFFLINE ||
        Offline.ShootMotor[1] == DEVICE_OFFLINE ||
        Offline.ShootMotor[2] == DEVICE_OFFLINE ||
        Offline.ShootMotor[3] == DEVICE_OFFLINE ||
        Offline.YawAngleEncoder == DEVICE_OFFLINE ||
        Offline.PitchAngleEncoder == DEVICE_OFFLINE) {
        DartRack.StateMachine = DART_RACK_NO_FORCE;
        return;
    }

    /* ң�������߱��� */
    if (Offline.Remote == DEVICE_OFFLINE) {
        DartRack.StateMachine = DART_RACK_NO_FORCE;
        return;
    }

    /* YAW PITCH�ᳬλ���� */
    if (DartRack.Encoder.YawAngleEncoder.Angle < YAW_MIN_ANGLE ||
        DartRack.Encoder.YawAngleEncoder.Angle > YAW_MAX_ANGLE ||
        DartRack.Encoder.PitchAngleEncoder.Angle < PITCH_MIN_ANGLE ||
        DartRack.Encoder.PitchAngleEncoder.Angle > PITCH_MAX_ANGLE) {
        DartRack.StateMachine = DART_RACK_NO_FORCE;
        return;
    }

    /* ���ڼ�״̬������ */
    switch (Remote.rc.s[0]) {
        /* �Ҳ��˴����ϣ����ڼܽ������ģʽ,��ģʽ��Ħ���ֲ����ѵ��� */
        case RC_SW_UP:
            if (DartRack.StateMachine == DART_RACK_MATCH) { // ��Ҫ�ظ������
                return;
            }
            FRICTION_CMD_TEMP            = 0;                   // Ħ����΢��ָ��ͼ�¼
            FRICTION_COARSE_INIT_FLAG    = 1;                   // �ֵ�Ĭ�ϱ�־
            FRICTION_COARSE_TUNE_FLAG    = 0;                   // �ֵ��ı��־
            DART_RACK_CLIENT_CMD         = 0;                   // �ͻ���Ĭ��ǰ��վ
            LAUCH_CMD_FLAG               = 0;                   // ���ڷ���ָ���¼
            YAW_CMD_TEMP                 = 0;                   // YAW��ָ��ͼ�¼
            YAW_TARGET_ANGLE             = YAW_TOWER_ANGLE;     // Ĭ����׼ǰ��վ
            DART_RACK_AIM_SHOOT          = DART_RACK_AIM_TOWER; // ��׼��־
            DART_RACK_NUM_COUNT          = 0;                   // ������������
            DART_RACK_CHAIN_START_TIME   = 0;                   // ��������˶���ʱ
            DART_RACK_FINISH_FLAG        = 0;                   // ���ڷ����־
            DART_RACK_SHELTER_TIME_COUNT = 0;                   // �����ڵ�ʱ�����
            DartRack.StateMachine        = DART_RACK_MATCH;
            //			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);              // ����������
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET); // �رռ���
            break;
        /* �Ҳ��˴��м䣬���ڼܽ������ģʽ,��ģʽ��Ħ�����ٶȿɵ��� */
        case RC_SW_MID:
            //			if(DartRack.StateMachine == DART_RACK_TEST){
            //				return;
            //			}
            //			/* ��λ */
            //			if(DartRack.StateMachine == DART_RACK_NO_FORCE){
            //				DART_RACK_RESET_TIME_COUNT = DART_RACK_RESET_TIME;
            //				DartRack.StateMachine = DART_RACK_INIT;
            //			}
            //			if(DART_RACK_RESET_TIME_COUNT){
            //				DART_RACK_RESET_TIME_COUNT--;
            //			}
            //			else{
            DartRack.StateMachine = DART_RACK_TEST;
            //				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);    				// �رչ�����
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET); // ��������
                                                                 //			}
            break;
        /* �Ҳ��˴����£���ң�������ݳ������ڼܽ�������ģʽ */
        case RC_SW_DOWN:
            DartRack.StateMachine = DART_RACK_NO_FORCE;
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET); // �رռ���
            break;
        default:
            DartRack.StateMachine = DART_RACK_NO_FORCE;
            //			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET); // �رռ���
            break;
    }

    DRSthis = DartRack.StateMachine; // ��¼֮��״̬
}

void MotorMeasureUpdate(void)
{
    GimbalMotorMeasureUpdate(&DartRack.MotorMeasure.GimbalMotorMeasure);
    ShootMotorMeasureUpdate(&DartRack.MotorMeasure.ShootMotorMeasure);
    /* Test */
    for (int i = 0; i < 4; i++) {
        DartRack.MotorMeasure.ShootMotorMeasure.ShootMotorSpeedTest[i] = fabsf(DartRack.MotorMeasure.ShootMotorMeasure.ShootMotorSpeed[i]);
    }
    for (int i = 0; i < 2; i++) {
        DartRack.MotorMeasure.ShootMotorMeasure.ShootMotorSpeedDiff[i] = (DartRack.MotorMeasure.ShootMotorMeasure.ShootMotorSpeed[2 * i] +
                                                                          DartRack.MotorMeasure.ShootMotorMeasure.ShootMotorSpeed[2 * i + 1]);
    }
}

void EncoderMeasureUpdate(void)
{
    AngleEncoderMeasureUpdate(&DartRack.Encoder);
}

void MotorAlgorithmUpdate(void)
{
    /* �����ظ������㷨 */
    if (DRSthis == DRSlast) {
        return;
    }

    /* ��λ */
    if (DartRack.StateMachine == DART_RACK_INIT) {
        cascade_PID_init(&DartRack.Pid.Yaw,
                         YAW_ANGLE_OPERATE,
                         YAW_SPEED_OPERATE,
                         ANGLE_INIT_TARGET,
                         SPEED_INIT_TARGET,
                         ANGLE_INPUT_ALPHA,
                         ANGLE_OUTPUT_ALPHA,
                         SPEED_INPUT_ALPHA,
                         SPEED_OUTPUT_ALPHA,
                         ANGLE_DELTA,
                         ANGLE_ALPHA,
                         SPEED_DELTA,
                         SPEED_ALPHA,
                         YAW_MAX_ANGULAR_VELOCITY,
                         YAW_MAX_ANGULAR_IVELOCITY,
                         YAW_MAX_OUTPUT,
                         M3508_MAX_IOUTPUT,
                         ANGLE_DEAD_AREA,
                         SPEED_DEAD_AREA);
        cascade_PID_init(&DartRack.Pid.Pitch,
                         PITCH_ANGLE_OPERATE,
                         PITCH_SPEED_OPERATE,
                         ANGLE_INIT_TARGET,
                         SPEED_INIT_TARGET,
                         ANGLE_INPUT_ALPHA,
                         ANGLE_OUTPUT_ALPHA,
                         SPEED_INPUT_ALPHA,
                         SPEED_OUTPUT_ALPHA,
                         ANGLE_DELTA,
                         ANGLE_ALPHA,
                         SPEED_DELTA,
                         SPEED_ALPHA,
                         YAW_MAX_ANGULAR_VELOCITY,
                         PITCH_MAX_ANGULAR_IVELOCITY,
                         PITCH_MAX_OUTPUT,
                         M3508_MAX_IOUTPUT,
                         ANGLE_DEAD_AREA,
                         SPEED_DEAD_AREA); // pitch��PID
    }
    if (DartRack.StateMachine == DART_RACK_MATCH) {
        PID_init(&DartRack.Pid.Chain,
                 PID_POSITION,
                 CHAIN_OPERATE,
                 SPEED_INIT_TARGET,
                 SPEED_INPUT_ALPHA,
                 SPEED_OUTPUT_ALPHA,
                 SPEED_DELTA,
                 SPEED_ALPHA,
                 M2006_MAX_OUTPUT,
                 M2006_MAX_IOUTPUT,
                 SPEED_DEAD_AREA);
        /* ������ ADRC */
        ADRC_init(&DartRack.Adrc.Shoot[0],
                  ADRC_SHOOT_DEFAULT,
                  M3508_MAX_OUTPUT);
        ADRC_init(&DartRack.Adrc.Shoot[1],
                  ADRC_SHOOT_DEFAULT,
                  M3508_MAX_OUTPUT);
        ADRC_init(&DartRack.Adrc.Shoot[2],
                  ADRC_SHOOT_DEFAULT,
                  M3508_MAX_OUTPUT);
        ADRC_init(&DartRack.Adrc.Shoot[3],
                  ADRC_SHOOT_DEFAULT,
                  M3508_MAX_OUTPUT);
        cascade_PID_init(&DartRack.Pid.Yaw,
                         YAW_ANGLE_OPERATE,
                         YAW_SPEED_OPERATE,
                         ANGLE_INIT_TARGET,
                         SPEED_INIT_TARGET,
                         ANGLE_INPUT_ALPHA,
                         ANGLE_OUTPUT_ALPHA,
                         SPEED_INPUT_ALPHA,
                         SPEED_OUTPUT_ALPHA,
                         ANGLE_DELTA,
                         ANGLE_ALPHA,
                         SPEED_DELTA,
                         SPEED_ALPHA,
                         YAW_MAX_ANGULAR_VELOCITY,
                         YAW_MAX_ANGULAR_IVELOCITY,
                         YAW_MAX_OUTPUT,
                         M3508_MAX_IOUTPUT,
                         ANGLE_DEAD_AREA,
                         SPEED_DEAD_AREA); // yaw��PID
        cascade_PID_init(&DartRack.Pid.Pitch,
                         PITCH_ANGLE_OPERATE,
                         PITCH_SPEED_OPERATE,
                         ANGLE_INIT_TARGET,
                         SPEED_INIT_TARGET,
                         ANGLE_INPUT_ALPHA,
                         ANGLE_OUTPUT_ALPHA,
                         SPEED_INPUT_ALPHA,
                         SPEED_OUTPUT_ALPHA,
                         ANGLE_DELTA,
                         ANGLE_ALPHA,
                         SPEED_DELTA,
                         SPEED_ALPHA,
                         YAW_MAX_ANGULAR_VELOCITY,
                         PITCH_MAX_ANGULAR_IVELOCITY,
                         PITCH_MAX_OUTPUT,
                         M3508_MAX_IOUTPUT,
                         ANGLE_DEAD_AREA,
                         SPEED_DEAD_AREA); // pitch��PID
    }
    if (DartRack.StateMachine == DART_RACK_TEST) {
        PID_init(&DartRack.Pid.Chain, PID_POSITION, CHAIN_OPERATE, SPEED_INIT_TARGET, SPEED_INPUT_ALPHA, SPEED_OUTPUT_ALPHA, SPEED_DELTA, SPEED_ALPHA, M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT, SPEED_DEAD_AREA); // ���ʹ����PID
        ADRC_init(&DartRack.Adrc.Shoot[0],
                  ADRC_SHOOT_DEFAULT,
                  M3508_MAX_OUTPUT);
        ADRC_init(&DartRack.Adrc.Shoot[1],
                  ADRC_SHOOT_DEFAULT,
                  M3508_MAX_OUTPUT);
        ADRC_init(&DartRack.Adrc.Shoot[2],
                  ADRC_SHOOT_DEFAULT,
                  M3508_MAX_OUTPUT);
        ADRC_init(&DartRack.Adrc.Shoot[3],
                  ADRC_SHOOT_DEFAULT,
                  M3508_MAX_OUTPUT);
        cascade_PID_init(&DartRack.Pid.Yaw,
                         YAW_ANGLE_OPERATE,
                         YAW_SPEED_OPERATE,
                         ANGLE_INIT_TARGET,
                         SPEED_INIT_TARGET,
                         ANGLE_INPUT_ALPHA,
                         ANGLE_OUTPUT_ALPHA,
                         SPEED_INPUT_ALPHA,
                         SPEED_OUTPUT_ALPHA,
                         ANGLE_DELTA,
                         ANGLE_ALPHA,
                         SPEED_DELTA,
                         SPEED_ALPHA,
                         YAW_MAX_ANGULAR_VELOCITY,
                         YAW_MAX_ANGULAR_IVELOCITY,
                         YAW_MAX_OUTPUT,
                         M3508_MAX_IOUTPUT,
                         ANGLE_DEAD_AREA,
                         SPEED_DEAD_AREA);
        cascade_PID_init(&DartRack.Pid.Pitch,
                         PITCH_ANGLE_OPERATE,
                         PITCH_SPEED_OPERATE,
                         ANGLE_INIT_TARGET,
                         SPEED_INIT_TARGET,
                         ANGLE_INPUT_ALPHA,
                         ANGLE_OUTPUT_ALPHA,
                         SPEED_INPUT_ALPHA,
                         SPEED_OUTPUT_ALPHA,
                         ANGLE_DELTA,
                         ANGLE_ALPHA,
                         SPEED_DELTA,
                         SPEED_ALPHA,
                         YAW_MAX_ANGULAR_VELOCITY,
                         PITCH_MAX_ANGULAR_IVELOCITY,
                         PITCH_MAX_OUTPUT,
                         M3508_MAX_IOUTPUT,
                         ANGLE_DEAD_AREA,
                         SPEED_DEAD_AREA); // pitch��PID
    }
}

void GimbalCommandUpdate(void)
{
    if (DartRack.StateMachine == DART_RACK_INIT) {
        DartRack.Command.Yaw   = YAW_ZERO_ANGLE;
        DartRack.Output.Yaw    = cascade_PID_calc(&DartRack.Pid.Yaw,
                                                  DartRack.Encoder.YawAngleEncoder.Angle,
                                                  DartRack.Encoder.YawAngleEncoder.AnguarVelocity,
                                                  DartRack.Command.Yaw,
                                                  CIRCLE_ANGLE,
                                                  ZERO_POINT);
        DartRack.Command.Pitch = PITCH_TARGET_ANGLE;
        DartRack.Output.Pitch  = cascade_PID_calc(&DartRack.Pid.Pitch,
                                                  DartRack.Encoder.PitchAngleEncoder.Angle,
                                                  DartRack.Encoder.PitchAngleEncoder.AnguarVelocity,
                                                  DartRack.Command.Pitch,
                                                  CIRCLE_ANGLE,
                                                  ZERO_POINT);
    }
    if (DartRack.StateMachine == DART_RACK_MATCH) {
        /* �ֵ��Ƕ� */
        DART_RACK_CLIENT_CMD = Referee.DartClientCmd.DartAttackTarget;
        if (GIMBAL_CMD_YAW_COARSE_KEYMAP == DART_RACK_AIM_TOWER) { // ����ǰ��վ
            YAW_TARGET_ANGLE          = YAW_TOWER_ANGLE;
            DART_RACK_AIM_SHOOT       = DART_RACK_AIM_TOWER;
            FRICTION_COARSE_TUNE_FLAG = 1;
        }
        if (GIMBAL_CMD_YAW_COARSE_KEYMAP == DART_RACK_AIM_BASE) { // ��������
            YAW_TARGET_ANGLE          = YAW_BASE_ANGLE;
            DART_RACK_AIM_SHOOT       = DART_RACK_AIM_BASE;
            FRICTION_COARSE_TUNE_FLAG = 1;
        }
        /* ϸ���Ƕ� */
        if (!is_zero(GIMBAL_CMD_YAW_SLIGHT_KEYMAP)) {
            YAW_CMD_TEMP = GIMBAL_CMD_YAW_SLIGHT_KEYMAP;
        }
        if (!is_zero(YAW_CMD_TEMP) && is_zero(GIMBAL_CMD_YAW_SLIGHT_KEYMAP)) { // �½��ش���
            YAW_TARGET_ANGLE += YAW_CMD_TEMP;
            YAW_CMD_TEMP = ZERO_POINT;
        }

        DartRack.Command.Yaw = YAW_TARGET_ANGLE;
        if (fabsf(DartRack.Command.Yaw) < 90.0f) DartRack.Command.Yaw = YAW_TOWER_ANGLE; // Ĭ����׼����
        DartRack.Output.Yaw    = cascade_PID_calc(&DartRack.Pid.Yaw,
                                                  DartRack.Encoder.YawAngleEncoder.Angle,
                                                  DartRack.Encoder.YawAngleEncoder.AnguarVelocity,
                                                  DartRack.Command.Yaw,
                                                  CIRCLE_ANGLE,
                                                  ZERO_POINT);
        DartRack.Command.Pitch = PITCH_TARGET_ANGLE;
        DartRack.Output.Pitch  = cascade_PID_calc(&DartRack.Pid.Pitch,
                                                  DartRack.Encoder.PitchAngleEncoder.Angle,
                                                  DartRack.Encoder.PitchAngleEncoder.AnguarVelocity,
                                                  DartRack.Command.Pitch,
                                                  CIRCLE_ANGLE,
                                                  ZERO_POINT);
    }
    if (DartRack.StateMachine == DART_RACK_TEST) {
        /* �Ƕȴֵ� */
        if (HandleGimbalLeftside()) { // ǰ��վ
            YAW_TARGET_ANGLE     = YAW_TOWER_ANGLE;
            DartRack.Command.Yaw = YAW_TARGET_ANGLE;
        }
        if (HandleGimbalRightside()) { // ����
            YAW_TARGET_ANGLE     = YAW_BASE_ANGLE;
            DartRack.Command.Yaw = YAW_TARGET_ANGLE;
        }
        if (HandleGimbalDownside()) { // ��λ
            YAW_TARGET_ANGLE     = YAW_ZERO_ANGLE;
            DartRack.Command.Yaw = YAW_TARGET_ANGLE;
        }
        /* ������� */
        if (fabsf(DartRack.Command.Yaw) < 90.0f) YAW_TARGET_ANGLE = YAW_ZERO_ANGLE;
        /* �Ƕ�΢��*/
        if (!is_zero(GIMBAL_CMD_YAW_SLIGHT_KEYMAP)) {
            YAW_CMD_TEMP = GIMBAL_CMD_YAW_SLIGHT_KEYMAP;
        }
        if (!is_zero(YAW_CMD_TEMP) && is_zero(GIMBAL_CMD_YAW_SLIGHT_KEYMAP)) { // �½��ش���
            YAW_TARGET_ANGLE += YAW_CMD_TEMP;
            YAW_CMD_TEMP = ZERO_POINT;
        }
        DartRack.Command.Yaw = YAW_TARGET_ANGLE;
        loop_constrain(DartRack.Command.Yaw, ZERO_POINT, CIRCLE_ANGLE);
        DartRack.Output.Yaw = cascade_PID_calc(&DartRack.Pid.Yaw,
                                               DartRack.Encoder.YawAngleEncoder.Angle,
                                               DartRack.Encoder.YawAngleEncoder.AnguarVelocity,
                                               DartRack.Command.Yaw,
                                               CIRCLE_ANGLE,
                                               ZERO_POINT);
        if (fabsf(DartRack.Command.Pitch) < 10.0f) DartRack.Command.Pitch = PITCH_TARGET_ANGLE;
        DartRack.Command.Pitch += GIMBAL_CMD_PITCH_TEST_KEYMAP;
        constrain(DartRack.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
        DartRack.Output.Pitch = cascade_PID_calc(&DartRack.Pid.Pitch,
                                                 DartRack.Encoder.PitchAngleEncoder.Angle,
                                                 DartRack.Encoder.PitchAngleEncoder.AnguarVelocity,
                                                 DartRack.Command.Pitch,
                                                 CIRCLE_ANGLE,
                                                 ZERO_POINT);
        /* ��¼��� */
        if (HandleGimbalUpside()) {
            if (in_range(YAW_TARGET_ANGLE, YAW_TOWER_MIN_ANGLE, YAW_TOWER_MAX_ANGLE)) {
                YAW_TOWER_ANGLE = YAW_TARGET_ANGLE;
            }
            if (in_range(YAW_TARGET_ANGLE, YAW_BASE_MIN_ANGLE, YAW_BASE_MAX_ANGLE)) {
                YAW_BASE_ANGLE = YAW_TARGET_ANGLE;
            }
        }
    }
    if (DartRack.StateMachine == DART_RACK_NO_FORCE) {
        DartRack.Output.Yaw   = ZERO_POINT;
        DartRack.Output.Pitch = ZERO_POINT;
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET); // �رռ���
    }
}

void ShootCommandUpdate(void)
{
    if (DartRack.StateMachine == DART_RACK_MATCH) {
        /* �ֶ����� */
        if (DART_RACK_NUM_COUNT < DART_RACK_NUM) {
            /* �ֵ�Ħ�����ٶ� */
            if (FRICTION_COARSE_INIT_FLAG || FRICTION_COARSE_TUNE_FLAG) {
                if (DART_RACK_AIM_SHOOT == DART_RACK_AIM_TOWER) {
                    SHOOT_MATCH_FIRST_SPEED  = DartRack.Target.Tower.FirstSpeed[DART_RACK_NUM_COUNT];
                    SHOOT_MATCH_SECOND_SPEED = DartRack.Target.Tower.SecondSpeed[DART_RACK_NUM_COUNT];
                }
                if (DART_RACK_AIM_SHOOT == DART_RACK_AIM_BASE) {
                    SHOOT_MATCH_FIRST_SPEED  = DartRack.Target.Base.FirstSpeed[DART_RACK_NUM_COUNT];
                    SHOOT_MATCH_SECOND_SPEED = DartRack.Target.Base.SecondSpeed[DART_RACK_NUM_COUNT];
                }
                if (DART_RACK_AIM_SHOOT == DART_RACK_AIM_STRAIGHT) {
                    SHOOT_MATCH_FIRST_SPEED  = DartRack.Target.Straight.FirstSpeed[DART_RACK_NUM_COUNT];
                    SHOOT_MATCH_SECOND_SPEED = DartRack.Target.Straight.SecondSpeed[DART_RACK_NUM_COUNT];
                }
                FRICTION_COARSE_INIT_FLAG = 0;
                FRICTION_COARSE_TUNE_FLAG = 0;
            }
            /* ϸ��Ħ�����ٶ� */
            switch (Remote.rc.s[1]) {
                case RC_SW_UP: // 10�ٶȵ�
                    if (!is_zero(SHOOT_CMD_FRICTION_FIRST_LEVEL_KEYMAP)) {
                        FRICTION_CMD_TEMP = SHOOT_CMD_FRICTION_FIRST_LEVEL_KEYMAP;
                    }
                    if (!is_zero(FRICTION_CMD_TEMP) && is_zero(SHOOT_CMD_FRICTION_FIRST_LEVEL_KEYMAP)) { // �½��ش���
                        SHOOT_MATCH_FIRST_SPEED += FRICTION_CMD_TEMP;
                        SHOOT_MATCH_SECOND_SPEED += FRICTION_CMD_TEMP;
                        FRICTION_CMD_TEMP = ZERO_POINT;
                    }
                    break;
                case RC_SW_MID: // 100�ٶȵ�
                    if (!is_zero(SHOOT_CMD_FRICTION_SECOND_LEVEL_KEYMAP)) {
                        FRICTION_CMD_TEMP = SHOOT_CMD_FRICTION_SECOND_LEVEL_KEYMAP;
                    }
                    if (!is_zero(FRICTION_CMD_TEMP) && is_zero(SHOOT_CMD_FRICTION_SECOND_LEVEL_KEYMAP)) { // �½��ش���
                        SHOOT_MATCH_FIRST_SPEED += FRICTION_CMD_TEMP;
                        SHOOT_MATCH_SECOND_SPEED += FRICTION_CMD_TEMP;
                        FRICTION_CMD_TEMP = ZERO_POINT;
                    }
                    break;
                default:
                    break;
            }
            /* �������� */
            switch (DART_RACK_AIM_SHOOT) {
                case DART_RACK_AIM_TOWER:
                    for (uint8_t i = 0; i < DART_RACK_NUM; i++) {
                        DartRack.Target.Tower.FirstSpeed[i]  = SHOOT_MATCH_FIRST_SPEED;
                        DartRack.Target.Tower.SecondSpeed[i] = SHOOT_MATCH_SECOND_SPEED;
                    }
                    break;
                case DART_RACK_AIM_BASE:
                    for (uint8_t i = 0; i < DART_RACK_NUM; i++) {
                        DartRack.Target.Base.FirstSpeed[i]  = SHOOT_MATCH_FIRST_SPEED;
                        DartRack.Target.Base.SecondSpeed[i] = SHOOT_MATCH_SECOND_SPEED;
                    }
                    break;
                case DART_RACK_AIM_STRAIGHT:
                    for (uint8_t i = 0; i < DART_RACK_NUM; i++) {
                        DartRack.Target.Straight.FirstSpeed[i]  = SHOOT_MATCH_FIRST_SPEED;
                        DartRack.Target.Straight.SecondSpeed[i] = SHOOT_MATCH_SECOND_SPEED;
                    }
                    break;
                default:
                    break;
            }

            /* ������� */
            if (SHOOT_CMD_LAUCH_KEYMAP && !LAUCH_CMD_FLAG) {
                LAUCH_CMD_FLAG             = 1;
                DART_RACK_CHAIN_START_TIME = GetSystemTimer();
            }
            if (LAUCH_CMD_FLAG) {
                DartRack.Command.Chain = CHAIN_MATCH_SPEED;
                /* ���ڼ�� */
                /* ����ż�� */
                //						if(GetLauch()){
                //							DART_RACK_SHELTER_TIME_COUNT++;
                //							if(DART_RACK_SHELTER_TIME_COUNT > DART_RACK_SHELTER_TIME) {
                //								DART_RACK_FINISH_FLAG = 1;
                //							}
                //							if(DART_RACK_FINISH_FLAG){
                //								LAUCH_CMD_FLAG = 0;
                //								FRICTION_COARSE_INIT_FLAG = 0;
                //								DartRack.Command.Chain = ZERO_POINT;       // �������ֹͣ
                //								DART_RACK_NUM_COUNT++;
                //								DART_RACK_FINISH_FLAG = 0;
                //								DART_RACK_SHELTER_TIME_COUNT = 0;
                //							}
                //						}
                /* ��ʱ���� */
                if (GetSystemTimer() - DART_RACK_CHAIN_START_TIME >= RACK_LAUCH_TIME[DART_RACK_NUM_COUNT]) {
                    DartRack.Command.Chain    = ZERO_POINT;
                    LAUCH_CMD_FLAG            = 0;
                    FRICTION_COARSE_INIT_FLAG = 0;
                    FRICTION_COARSE_TUNE_FLAG = 0;
                    DART_RACK_NUM_COUNT++;
                }
            }
        } else { // ��������
            DartRack.Command.Chain = ZERO_POINT;
        }
        DartRack.Output.Chain = PID_calc(&DartRack.Pid.Chain,
                                         DartRack.MotorMeasure.ShootMotorMeasure.ChainMotorSpeed,
                                         CHAIN_MOTOR_DIRECTION * DartRack.Command.Chain,
                                         0);
        /* Ħ���ֿ��� */
        DartRack.Output.Shoot[0] = ADRC_calc(&DartRack.Adrc.Shoot[0],
                                             DartRack.MotorMeasure.ShootMotorMeasure.ShootMotorSpeed[0],
                                             SHOOT_MOTOR1_DIRECTION * SHOOT_MATCH_FIRST_SPEED);
        DartRack.Output.Shoot[1] = ADRC_calc(&DartRack.Adrc.Shoot[1],
                                             DartRack.MotorMeasure.ShootMotorMeasure.ShootMotorSpeed[1],
                                             SHOOT_MOTOR2_DIRECTION * SHOOT_MATCH_FIRST_SPEED);
        DartRack.Output.Shoot[2] = ADRC_calc(&DartRack.Adrc.Shoot[2],
                                             DartRack.MotorMeasure.ShootMotorMeasure.ShootMotorSpeed[2],
                                             SHOOT_MOTOR3_DIRECTION * SHOOT_MATCH_SECOND_SPEED);
        DartRack.Output.Shoot[3] = ADRC_calc(&DartRack.Adrc.Shoot[3],
                                             DartRack.MotorMeasure.ShootMotorMeasure.ShootMotorSpeed[3],
                                             SHOOT_MOTOR4_DIRECTION * SHOOT_MATCH_SECOND_SPEED);
    }
    if (DartRack.StateMachine == DART_RACK_TEST) {
        DART_RACK_NUM_COUNT      = 0; // ����������������
        DartRack.Command.Chain   = MOTOR_CHAIN_MAX_SPEED * SHOOT_CMD_CHAIN_KEYMAP;
        DartRack.Output.Chain    = PID_calc(&DartRack.Pid.Chain,
                                            DartRack.MotorMeasure.ShootMotorMeasure.ChainMotorSpeed,
                                            CHAIN_MOTOR_DIRECTION * DartRack.Command.Chain,
                                            0);
        DartRack.Command.Shoot   = MOTOR_SHOOT_MAX_SPEED * SHOOT_CMD_FRICTION_KEYMAP;
        DartRack.Output.Shoot[0] = ADRC_calc(&DartRack.Adrc.Shoot[0],
                                             DartRack.MotorMeasure.ShootMotorMeasure.ShootMotorSpeed[0],
                                             SHOOT_MOTOR1_DIRECTION * DartRack.Command.Shoot);
        DartRack.Output.Shoot[1] = ADRC_calc(&DartRack.Adrc.Shoot[1],
                                             DartRack.MotorMeasure.ShootMotorMeasure.ShootMotorSpeed[1],
                                             SHOOT_MOTOR2_DIRECTION * DartRack.Command.Shoot);
        DartRack.Output.Shoot[2] = ADRC_calc(&DartRack.Adrc.Shoot[2],
                                             DartRack.MotorMeasure.ShootMotorMeasure.ShootMotorSpeed[2],
                                             SHOOT_MOTOR3_DIRECTION * DartRack.Command.Shoot);
        DartRack.Output.Shoot[3] = ADRC_calc(&DartRack.Adrc.Shoot[3],
                                             DartRack.MotorMeasure.ShootMotorMeasure.ShootMotorSpeed[3],
                                             SHOOT_MOTOR4_DIRECTION * DartRack.Command.Shoot);
    }
    if (DartRack.StateMachine == DART_RACK_NO_FORCE || DartRack.StateMachine == DART_RACK_INIT) {
        DartRack.Output.Chain    = ZERO_POINT;
        DartRack.Output.Shoot[0] = ZERO_POINT;
        DartRack.Output.Shoot[1] = ZERO_POINT;
        DartRack.Output.Shoot[2] = ZERO_POINT;
        DartRack.Output.Shoot[3] = ZERO_POINT;
    }
}

// void DartLauchSpeedUpdate(void){
//	fp32 speed = GetLauchSpeed(DART_LENGTH);
//	if(!is_zero(speed)){
//		DartRack.Output.LauchSpeed = speed;
//	}
// }

void get_encoder_angle(DartRackEncoder_t *encoder)
{
    encoder->YawAngleEncoder.Angle   = DartRack.Encoder.YawAngleEncoder.Angle;
    encoder->PitchAngleEncoder.Angle = DartRack.Encoder.PitchAngleEncoder.Angle;
}

void get_shoot_speed(fp32 *first_speed, fp32 *second_speed, ShootMotorMeasure_t *measure)
{
    *first_speed  = SHOOT_MATCH_FIRST_SPEED;
    *second_speed = SHOOT_MATCH_SECOND_SPEED;
    memcpy(measure, &DartRack.MotorMeasure.ShootMotorMeasure, sizeof(ShootMotorMeasure_t));
}
