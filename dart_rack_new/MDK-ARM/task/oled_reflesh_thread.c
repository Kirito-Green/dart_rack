#include "oled_reflesh_thread.h"
#include "cmsis_os.h"
#include "string.h"
#include "launch_thread.h"
#include "oled.h"
#include "arm_math.h"
#include "user_lib.h"
#include "hx711.h"
#include "motor.h"
#include "setting.h"
#include "launch_thread.h"

static fp32 force_left;
static fp32 force_right;
static DartRackTarget_t target;
static DartRackEncoder_t encoder;
static DartRackMotorMeasure_t motor_measure;
static DartRackTarget_t target;
static uint8_t rx_buf[20];
static uint8_t rx_len;
static DartRackStateMachine_e state;

void oled_show_data(void);

/**
 * @brief ������oled_reflesh_thread��ʹ�����Ը��ִ������͵�������ݲ��ϸ��� OLED ��ʾ����
 *
 * @param argument ��argument��������ָ����������Ҫ���ݸ��̵߳��κθ������ݵ�ָ�롣����������£���û�б�ʹ�ã���������Ժ�������
 */
void oled_reflesh_thread(void const *argument)
{
    oled_ready();
    for (;;) {
        get_state(&state);
        // switch (state) {
        //     case DART_RACK_NO_FORCE:

        //         break;
        //     default:
        get_hx711_data(&force_left, &force_right);
        get_target(&target);
        get_encoder_angle(&encoder);
        gimbal_motor_measure_update(&motor_measure.gimbal_motor_measure);
        shoot_motor_measure_update(&motor_measure.shoot_motor_measure);
        oled_show_data();
        // break;
        // }

        osDelay(300);
    }
}

/**
 * @brief ������oled_show_fp32���� OLED ��ʾ������ʾ��������С�����ָ��λ����
 *
 * @param val ��val��������һ��������(fp32)������Ҫ��OLED��Ļ����ʾ��ֵ��
 * @param digit ��digit��������һ��������ָ��Ҫ��ʾ�ĸ���ֵС������λ����
 */
void oled_show_fp32(fp32 val, uint8_t digit)
{
    if (sign(val) == NEGTIVE) {
        oled_enter_string((uint8_t *)"-", sizeof("-"));
    }
    fp32_to_string(fabsf(val), digit, rx_buf, &rx_len);
    oled_enter_string(rx_buf, rx_len + 1);
}

/**
 * @brief ������oled_show_data����OLED��Ļ����ʾ��������ֵ��
 */
void oled_show_data(void)
{
    oled_reset();

    oled_enter_string((uint8_t *)"Y ", sizeof("Y "));
    oled_show_fp32(target.yaw, OLED_SHOW_ENCODER_DIGIT);
    oled_enter_string((uint8_t *)" ", sizeof(" "));
    oled_show_fp32(encoder.yaw_angle_encoder.angle, OLED_SHOW_ENCODER_DIGIT);
    oled_new_line();

    oled_enter_string((uint8_t *)"P ", sizeof("P "));
    oled_show_fp32(target.pitch, OLED_SHOW_FP32_DIGIT);
    // oled_show_fp32(encoder.pitch_angle_encoder.angle, OLED_SHOW_ENCODER_DIGIT);
    oled_show_fp32(motor_measure.gimbal_motor_measure.pitch_dist, OLED_SHOW_FP32_DIGIT);
    oled_new_line();

    oled_enter_string((uint8_t *)"FL ", sizeof("FL "));
    oled_show_fp32(force_left, OLED_SHOW_FP32_DIGIT);
    oled_new_line();

    oled_enter_string((uint8_t *)"FR ", sizeof("FR "));
    oled_show_fp32(force_right, OLED_SHOW_FP32_DIGIT);
    oled_new_line();

    oled_enter_string((uint8_t *)"DL ", sizeof("DL "));
    oled_show_fp32(target.drag_left, OLED_SHOW_FP32_DIGIT);
    oled_enter_string((uint8_t *)" ", sizeof(" "));
    oled_show_fp32(motor_measure.shoot_motor_measure.drag_left_dist, OLED_SHOW_FP32_DIGIT);
    oled_new_line();

    oled_enter_string((uint8_t *)"DR ", sizeof("DR "));
    oled_show_fp32(target.drag_right, OLED_SHOW_FP32_DIGIT);
    oled_enter_string((uint8_t *)" ", sizeof(" "));
    oled_show_fp32(motor_measure.shoot_motor_measure.drag_right_dist, OLED_SHOW_FP32_DIGIT);
    oled_new_line();

    oled_enter_string((uint8_t *)"LD ", sizeof("LD "));
    oled_show_fp32(target.load, OLED_SHOW_FP32_DIGIT);
    oled_enter_string((uint8_t *)" ", sizeof(" "));
    oled_show_fp32(motor_measure.shoot_motor_measure.load_dist, OLED_SHOW_FP32_DIGIT);
    oled_new_line();

    oled_enter_string((uint8_t *)"AD ", sizeof("AD "));
    oled_show_fp32(target.adjust, OLED_SHOW_FP32_DIGIT);
    oled_enter_string((uint8_t *)" ", sizeof(" "));
    oled_show_fp32(motor_measure.shoot_motor_measure.adjust_dist, OLED_SHOW_FP32_DIGIT);
    oled_new_line();

    oled_refresh();
}
