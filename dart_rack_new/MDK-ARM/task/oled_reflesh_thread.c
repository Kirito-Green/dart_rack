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
 * @brief 函数“oled_reflesh_thread”使用来自各种传感器和电机的数据不断更新 OLED 显示屏。
 *
 * @param argument “argument”参数是指向您可能想要传递给线程的任何附加数据的指针。在这种情况下，它没有被使用，所以你可以忽略它。
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
 * @brief 函数“oled_show_fp32”在 OLED 显示屏上显示浮点数，小数点后指定位数。
 *
 * @param val “val”参数是一个浮点数(fp32)，代表要在OLED屏幕上显示的值。
 * @param digit “digit”参数是一个整数，指定要显示的浮点值小数点后的位数。
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
 * @brief 函数“oled_show_data”在OLED屏幕上显示各种数据值。
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
