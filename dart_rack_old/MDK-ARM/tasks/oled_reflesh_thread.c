#include "oled_reflesh_thread.h"
#include "cmsis_os.h"
#include "string.h"
#include "LauchThread.h"
#include "oled.h"
#include "arm_math.h"
#include "user_lib.h"

static fp32 SHOOT_MATCH_FIRST_SPEED;
static fp32 SHOOT_MATCH_SECOND_SPEED;
static DartRackEncoder_t encoder;
static ShootMotorMeasure_t motor_measure;
static uint8_t rx_buf[20];
static uint8_t rx_len;

void oled_reflesh_thread(void const *argument)
{
    oled_ready();
    for (;;) {
        get_encoder_angle(&encoder);
        get_shoot_speed(&SHOOT_MATCH_FIRST_SPEED, &SHOOT_MATCH_SECOND_SPEED, &motor_measure);
        oled_show();

        osDelay(300);
    }
}

void oled_show(void)
{
    oled_reset();
    fp32_to_string(encoder.YawAngleEncoder.Angle, 2, rx_buf, &rx_len);
    oled_enter_string((uint8_t *)"Y ", sizeof("Y "));
    oled_enter_string(rx_buf, rx_len + 1);
    oled_new_line();

    fp32_to_string(encoder.PitchAngleEncoder.Angle, 2, rx_buf, &rx_len);
    oled_enter_string((uint8_t *)"P ", sizeof("P "));
    oled_enter_string(rx_buf, rx_len + 1);
    oled_new_line();

    fp32_to_string(SHOOT_MATCH_FIRST_SPEED, 2, rx_buf, &rx_len);
    oled_enter_string(rx_buf, rx_len + 1);
    oled_new_line();

    fp32_to_string(fabsf(motor_measure.ShootMotorSpeed[0]), 2, rx_buf, &rx_len);
    oled_enter_string(rx_buf, rx_len + 1);
    oled_enter_string((uint8_t *)" ", sizeof(" ")); // space for the other speed.
    fp32_to_string(fabsf(motor_measure.ShootMotorSpeed[1]), 2, rx_buf, &rx_len);
    oled_enter_string(rx_buf, rx_len + 1);
    oled_new_line();

    fp32_to_string(SHOOT_MATCH_SECOND_SPEED, 2, rx_buf, &rx_len);
    oled_enter_string(rx_buf, rx_len + 1);
    oled_new_line();

    fp32_to_string(fabsf(motor_measure.ShootMotorSpeed[2]), 2, rx_buf, &rx_len);
    oled_enter_string(rx_buf, rx_len + 1);
    oled_enter_string((uint8_t *)" ", sizeof(" ")); // space for the other speed.
    fp32_to_string(fabsf(motor_measure.ShootMotorSpeed[3]), 2, rx_buf, &rx_len);
    oled_enter_string(rx_buf, rx_len + 1);
    oled_new_line();

    oled_refresh();
}
