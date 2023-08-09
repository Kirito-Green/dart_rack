#include "hx711.h"
#include "user_lib.h"
#include "cmsis_os.h"

// A channel 128+
static uint_fast32_t count;
static fp32 force_left  = 0.0f;
static fp32 force_right = 0.0f;

/**
 * @brief ���� hx711_read ������ HX711 ��������ȡ���ݣ������ݽ��յ������ݼ�����ֵ��
 */
void hx711_read(void)
{
    HX711_LEFT_SCL_RESET;
    mcu_delay_us(4);
    HX711_RIGHT_SCL_RESET;
    mcu_delay_us(4);

    if (!HX711_LEFT_SDA_READ) {
        /* HX711_LEFT */
        // while (HX711_LEFT_SDA_READ) { mcu_delay_ms(1); } /* wait until SDA line goes from HIGH to LOW */
        count = 0;
        for (uint8_t i = 0; i < 24; i++) {
            HX711_LEFT_SCL_SET;
            mcu_delay_us(4);
            HX711_LEFT_SCL_RESET;
            mcu_delay_us(4);
            count = count << 1;
            if (HX711_LEFT_SDA_READ) {
                count++;
            }
        }
        HX711_LEFT_SCL_SET;
        mcu_delay_us(4);
        HX711_LEFT_SCL_RESET;
        mcu_delay_us(4);
        count ^= 0x800000;
        // count &= 0x8fffff; /* clear overflow bit */
        force_left = (fp32)count / 8388607.0f * 500.0f;
    }

    if (!HX711_RIGHT_SDA_READ) {
        /* HX711_RIGHT*/
        count = 0;
        // while (HX711_RIGHT_SDA_READ) { mcu_delay_ms(1); } /* wait until SDA line goes from HIGH to LOW */
        for (uint8_t i = 0; i < 24; i++) {
            HX711_RIGHT_SCL_SET;
            mcu_delay_us(4);
            HX711_RIGHT_SCL_RESET;
            mcu_delay_us(4);
            count = count << 1;
            if (HX711_RIGHT_SDA_READ) {
                count++;
            }
        }
        HX711_RIGHT_SCL_SET;
        mcu_delay_us(4);
        HX711_RIGHT_SCL_RESET;
        mcu_delay_us(4);
        count ^= 0x800000;
        // count &= 0x8fffff; /* clear overflow bit */
        force_right = (fp32)count / 8388607.0f * 500.0f;
    }
}

/**
 * @brief ������get_hx711_data��������force_left���͡�force_right����ֵ���������Ƿֱ�����������fl���͡�fr����
 *
 * @param fl ָ�򸡵������ָ�룬�ñ������洢��������������ݵ�ֵ��
 * @param fr ������fr����һ��ָ�򸡵������ָ�룬�ñ������洢�Ҳ�����������ֵ��
 */
void get_hx711_data(fp32 *fl, fp32 *fr)
{
    *fl = force_left;
    *fr = force_right;
}
