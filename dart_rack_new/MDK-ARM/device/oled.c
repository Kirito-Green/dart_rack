#include "oled.h"
#include "main.h"
#include "spi.h"
#include "launch_thread.h"
#include "oled_lib.h"
#include "string.h"
#include "ctype.h"
#include "cmsis_os.h"
#include "user_lib.h"

extern SPI_HandleTypeDef hspi1;

static uint8_t OLED_GRAM[8][128];
static uint8_t idx, idy;
static DartRackEncoder_t encoder;

/**
 * @brief ������oled_write_byte��ʹ�� SPI ͨ�Ž�һ���ֽڵ����ݻ�����д�� OLED ��ʾ����
 *
 * @param dat ��dat����������Ҫ���䵽OLED��ʾ���������ֽڡ��������Ǳ�ʾҪ���͵����ݵ��κ� 8 λֵ��
 * @param cmd
 * ��cmd����������ָ�����͵�����������OLED��ʾ������������ݡ������cmd������OLED_CMD�����ʾ����������������cmd������OLED_DATA�����ʾ��������ʵ�ʵ�
 */
void oled_write_byte(uint8_t dat, uint8_t cmd)
{
    if (cmd == OLED_CMD)
        OLED_DC_SET_CMD;
    if (cmd == OLED_DATA)
        OLED_DC_SET_DATA;
    mcu_delay_us(5);
    HAL_SPI_Transmit(&hspi1, &dat, 1, 10);
}

/**
 * @brief ������oled_ready����ʼ������ OLED ��ʾ����
 */
void oled_ready(void)
{
    OLED_RST_RESET;
    mcu_delay_us(5);
    OLED_RST_SET;
    mcu_delay_us(5);
    oled_init();
    oled_display_on();
    oled_operate(OLED_CLEAR);
}

/**
 * @brief �ú���ͨ������һϵ�����������ø�����������ʼ�� OLED ��ʾ����
 */
void oled_init(void)
{
    oled_write_byte(0xAE, OLED_CMD); // display off
    oled_write_byte(0x20, OLED_CMD); // Set Memory Addressing Mode
    oled_write_byte(0x10, OLED_CMD); // 00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
    oled_write_byte(0xb0, OLED_CMD); // Set Page Start Address for Page Addressing Mode,0-7
    oled_write_byte(0xc8, OLED_CMD); // Set COM Output Scan Direction
    oled_write_byte(0x00, OLED_CMD); //---set low column address
    oled_write_byte(0x10, OLED_CMD); //---set high column address
    oled_write_byte(0x40, OLED_CMD); //--set start line address
    oled_write_byte(0x81, OLED_CMD); //--set contrast control register
    oled_write_byte(0xff, OLED_CMD); // brightness 0x00~0xff
    oled_write_byte(0xa1, OLED_CMD); //--set segment re-map 0 to 127
    oled_write_byte(0xa6, OLED_CMD); //--set normal display
    oled_write_byte(0xa8, OLED_CMD); //--set multiplex ratio(1 to 64)
    oled_write_byte(0x3F, OLED_CMD); //
    oled_write_byte(0xa4, OLED_CMD); // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content
    oled_write_byte(0xd3, OLED_CMD); //-set display offset
    oled_write_byte(0x00, OLED_CMD); //-not offset
    oled_write_byte(0xd5, OLED_CMD); //--set display clock divide ratio/oscillator frequency
    oled_write_byte(0xf0, OLED_CMD); //--set divide ratio
    oled_write_byte(0xd9, OLED_CMD); //--set pre-charge period
    oled_write_byte(0x22, OLED_CMD); //
    oled_write_byte(0xda, OLED_CMD); //--set com pins hardware configuration
    oled_write_byte(0x12, OLED_CMD);
    oled_write_byte(0xdb, OLED_CMD); //--set vcomh
    oled_write_byte(0x20, OLED_CMD); // 0x20,0.77xVcc
    oled_write_byte(0x8d, OLED_CMD); //--set DC-DC enable
    oled_write_byte(0x14, OLED_CMD); //
    oled_write_byte(0xaf, OLED_CMD); //--turn on oled panel
}

/**
 * @brief ������oled_display_on��ͨ������ʾ�������ض��������� OLED ��ʾ����
 */
void oled_display_on(void)
{
    oled_write_byte(0x8d, OLED_CMD);
    oled_write_byte(0x14, OLED_CMD);
    oled_write_byte(0xaf, OLED_CMD);
}

/**
 * @brief ������oled_display_off���ر� OLED ��ʾ����
 */
void oled_display_off(void)
{
    oled_write_byte(0x8d, OLED_CMD);
    oled_write_byte(0x10, OLED_CMD);
    oled_write_byte(0xae, OLED_CMD);
}

/**
 * @brief ������oled_set_pos������ OLED ��ʾ����λ�á�
 *
 * @param i ������i������ҳ��ַ����������OLED��ʾ���ϵ���λ�á� OLED��ʾ����Ϊ���ҳ�棬ÿ��ҳ����8����ɡ�
 * @param j ������j������OLED��ʾ���ϵ��е�ַ������һ��8λֵ�����и�4λ����͵�ַ����4λ����ߵ�ַ��
 */
void oled_set_pos(uint8_t i, uint8_t j)
{
    oled_write_byte((0xb0 + i), OLED_CMD);               // set page address y
    oled_write_byte(((j & 0xf0) >> 4) | 0x10, OLED_CMD); // set column high address
    oled_write_byte((j & 0x0f), OLED_CMD);               // set column low address
}

/**
 * @brief ������oled_refresh��ͨ���� OLED_GRAM ���������д����ʾ����ˢ�� OLED ��ʾ����
 */
void oled_refresh(void)
{
    for (uint8_t i = 0; i < 8; i++) {
        oled_set_pos(i, 0);
        for (uint8_t j = 0; j < 128; j++) {
            oled_write_byte(OLED_GRAM[i][j], OLED_DATA);
        }
    }
}

/**
 * @brief ������oled_operate��ͨ������������Ϊ��ȫ��������ȫ������л��䵱ǰ״̬���� OLED ��ʾ�����в�����
 *
 * @param operate ��������������ȷ��Ҫ��OLED��ʾ����ִ�еĲ��������롣�����Բ�������ֵ֮һ: ���� ��� ��ת
 */
void oled_operate(uint8_t operate)
{
    for (uint8_t i = 0; i < 8; i++) {
        for (uint8_t j = 0; j < 128; j++) {
            if (operate == OLED_LIGHT) {
                OLED_GRAM[i][j] = 0xff;
            }
            if (operate == OLED_CLEAR) {
                OLED_GRAM[i][j] = 0x00;
            }
            if (operate == OLED_TOGGLE) {
                OLED_GRAM[i][j] = 0xff - OLED_GRAM[i][j];
            }
        }
    }
    oled_refresh();
}

/**
 * @brief ������oled_reset��ͨ����������������Ϊ 0 ����� OLED_GRAM ���������� OLED ��ʾ��
 */
void oled_reset(void)
{
    idx = 0;
    idy = 0;
    memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
}

/**
 * @brief ��ʾ��������ʾ
 * �����idx������С�� 8��������oled_new_line���������idx��������������idy����������Ϊ 0��
 */
void oled_new_line(void)
{
    if (idx + 1 < 8) {
        idx++;
        idy = 0;
    }
}

/**
 * @brief ������oled_enter_string������һ���ַ�����ÿ���ַ�ת��Ϊ��Ӧ��ֵ����ʾ��OLED��Ļ�ϡ�
 *
 * @param pdata ָ�� uint8_t ���������ָ�룬���а���Ҫ�� OLED ��ʾ����������ַ�����
 * @param size ������size������pdata������Ĵ�С����Ҫ��OLED��Ļ����ʾ���ַ����ĳ��ȡ�
 */
void oled_enter_string(uint8_t *pdata, uint8_t size)
{
    uint8_t n;
    for (uint8_t i = 0; i < size - 1 && idy < 16; i++, idy++) {
        if (isdigit(pdata[i])) {
            n = (uint8_t)pdata[i] - (uint8_t)'0';
            oled_enter_one(n, NUMBER);
        } else if (pdata[i] >= 0 && pdata[i] <= 9) {
            oled_enter_one(pdata[i], NUMBER);
        }

        else if (islower(pdata[i])) {
            n = (uint8_t)pdata[i] - (uint8_t)'a';
            oled_enter_one(n, LOWER);
        } else if (isupper(pdata[i])) {
            n = (uint8_t)pdata[i] - (uint8_t)'A';
            oled_enter_one(n, UPPER);
        } else {
            if (pdata[i] == ' ') {
                oled_enter_one(0, SPECIAL);
            }
            if (pdata[i] == '.') {
                oled_enter_one(1, SPECIAL);
            }
            if (pdata[i] == ':') {
                oled_enter_one(2, SPECIAL);
            }
            if (pdata[i] == '-') {
                oled_enter_one(3, SPECIAL);
            }
        }
    }
}

/**
 * @brief ������oled_enter_one�����ַ�д�� OLED ��ʾ���ϵ��ض�λ�á�
 *
 * @param n ������n����ʾOLED��Ļ��Ҫ��ʾ���ַ�����ŵ������������ڴ�Ԥ�����������ѡ���ʵ����ַ�����š�
 * @param type ������oled_enter_one���еġ�type����������ָ��OLED��Ļ����ʾ���ַ����͡������Բ�������ֵ֮һ��
 * ���֡�Сд��ĸ����д��ĸ�������ַ�
 * @return ʲô��û�У���Ч����
 */
void oled_enter_one(uint8_t n, uint8_t type)
{
    /* out */
    if (idx >= 8 || idy >= 16) {
        return;
    }
    for (uint8_t j = 0; j < 8; j++) {
        if (type == NUMBER)
            OLED_GRAM[idx][8 * idy + j] = OLED_NUMBER[n][j];
        if (type == LOWER)
            OLED_GRAM[idx][8 * idy + j] = OLED_ALPHA_LOWER[n][j];
        if (type == UPPER)
            OLED_GRAM[idx][8 * idy + j] = OLED_ALPHA_UPPER[n][j];
        if (type == SPECIAL)
            OLED_GRAM[idx][8 * idy + j] = OLED_SPECIAL[n][j];
    }
}
