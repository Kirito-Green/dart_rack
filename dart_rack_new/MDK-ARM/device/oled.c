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
 * @brief 函数“oled_write_byte”使用 SPI 通信将一个字节的数据或命令写入 OLED 显示器。
 *
 * @param dat “dat”参数是需要传输到OLED显示屏的数据字节。它可以是表示要发送的数据的任何 8 位值。
 * @param cmd
 * “cmd”参数用于指定发送的数据是用于OLED显示器的命令还是数据。如果“cmd”等于OLED_CMD，则表示该数据是命令。如果“cmd”等于OLED_DATA，则表示该数据是实际的
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
 * @brief 函数“oled_ready”初始化并打开 OLED 显示屏。
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
 * @brief 该函数通过发送一系列命令来配置各种设置来初始化 OLED 显示屏。
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
 * @brief 函数“oled_display_on”通过向显示器发送特定命令来打开 OLED 显示器。
 */
void oled_display_on(void)
{
    oled_write_byte(0x8d, OLED_CMD);
    oled_write_byte(0x14, OLED_CMD);
    oled_write_byte(0xaf, OLED_CMD);
}

/**
 * @brief 函数“oled_display_off”关闭 OLED 显示屏。
 */
void oled_display_off(void)
{
    oled_write_byte(0x8d, OLED_CMD);
    oled_write_byte(0x10, OLED_CMD);
    oled_write_byte(0xae, OLED_CMD);
}

/**
 * @brief 函数“oled_set_pos”设置 OLED 显示屏的位置。
 *
 * @param i 参数“i”代表页地址，它决定了OLED显示屏上的行位置。 OLED显示屏分为多个页面，每个页面由8行组成。
 * @param j 参数“j”代表OLED显示屏上的列地址。它是一个8位值，其中高4位代表低地址，低4位代表高地址。
 */
void oled_set_pos(uint8_t i, uint8_t j)
{
    oled_write_byte((0xb0 + i), OLED_CMD);               // set page address y
    oled_write_byte(((j & 0xf0) >> 4) | 0x10, OLED_CMD); // set column high address
    oled_write_byte((j & 0x0f), OLED_CMD);               // set column low address
}

/**
 * @brief 函数“oled_refresh”通过将 OLED_GRAM 数组的内容写入显示器来刷新 OLED 显示器。
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
 * @brief 函数“oled_operate”通过将像素设置为完全点亮、完全清除或切换其当前状态来对 OLED 显示器进行操作。
 *
 * @param operate “操作”参数是确定要在OLED显示器上执行的操作的输入。它可以采用以下值之一: 点亮 清除 翻转
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
 * @brief 函数“oled_reset”通过将索引变量设置为 0 并清除 OLED_GRAM 数组来重置 OLED 显示。
 */
void oled_reset(void)
{
    idx = 0;
    idy = 0;
    memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
}

/**
 * @brief 显示屏换行显示
 * 如果“idx”变量小于 8，则函数“oled_new_line”会递增“idx”变量，并将“idy”变量设置为 0。
 */
void oled_new_line(void)
{
    if (idx + 1 < 8) {
        idx++;
        idy = 0;
    }
}

/**
 * @brief 函数“oled_enter_string”接收一串字符并将每个字符转换为相应的值以显示在OLED屏幕上。
 *
 * @param pdata 指向 uint8_t 数据数组的指针，其中包含要在 OLED 显示屏上输入的字符串。
 * @param size 参数“size”代表“pdata”数组的大小，即要在OLED屏幕上显示的字符串的长度。
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
 * @brief 函数“oled_enter_one”将字符写入 OLED 显示屏上的特定位置。
 *
 * @param n 参数“n”表示OLED屏幕上要显示的字符或符号的索引。它用于从预定义的数组中选择适当的字符或符号。
 * @param type 函数“oled_enter_one”中的“type”参数用于指定OLED屏幕上显示的字符类型。它可以采用以下值之一：
 * 数字、小写字母、大写字母、特殊字符
 * @return 什么都没有（无效）。
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
