#include "user_lib.h"
#include "tim.h"
// #include "stm32f4xx_hal_tim.h"

/**
 * ������get_max��������������������֮������ֵ��
 *
 * @param v1 ��һ������ v1 �ǵ����ȸ����� (fp32)��
 * @param v2 ����v2�ǵ����ȸ�������fp32����
 *
 * @return v1 �� v2 ֮������ֵ��
 */
fp32 get_max(fp32 v1, fp32 v2)
{
    return v1 > v2 ? v1 : v2;
}

/**
 * @brief ������is_zero����鸡��ֵ�Ƿ���һ���ݲ��ڽӽ��㡣
 *
 * @param val ������val��������Ϊ�����ȸ�������
 *
 * @return ����ֵ (bool_t)��ָʾ����ֵ (val) �Ƿ���Ϊ�㡣
 */
bool_t is_zero(fp32 val)
{
    return fabsf(val) < (fp32)1e-4 ? 1 : 0;
}

/**
 * @brief �жϸ���������
 *
 * @param val ������val��������Ϊ32 λ���ȵĸ��������Զ����������͡�
 *
 * @return int16_t ֵ�������з��ŵ� 16 λ�������������ֵС��-1e-4����ú�������-1���������ֵ����1e-4���򷵻�1�����򷵻�0��
 */
int16_t sign(fp32 val)
{
    if (val < -1e-4)
        return -1;
    if (val > 1e-4)
        return 1;
    return 0;
}

/**
 * @brief ��Ծ����
 *
 * @param val ������val��������Ϊfp32��ͨ������32λ������������ʾ��Ҫ�롰JudgePoint���������бȽϵ�ֵ��
 * @param JudgePoint JudgePoint ������һ������ֵ����ʾ��ֵ�� step_function ������ val ������ JudgePoint ֵ���бȽϣ���� val С�ڻ����
 * JudgePoint���򷵻� 0�����򷵻� 1��
 *
 * @return int16_t ֵ��
 */
int16_t step_function(fp32 val, fp32 JudgePoint)
{
    return val <= JudgePoint ? 0 : 1;
}

fp32 choose_shortest_path(fp32 set, fp32 ref, fp32 val_range)
{
    fp32 front_dis = set - ref;
    int16_t s      = sign(front_dis);
    if (s != 0) {
        fp32 back_dis = (val_range - fabsf(set - ref)) * (-s);
        return fabsf(front_dis) < fabsf(back_dis) ? front_dis : back_dis;
    }
    return front_dis;
}

/**
 * @brief �����Դ�����
 *
 * @param error ���ڴ�������
 * @param alpha ��ָ������ָ��С��1ʱ��������С���С�����
 * ��ָ������1�����С����С ���������
 * @param delta ��ֵԽ���ֹƵ�ʱ�󣬴����������ٶȱ��
 * ��ֵԽС��ֹƵ�ʱ�С������խ�������ٶȱ��
 *
 * @return fp32ֵ
 */
fp32 fal(fp32 error, fp32 alpha, fp32 delta)
{
    return fabsf(error) > delta ? pow(fabsf(error), alpha) * sign(error) : error / pow(delta, 1 - alpha);
}

/**
 * @brief �ú���������ֵ�Ƿ���ָ����Χ�ڡ�
 *
 * @param val ��Ҫ����ֵ�Ƿ��ڷ�Χ�ڡ�
 * @param min ����ֵӦ�ڷ�Χ�ڵ���Сֵ��
 * @param max ��max�������ǡ�val���������Ծ��е����ֵ���Ա㺯������ true��
 *
 * @return ����ֵ��1 �� 0��
 */
bool_t in_range(fp32 val, fp32 min, fp32 max)
{
    if (val >= min && val <= max)
        return 1;
    return 0;
}

/**
 * @brief ������get_close��������ֵ�Ƿ���Ŀ��ֵ��ָ����Χ�ڡ�
 *
 * @param val ��Ҫ����Ƿ�ӽ�Ŀ��ֵ��ֵ��
 * @param target ����Ҫ�����ֵ��val�����бȽϵ�Ŀ��ֵ��
 * @param range range ������ʾ�ɽ��ܵķ�Χ���ڸ÷�Χ�ڣ�val ��Ŀ��ֵ֮��Ĳ��챻��Ϊ�ǽӽ��ġ�
 *
 * @return һ������ֵ�������val���͡�target��֮��ľ��Բ�С�ڡ�range�����򷵻� 1�����򷵻� 0��
 */
bool_t get_close(fp32 val, fp32 target, fp32 range)
{
    if (fabsf(val - target) < range) {
        return 1;
    }
    return 0;
}

/**
 * @brief ���� int_pow ͨ��������ֵ�ݹ�����������ָ���������������ֵ���ݡ�
 *
 * @param val �����ݵĻ�ֵ��
 * @param times ��times������������Ϊuint8_t������ζ������һ���޷���8λ����������ʾ��ֵӦ����������˵Ĵ�����
 *
 * @return ��ֵ��val����ߵ���times���η��Ľ����
 */
int16_t int_pow(int16_t val, uint8_t times)
{
    if (times == 0)
        return 1;
    return val * int_pow(val, times - 1);
}

static uint8_t rx_buf[20];
static uint8_t rx_len;
/**
 * @brief ������fp32_to_string����������ת��Ϊ����ָ��С��λ�����ַ�����ʾ��ʽ��
 *
 * @param val ��val��������һ��Ҫת��Ϊ�ַ����ĸ�������
 * @param digit ������digit����ʾת������ַ����а�����С��λ����
 * @param target_rx_buf ָ�򽫴洢����ַ����Ļ�������ָ�롣
 * @param target_rx_len ������target_rx_len����ָ��uint8_t��������ָ�룬�ñ������洢����ַ����ĳ��ȡ�
 */
void fp32_to_string(fp32 val, uint8_t digit, uint8_t *target_rx_buf, uint8_t *target_rx_len)
{
    uint16_t temp_val = roundf(val * int_pow(10, digit));
    rx_len            = 0;
    /* int to string */
    while (temp_val) {
        rx_buf[rx_len] = (uint8_t)(temp_val % 10);
        temp_val /= 10;
        rx_len++;
    }
    /* reverse */
    uint8_t temp_save;
    for (uint8_t i = 0; i < rx_len / 2; i++) {
        temp_save              = rx_buf[i];
        rx_buf[i]              = rx_buf[rx_len - i - 1];
        rx_buf[rx_len - i - 1] = temp_save;
    }
    memcpy(&rx_buf[rx_len - digit + 1], &rx_buf[rx_len - digit], digit);
    rx_buf[rx_len - digit] = (uint8_t)'.';
    rx_len++;

    /* ������ */
    memcpy(target_rx_buf, &rx_buf, rx_len + 1);
    memcpy(target_rx_len, &rx_len, 1);
}

/**
 * @brief ������systick_delay_us������ʹ�� SysTick ��ʱ������΢�뼶���ӳ١�
 *
 * @param us ������us����ʾ��΢��Ϊ��λ���ӳ�ʱ�䡣
 */
void systick_delay_us(uint16_t us)
{
    uint32_t temp;
    SysTick->LOAD = SYSTICK_PERIOID * us;
    SysTick->VAL  = 0x00; // clear timer
    SysTick->CTRL = 0x01; // enable
    do {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && (!(temp & (1 << 16))));

    SysTick->VAL  = 0x00;
    SysTick->CTRL = 0x00;
}

/**
 * @brief ���� systick_delay_ms ����ʹ�� SysTick ��ʱ�������Ժ���Ϊ��λ���ӳ١�
 *
 * @param ms ������ms����ʾ�ӳٵĺ�������
 */
void systick_delay_ms(uint32_t ms)
{
    uint32_t temp;
    SysTick->LOAD = 1000 * SYSTICK_PERIOID * ms;
    SysTick->VAL  = 0x00; // clear timer
    SysTick->CTRL = 0x01; // enable
    do {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && (!(temp & (1 << 16))));

    SysTick->VAL  = 0x00;
    SysTick->CTRL = 0x00;
}

/**
 * @brief ���� htim_delay_us ����ʹ�� TIM3 ��ʱ������΢�뼶���ӳ١�
 *
 * @param us ������us����ʾ�ӳٵ�΢������
 */
void htim_delay_us(uint16_t us)
{

    htim3.Instance->CNT = us - 1;                            // �������װҪ�ݼ�����������0��ᴥ����ʱ����TIM_FLAG_UpDate��־λ
    htim3.Instance->CR1 |= TIM_CR1_CEN;                      // ʹ�ܼ������� ��������ʼ�ݼ�
    while ((htim3.Instance->SR & TIM_FLAG_UPDATE) != SET) {} // �ȵ�����������0
    htim3.Instance->CR1 &= (~TIM_CR1_CEN);                   // �رռ�����
    htim3.Instance->SR &= ~TIM_FLAG_UPDATE;                  // �����ʱ����Ϊ0�ı�־λ
}

/**
 * @brief ���� htim_delay_ms ������ִ���ӳ�ָ���ĺ�������
 *
 * @param ms ������ms��������Ϊuint32_t�������޷���32λ����������ʾ�ӳٵĺ�������
 */
void htim_delay_ms(uint32_t ms)
{
    htim_delay_us(1000 * ms);
}

/**
 * @brief ������mcu_delay_us������ͨ��ִ��һϵ���޲���ָ��������΢�뼶���ӳ١�
 *
 * @param us ������us����ʾ�ӳٵ�΢������
 */
void mcu_delay_us(uint16_t us)
{
    for (uint16_t i = 0; i < us; i++) {
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
    }
}

/**
 * @brief ������mcu_delay_ms����һ���ӳٺ�����ͨ����ε�����һ��������mcu_delay_us�����ȴ�ָ���ĺ�������
 *
 * @param ms ������ms��������Ϊuint16_t�������޷���16λ����������ʾӦִ���ӳٵĺ�������
 */
void mcu_delay_ms(uint16_t ms)
{
    for (uint16_t i = 0; i < 1000; i++) {
        mcu_delay_us(ms);
    }
}
