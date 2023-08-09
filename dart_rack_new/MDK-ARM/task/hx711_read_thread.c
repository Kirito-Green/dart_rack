#include "hx711_read_thread.h"
#include "hx711.h"
#include "user_lib.h"
#include "cmsis_os.h"

/**
 * @brief ����hx711_read_thread������ȡHX711����������ֵ
 *
 * @param argument ��argument��������ָ��Ҫ���ݸ��̵߳��κθ������ݵ�ָ�롣�����������߳��ṩ�����Ļ�������Ϣ������������£���δ��ʹ�ò�����Ϊ NULL��
 */
void hx711_read_thread(void const *argument)
{
    for (;;) {
        hx711_read();
        osDelay(100);
    }
}
