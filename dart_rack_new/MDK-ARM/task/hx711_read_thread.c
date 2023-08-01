#include "hx711_read_thread.h"
#include "hx711.h"
#include "user_lib.h"
#include "cmsis_os.h"

/**
 * @brief 函数hx711_read_thread连续读取HX711传感器的数值
 *
 * @param argument “argument”参数是指向要传递给线程的任何附加数据的指针。它可用于向线程提供上下文或配置信息。在这种情况下，它未被使用并设置为 NULL。
 */
void hx711_read_thread(void const *argument)
{
    for (;;) {
        // hx711_read();
        osDelay(100);
    }
}
