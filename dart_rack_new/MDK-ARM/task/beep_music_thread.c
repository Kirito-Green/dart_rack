#include "beep_music_thread.h"
#include "launch_thread.h"
#include "cmsis_os.h"

static DartRackStateMachine_e state;

/**
 * @brief  函数“beep_music_thread”在发射架无力状态下播放音乐
 *
 * @param argument “argument”参数是指向要传递给线程的任何附加数据的指针。它可用于向线程提供上下文或配置信息。在本例中，它的类型为“void const
 * *”，这意味着它是指向常量 void 类型的指针。
 */
void beep_music_thread(void const *argument)
{
    for (;;) {
        get_state(&state);
        switch (state) {
            case DART_RACK_NO_FORCE:

                break;
            default:
                break;
        }
        osDelay(400);
    }
}