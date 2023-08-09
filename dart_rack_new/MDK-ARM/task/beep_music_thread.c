#include "beep_music_thread.h"
#include "launch_thread.h"
#include "cmsis_os.h"

static DartRackStateMachine_e state;

/**
 * @brief  ������beep_music_thread���ڷ��������״̬�²�������
 *
 * @param argument ��argument��������ָ��Ҫ���ݸ��̵߳��κθ������ݵ�ָ�롣�����������߳��ṩ�����Ļ�������Ϣ���ڱ����У���������Ϊ��void const
 * *��������ζ������ָ���� void ���͵�ָ�롣
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