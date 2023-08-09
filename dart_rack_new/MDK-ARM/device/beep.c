#include "beep.h"
#include "tim.h"
#include "math.h"
#include "beep_lib.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "launch_thread.h"

int16_t compare;
int16_t period;
DartRackStateMachine_e state;
extern TIM_HandleTypeDef htim12;

void beep_last(uint16_t last);
void beep_pause(int16_t pause);
void beep_times(uint16_t fre, uint8_t volume, uint8_t times, uint16_t delay_time);
void beep_music_last(uint8_t mark, uint16_t music_beat_time);
void beep_music_pause(uint8_t mark, uint16_t music_pause_time);
void beep_play_tone(uint16_t tune, uint16_t last, uint16_t pause, uint8_t volume);
void beep_play_music(const uint8_t *music_tune,
                     const uint8_t *music_last,
                     const uint8_t *music_pause,
                     uint16_t beat_time,
                     uint16_t pause_time,
                     uint16_t length,
                     uint16_t volume,
                     uint8_t tune_up);

/**
 * @brief ���� beep_init ��ʼ����ʱ�� htim12��
 */
void beep_init(void)
{
    HAL_TIM_Base_Start(&htim12);
}

/**
 * @brief ������beep_start���ڶ�ʱ�� 12 ��ͨ�� 1 ������ PWM �źš�
 */
void beep_start(void)
{
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
}

/**
 * @brief ������beep_stop��ֹͣ��ʱ�� 12 ͨ�� 1 �ϵ� PWM �źš�
 */
void beep_stop(void)
{
    HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
}

/**
 * @brief ������beep_set_volume�����ݸ���������ֵ���÷�������������
 *
 * @param volume ����������һ���޷��� 16 λ��������ʾ�������������
 */
void beep_set_volume(uint8_t volume)
{
    compare = roundf(volume / 100.0f * period);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, volume);
}

/**
 * @brief �ù������÷�������Ƶ�ʡ�
 *
 * @param frequency Ƶ�ʲ���������ҪΪ���������õ�����Ƶ�ʣ��Ժ��� (Hz) Ϊ��λ��
 */
void beep_set_frequency(uint16_t frequency)
{
    period = roundf(1000000.0f / frequency);
    __HAL_TIM_SET_AUTORELOAD(&htim12, period);
}

/**
 * @brief ������beep_times����ָ����Ƶ�ʡ�������ÿ�η���֮����ӳ�ʱ�䲥��ָ�������ķ�������
 *
 * @param fre ��fre�����������������Ƶ�ʡ���������Ϊ uint16_t������ζ�������Ա���� 0 �� 65535 ��ֵ��Ƶ�ʾ����˷����������ߡ�
 * @param volume
 * ��������������ʾ��������������С������������uint8_t�������Ա����0��100��ֵ��ֵԽ�ߣ�����Խ�󣬶�ֵԽ�ͣ�����ԽС��
 * @param times ��times������ָ��Ӧ���ŷ������Ĵ�����
 * @param delay_time Delay_time ������ÿ�η�����֮��ȴ���ʱ����(�Ժ���Ϊ��λ)��
 */
void beep_times(uint16_t fre, uint8_t volume, uint8_t times, uint16_t delay_time)
{
    for (uint8_t i = 0; i < times; i++) {
        beep_start();
        beep_set_frequency(fre);
        beep_set_volume(volume);
        osDelay(delay_time);
        beep_stop();
        osDelay(delay_time);
    }
}

/**
 * @brief ����"beep_ready"���Ʒ������ض���Ƶ�ʣ�������������������ʾ��׼����
 *
 */
void beep_ready(void)
{
    beep_times(BEEP_READY_TUNE, BEEP_READY_VOLUME, BEEP_READY_TIMES, BEEP_DELAY_TIME);
}

/**
 * @brief ����"beep_ready"���Ʒ������ض���Ƶ�ʣ�������������������ʾ��������
 *
 */
void beep_fine(void)
{
    beep_times(BEEP_FINE_TUNE, BEEP_FINE_VOLUME, BEEP_FINE_TIMES, BEEP_DELAY_TIME);
}

/**
 * @brief ����"beep_ready"���Ʒ������ض���Ƶ�ʣ�������������������ʾ�����쳣
 *
 */
void beep_alarm(void)
{
    beep_times(BEEP_ALARM_TUNE, BEEP_ALARM_VOLUME, BEEP_ALARM_TIMES, BEEP_DELAY_TIME);
}

/**
 * @brief ������beep_last����������ִ���ӳ�ָ����ʱ�䡣
 *
 * @param last ������last��������Ϊuint16_t�������޷���16λ������������ָ���ӳٵĳ���ʱ��(�Ժ���Ϊ��λ)��
 */
void beep_last(uint16_t last)
{
    osDelay(last);
}

/**
 * @brief ������beep_pause��ֹͣ����������������ָͣ���ĳ���ʱ�䡣
 *
 * @param pause ������pause��������Ϊint16_t������һ���з��ŵ�16λ����������ʾ��ͣ�ĳ���ʱ��(�Ժ���Ϊ��λ)��
 */
void beep_pause(int16_t pause)
{
    beep_stop();
    osDelay(pause);
}

/**
 * @brief �ú������ݸ����ı�Ǻ����ֽ���ʱ���ӳٳ����ִ��һ����ʱ�䡣
 *
 * @param mark ��mark��������һ�� uint8_t ��������ʾ��������ĵĳ���ʱ�䡣��������ĳ���ʱ�䣬�����Բ��ò�ͬ��ֵ��
 * @param music_beat_time music_beat_time ������1/4���ĵĳ���ʱ��(�Ժ���Ϊ��λ)��
 */
void beep_music_last(uint8_t mark, uint16_t music_beat_time)
{
    if (mark == TRIPLET_TIME_FLAG) {
        osDelay(roundf(music_beat_time * 2.0f / 3.0f));
    } else {
        osDelay(music_beat_time * mark);
    }
}

/**
 * @brief ��������ض�������������beep_music_pause����ֹͣ����������ͣ����ָ����ʱ�䡣
 *
 * @param mark ��mark��������һ������ֵ(0��1)��ָʾ�����Ƿ�Ӧ����ͣ�������mark��Ϊ1�����ʾ����Ӧ����ͣ�����Ϊ0�������ֲ�����ͣ��
 * @param music_pause_time music_pause_time����������Ϊuint16_t������ζ������һ���޷���16λ����������ʾ��ͣ�ĳ���ʱ��(�Ժ���Ϊ��λ)��
 */
void beep_music_pause(uint8_t mark, uint16_t music_pause_time)
{
    beep_stop();
    if (mark && music_pause_time) {
        osDelay(music_pause_time);
    }
}

/**
 * @brief ������beep_play_tone����ָ����Ƶ�ʡ�����ʱ�䡢��ͣ����������������
 *
 * @param tune ����������Ҫ���ŵ�������Ƶ�ʡ���������Ϊ uint16_t������ζ�������Ա��� 0 �� 65535 ֮���ֵ��Ƶ��ͨ���Ժ��� (Hz) Ϊ��λָ����
 * @param last ����󡱲����������ĳ���ʱ��(�Ժ���Ϊ��λ)��������ֹ֮ͣǰ���������Ŷ೤ʱ�䡣
 * @param pause ������beep_play_tone���еġ�pause��������ʾÿ������֮�����ͣ����ʱ��(�Ժ���Ϊ��λ)��
 * @param volume ��������ȷ�����ڲ��ŵ�������������������һ�� 8 λ�޷�������������ֵ������ 0 �� 100 ֮�䡣ֵԽ�ߣ�����Խ�󣬶�ֵԽ�ͣ�����ԽС��
 */
void beep_play_tone(uint16_t tune, uint16_t last, uint16_t pause, uint8_t volume)
{
    beep_start();
    beep_set_volume(volume);
    beep_set_frequency(tune);
    beep_last(last);
    beep_pause(pause);
}

/**
 * @brief ������beep_play_music��ʹ�þ���ָ������(�������ʱ�䡢��ͣʱ�䡢���ȡ������͵���)�ķ���������������������
 *
 * @param music_tune ��ʾ���������������� uint8_t ֵ���顣ÿ��ֵ��Ӧ���������ض�Ƶ�ʡ�
 * @param music_last ��music_last��������һ�� uint8_t
 * ֵ�����飬��ʾ����������ÿ�������ĳ���ʱ�䡣�����е�ÿ��ֵ��Ӧ�ڡ�music_tune�������е�һ������������ʱ���������Ӧ�����Ľ�������ָ����
 * @param music_pause ������music_pause����ָ�� uint8_t ֵ�����ָ�롣����������������ÿ������֮���ͣ�ٳ���ʱ�䡣�����е�ÿ��Ԫ�ض�Ӧ���������������е�һ��������
 * @param beat_time beat_time ������ÿ��1/4���ĵĳ���ʱ��(�Ժ���Ϊ��λ)��
 * @param pause_time ������pause_time����ÿ������֮�����ͣ����ʱ��(�Ժ���Ϊ��λ)��
 * @param length length ������ʾ music_tune��music_last �� music_pause �����е�Ԫ��������������ѭ���������Ͳ������ֵĴ�����
 * @param volume �����������������ڲ��ŵ����ֵ������������� 0 �� 100 ֮���ֵ������ 0 �����������(����)��100 �������������
 * @param tune_up `tune_up`
 * �������ڵ����������������ߡ�������ӵ���music_tune������������У���ѡ��ÿ�����������Ƶ�ʡ�ͨ�����ġ�tune_up����ֵ�����������ϻ������Ƶ���������
 */
void beep_play_music(const uint8_t *music_tune,
                     const uint8_t *music_last,
                     const uint8_t *music_pause,
                     uint16_t beat_time,
                     uint16_t pause_time,
                     uint16_t length,
                     uint16_t volume,
                     uint8_t tune_up)
{
    for (int i = 0; i < length; i++) {
        beep_start();
        beep_set_volume(volume);
        beep_set_frequency(MUSIC_TUNE[music_tune[i] + tune_up]);
        beep_music_last(music_last[i], beat_time);
        beep_music_pause(music_pause[i], pause_time);
    }
}

/**
 * @brief �ú���ʹ�� beep_play_tone ������������
 */
void beep_play_music_steps(void)
{
    for (uint8_t i = 0; i < MUSIC_TUNE_NUMBER; i++) {
        beep_play_tone(MUSIC_TUNE[i], 400, 100, 30);
    }
}

/**
 * @brief ������beep_play_beautiously_you_laugh��ʹ�÷�����������������
 */
void beep_play_seven_feets_soundly(void)
{
    beep_play_music(MUSIC_SEVEN_FEETS_SOUNDLY_TUNE,
                    MUSIC_SEVEN_FEETS_SOUNDLY_LAST,
                    MUSIC_SEVEN_FEETS_SOUNDLY_PAUSE,
                    SEVEN_FEETS_SOUNDLY_BEAT_TIME,
                    SEVEN_FEETS_SOUNDLY_PAUSE_TIME,
                    SEVEN_FEETS_SOUNDLY_SONG_LENGTH,
                    SEVEN_FEETS_SOUNDLY_SONG_VOLUME,
                    SEVEN_FEETS_SOUNDLY_TUNE_UP);
}

/**
 * @brief ������beep_play_beautiously_you_laugh��ʹ�÷�����������Ц������ÿ���
 */
void beep_play_beautifully_you_laugh(void)
{
    beep_play_music(MUSIC_BEAUTIFULLY_YOU_LAUGH_TUNE,
                    MUSIC_BEAUTIFULLY_YOU_LAUGH_LAST,
                    MUSIC_BEAUTIFULLY_YOU_LAUGH_PAUSE,
                    BEAUTIFULLY_YOU_LAUGH_BEAT_TIME,
                    BEAUTIFULLY_YOU_LAUGH_PAUSE_TIME,
                    BEAUTIFULLY_YOU_LAUGH_SONG_LENGTH,
                    BEAUTIFULLY_YOU_LAUGH_SONG_VOLUME,
                    BEAUTIFULLY_YOU_LAUGH_TUNE_UP);
}
