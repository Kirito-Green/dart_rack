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
 * @brief 函数 beep_init 初始化定时器 htim12。
 */
void beep_init(void)
{
    HAL_TIM_Base_Start(&htim12);
}

/**
 * @brief 函数“beep_start”在定时器 12 的通道 1 上启动 PWM 信号。
 */
void beep_start(void)
{
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
}

/**
 * @brief 函数“beep_stop”停止定时器 12 通道 1 上的 PWM 信号。
 */
void beep_stop(void)
{
    HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
}

/**
 * @brief 函数“beep_set_volume”根据给定的音量值设置蜂鸣声的音量。
 *
 * @param volume 音量参数是一个无符号 16 位整数，表示所需的音量级别。
 */
void beep_set_volume(uint8_t volume)
{
    compare = roundf(volume / 100.0f * period);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, volume);
}

/**
 * @brief 该功能设置蜂鸣声的频率。
 *
 * @param frequency 频率参数是您想要为蜂鸣声设置的所需频率，以赫兹 (Hz) 为单位。
 */
void beep_set_frequency(uint16_t frequency)
{
    period = roundf(1000000.0f / frequency);
    __HAL_TIM_SET_AUTORELOAD(&htim12, period);
}

/**
 * @brief 函数“beep_times”以指定的频率、音量和每次蜂鸣之间的延迟时间播放指定次数的蜂鸣声。
 *
 * @param fre “fre”参数代表蜂鸣声的频率。它的类型为 uint16_t，这意味着它可以保存从 0 到 65535 的值。频率决定了蜂鸣声的音高。
 * @param volume
 * “音量”参数表示蜂鸣声的音量大小。它的类型是uint8_t，它可以保存从0到100的值。值越高，声音越大，而值越低，声音越小。
 * @param times “times”参数指定应播放蜂鸣声的次数。
 * @param delay_time Delay_time 参数是每次蜂鸣声之间等待的时间量（以毫秒为单位）。
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
 * @brief 函数"beep_ready"控制蜂鸣器特定的频率，音量，次数发声来表示已准备好
 *
 */
void beep_ready(void)
{
    beep_times(BEEP_READY_TUNE, BEEP_READY_VOLUME, BEEP_READY_TIMES, BEEP_DELAY_TIME);
}

/**
 * @brief 函数"beep_ready"控制蜂鸣器特定的频率，音量，次数发声来表示工作正常
 *
 */
void beep_fine(void)
{
    beep_times(BEEP_FINE_TUNE, BEEP_FINE_VOLUME, BEEP_FINE_TIMES, BEEP_DELAY_TIME);
}

/**
 * @brief 函数"beep_ready"控制蜂鸣器特定的频率，音量，次数发声来表示工作异常
 *
 */
void beep_alarm(void)
{
    beep_times(BEEP_ALARM_TUNE, BEEP_ALARM_VOLUME, BEEP_ALARM_TIMES, BEEP_DELAY_TIME);
}

/**
 * @brief 函数“beep_last”将蜂鸣器执行延迟指定的时间。
 *
 * @param last 参数“last”的类型为uint16_t，代表无符号16位整数。它用于指定延迟的持续时间（以毫秒为单位）。
 */
void beep_last(uint16_t last)
{
    osDelay(last);
}

/**
 * @brief 函数“beep_pause”停止蜂鸣声并将程序暂停指定的持续时间。
 *
 * @param pause 参数“pause”的类型为int16_t，它是一个有符号的16位整数。它表示暂停的持续时间（以毫秒为单位）。
 */
void beep_pause(int16_t pause)
{
    beep_stop();
    osDelay(pause);
}

/**
 * @brief 该函数根据给定的标记和音乐节拍时间延迟程序的执行一定的时间。
 *
 * @param mark “mark”参数是一个 uint8_t 变量，表示音符或节拍的持续时间。根据所需的持续时间，它可以采用不同的值。
 * @param music_beat_time music_beat_time 参数是1/4节拍的持续时间（以毫秒为单位）。
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
 * @brief 如果满足特定条件，函数“beep_music_pause”将停止蜂鸣声并暂停音乐指定的时间。
 *
 * @param mark “mark”参数是一个布尔值（0或1），指示音乐是否应该暂停。如果“mark”为1，则表示音乐应该暂停。如果为0，则音乐不会暂停。
 * @param music_pause_time music_pause_time参数的类型为uint16_t，这意味着它是一个无符号16位整数。它表示暂停的持续时间（以毫秒为单位）。
 */
void beep_music_pause(uint8_t mark, uint16_t music_pause_time)
{
    beep_stop();
    if (mark && music_pause_time) {
        osDelay(music_pause_time);
    }
}

/**
 * @brief 函数“beep_play_tone”以指定的频率、持续时间、暂停和音量播放音调。
 *
 * @param tune 音调参数是要播放的音调的频率。它的类型为 uint16_t，这意味着它可以保存 0 到 65535 之间的值。频率通常以赫兹 (Hz) 为单位指定。
 * @param last “最后”参数是音调的持续时间（以毫秒为单位）。它决定停止之前音调将播放多长时间。
 * @param pause 函数“beep_play_tone”中的“pause”参数表示每个音调之间的暂停持续时间（以毫秒为单位）。
 * @param volume 音量参数确定正在播放的音调的音量级别。它是一个 8 位无符号整数，它的值可以在 0 到 100 之间。值越高，声音越大，而值越低，声音越小。
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
 * @brief 函数“beep_play_music”使用具有指定参数（例如节拍时间、暂停时间、长度、音量和调音）的蜂鸣声来播放音乐曲调。
 *
 * @param music_tune 表示音乐曲调的音符的 uint8_t 值数组。每个值对应于音符的特定频率。
 * @param music_last “music_last”参数是一个 uint8_t
 * 值的数组，表示音乐曲调中每个音符的持续时间。数组中的每个值对应于“music_tune”数组中的一个音符。持续时间根据音符应持续的节拍数来指定。
 * @param music_pause 参数“music_pause”是指向 uint8_t 值数组的指针。它代表音乐曲调中每个音符之间的停顿持续时间。数组中的每个元素对应于音乐曲调数组中的一个音符。
 * @param beat_time beat_time 参数是每个1/4节拍的持续时间（以毫秒为单位）。
 * @param pause_time 参数“pause_time”是每个音符之间的暂停持续时间（以毫秒为单位）。
 * @param length length 参数表示 music_tune、music_last 和 music_pause 数组中的元素数量。它决定循环将迭代和播放音乐的次数。
 * @param volume 音量参数决定了正在播放的音乐的音量级别。它是 0 到 100 之间的值，其中 0 代表最低音量（静音），100 代表最高音量。
 * @param tune_up `tune_up`
 * 参数用于调整音乐曲调的音高。它被添加到“music_tune”数组的索引中，以选择每个音符所需的频率。通过更改“tune_up”的值，您可以向上或向下移调音乐曲调
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
 * @brief 该函数使用 beep_play_tone 函数播放音阶
 */
void beep_play_music_steps(void)
{
    for (uint8_t i = 0; i < MUSIC_TUNE_NUMBER; i++) {
        beep_play_tone(MUSIC_TUNE[i], 400, 100, 30);
    }
}

/**
 * @brief 函数“beep_play_beautiously_you_laugh”使用蜂鸣器播放你七里香
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
 * @brief 函数“beep_play_beautiously_you_laugh”使用蜂鸣器播放你笑起来真好看。
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
