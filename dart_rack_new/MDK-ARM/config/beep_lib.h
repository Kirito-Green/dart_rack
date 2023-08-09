#ifndef _BEEP_LIB_H
#define _BEEP_LIB_H

#include "struct_typedef.h"

/* tune */
#define MUSIC_TUNE_NUMBER 43
const uint16_t MUSIC_TUNE[] = {100,
                               261, 294, 330, 349, 392, 440, 494,         // low
                               523, 587, 659, 698, 784, 880, 988,         // medium
                               1047, 1175, 1319, 1397, 1568, 1760, 1976,  // high
                               2093, 2217, 2349, 2489, 2637, 2794, 2960,  // high'
                               3136, 3322, 3520, 3729, 3951, 4186, 4435,  // high''
                               4699, 4978, 5274, 5588, 5920, 6272, 6645}; // high'''

/* -------------------------------------------------------------------------- */
/*                                     七里                                    */
/* -------------------------------------------------------------------------- */
/* 16分音符时 */
#define SEVEN_FEETS_SOUNDLY_SONG_LENGTH 221
#define SEVEN_FEETS_SOUNDLY_SONG_VOLUME 10
#define SEVEN_FEETS_SOUNDLY_BEAT_TIME   208
#define SEVEN_FEETS_SOUNDLY_PAUSE_TIME  50
#define SEVEN_FEETS_SOUNDLY_TUNE_UP     7
#define TRIPLET_TIME_FLAG               0

const uint8_t MUSIC_SEVEN_FEETS_SOUNDLY_TUNE[] = {5, 0, 5,
                                                  8, 7, 8, 8, 8, 8,
                                                  8, 7, 6, 7, 6, 6, 5, 5,
                                                  4, 4, 3, 5, 5, 5,
                                                  5, 5, 2, 2, 4, 4, 3, 3, 0, 5,
                                                  8, 7, 8, 8, 8, 8,
                                                  8, 7, 7, 8, 9, 9, 8, 8, 8, 7,
                                                  8, 8, 8, 8, 7, 7, 6, 6, 6, 7, 6,
                                                  5, 5, 5, 5, 7, 8,
                                                  8, 3, 6, 6, 5, 9,
                                                  9, 2, 4, 4, 4, 3, 5,
                                                  5, 4, 4, 3, 3, 2, 2, 2, 1,
                                                  3, 2, 2, 2, 4, 3, 5, 7, 8,
                                                  8, 3, 5, 6, 6, 5, 5, 9,
                                                  9, 3, 3, 4, 4, 3, 5, 5, 10,
                                                  10, 9, 9, 8, 8, 9, 8, 10,
                                                  10, 9, 9, 9, 0, 5, 7, 8, 8,
                                                  8, 8, 8, 7, 6, 5, 5,
                                                  7, 8, 9, 8, 8, 5, 7, 8, 8,
                                                  8, 8, 8, 7, 6, 5, 5,
                                                  9, 10, 11, 11, 10, 0, 5, 7, 8, 8,
                                                  8, 8, 8, 7, 6, 5, 5,
                                                  9, 10, 11, 10, 10, 10, 5,
                                                  9, 8, 7, 8, 8, 8, 8,
                                                  8, 10, 10, 8, 8, 8, 7, 9, 8, 8,
                                                  8, 0, 5, 7, 8, 8,
                                                  9, 8, 9, 11, 10, 10, 8, 8, 8,
                                                  8, 10, 9, 8, 8, 8, 7, 9, 8, 8,
                                                  8};
/* 16分音符为基准*/
const uint8_t MUSIC_SEVEN_FEETS_SOUNDLY_LAST[] = {12, 2, 2,
                                                  4, 2, 1, 1, 6, 2,
                                                  2, 2, 2, 1, 1, 2, 4, 2,
                                                  4, 2, 1, 1, 6, 2,
                                                  2, 1, 1, 2, 1, 1, 1, 3, 2, 2,
                                                  4, 2, 1, 1, 6, 2,
                                                  2, 2, 2, 2, 2, 2, 2, 3, 2, 2,
                                                  2, 1, 1, 1, 1, 1, 1, 4, 2, 1, 1,
                                                  2, 6, 4, 2, 1, 1,
                                                  4, 2, 2, 4, 2, 2,
                                                  4, 2, 1, 1, 2, 4, 2,
                                                  2, 2, 2, 2, 2, 2, 2, 2, 2,
                                                  1, 1, 2, 2, 2, 4, 2, 1, 1,
                                                  4, 2, 2, 2, 2, 2, 2, 2,
                                                  2, 2, 2, 1, 1, 2, 2, 2, 2,
                                                  2, 2, 2, 2, 2, 2, 2, 2,
                                                  1, 1, 2, 4, 2, 2, 2, 1, 1,
                                                  6, 2, 2, 2, 1, 1, 2,
                                                  2, 2, 3, 1, 2, 2, 2, 1, 1,
                                                  6, 2, 2, 2, 1, 1, 2,
                                                  2, 1, 1, 1, 3, 2, 2, 2, 1, 1,
                                                  6, 2, 2, 2, 1, 1, 2,
                                                  2, 2, 2, 1, 1, 2, 2,
                                                  2, 2, 2, 1, 1, 6, 2,
                                                  2, 2, 1, 1, 2, 2, 2, 2, 1, 1,
                                                  8, 2, 2, 2, 1, 1,
                                                  2, 2, 2, 1, 1, 2, 2, 2, 2,
                                                  2, 2, 1, 1, 2, 2, 2, 2, 1, 1,
                                                  16};

const uint8_t MUSIC_SEVEN_FEETS_SOUNDLY_PAUSE[] = {1, 1, 1,
                                                   1, 1, 1, 0, 1, 1,
                                                   1, 1, 1, 1, 0, 1, 1, 1,
                                                   1, 1, 1, 0, 1, 1,
                                                   1, 0, 1, 1, 1, 0, 1, 1, 1, 1,
                                                   1, 1, 1, 0, 1, 1,
                                                   0, 1, 1, 1, 1, 0, 1, 1, 1, 1,
                                                   1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0,
                                                   1, 0, 1, 1, 0, 1,
                                                   1, 1, 1, 1, 1, 1,
                                                   1, 1, 1, 0, 1, 1, 1,
                                                   1, 1, 1, 1, 1, 1, 0, 1, 1,
                                                   0, 1, 0, 1, 1, 1, 1, 0, 1,
                                                   1, 1, 0, 1, 1, 1, 1, 1,
                                                   1, 1, 1, 1, 0, 1, 1, 1, 1,
                                                   1, 1, 1, 1, 1, 1, 1, 1,
                                                   0, 1, 0, 1, 1, 1, 1, 1, 0,
                                                   1, 1, 1, 1, 0, 1, 1,
                                                   1, 1, 1, 0, 1, 1, 1, 1, 0,
                                                   1, 1, 1, 1, 0, 1, 1,
                                                   1, 1, 0, 1, 1, 1, 1, 1, 1, 0,
                                                   1, 1, 1, 1, 0, 1, 1,
                                                   1, 1, 1, 1, 0, 1, 1,
                                                   1, 1, 1, 1, 0, 1, 1,
                                                   1, 0, 1, 1, 1, 1, 1, 1, 1, 0,
                                                   1, 1, 1, 1, 1, 0,
                                                   1, 1, 1, 1, 0, 1, 0, 1, 1,
                                                   1, 1, 0, 1, 1, 1, 1, 1, 1, 0,
                                                   1};

/* -------------------------------------------------------------------------- */
/*                                   你笑起来真好                            */
/* -------------------------------------------------------------------------- */
#define BEAUTIFULLY_YOU_LAUGH_SONG_LENGTH 121
#define BEAUTIFULLY_YOU_LAUGH_SONG_VOLUME 30
#define BEAUTIFULLY_YOU_LAUGH_BEAT_TIME   100
#define BEAUTIFULLY_YOU_LAUGH_PAUSE_TIME  50
#define BEAUTIFULLY_YOU_LAUGH_TUNE_UP     0

uint8_t MUSIC_BEAUTIFULLY_YOU_LAUGH_TUNE[] = {
    5, 10, 10, 5, 5, 9, 9, 16, 8, 8, 8, 9, 10, 5, 5, 16,  // 想去远方的山川，想去海边看海
    6, 8, 8, 6, 5, 10, 10, 16, 9, 8, 8, 6, 9, 16,         // 不管风雨有多少，有你就足
    5, 10, 10, 5, 5, 9, 9, 16, 8, 8, 8, 6, 5, 10, 10, 16, // 喜欢看你的嘴角，喜欢看你的眉
    6, 11, 11, 6, 5, 10, 10, 16, 9, 8, 8, 6, 8, 16,       // 白云挂在那蓝天，像你的微
    5, 12, 5, 5, 12, 5, 9, 16, 8, 6, 8, 8, 8, 10, 12, 16, // 你笑起来真好看，像春天的花一样！
    8, 6, 8, 8, 8, 13, 12, 10, 9, 8, 6, 8, 8, 10, 9, 16,  // 把所有的烦恼，所有的忧愁，统统都吹散
    5, 12, 5, 5, 12, 5, 9, 16, 8, 6, 8, 8, 13, 12, 16,    // 你笑起来真好看，像夏天的阳光
    8, 8, 8, 13, 12, 10, 9, 8, 6, 8, 8, 9, 8, 16          // 整个世界全部的时光，美得像画卷
};

uint8_t MUSIC_BEAUTIFULLY_YOU_LAUGH_LAST[] = {
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, // 想去远方的山川，想去海边看海
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 8, 4,       // 不管风雨有多少，有你就足
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, // 喜欢看你的嘴角，喜欢看你的眉
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 8, 4,       // 白云挂在那蓝天，像你的微
    4, 4, 2, 2, 4, 4, 4, 4, 4, 4, 2, 2, 4, 4, 8, 4, // 你笑起来真好看，像春天的花一样！
    4, 4, 2, 2, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 8, 4, // 把所有的烦恼，所有的忧愁，统统都吹散
    4, 4, 2, 2, 4, 4, 4, 4, 4, 4, 4, 4, 4, 8, 4,    // 你笑起来真好看，像夏天的阳光
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 8, 4        // 整个世界全部的时光，美得像画卷
};

uint8_t MUSIC_BEAUTIFULLY_YOU_LAUGH_PAUSE[] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

#endif
