#include "tfcard.h"
#include "fatfs.h"
#include "stdio.h"
#include "motor.h"
#include "user_lib.h"

static FRESULT retsd;

/**
 * @brief ������tfcard_start������SD���ļ�ϵͳ��
 */
void tfcard_start(void)
{
    retsd = f_mount(&SDFatFS, "0:/", 1);
    if (retsd == FR_OK) {
        // printf("f_mount success\r\n");
    } else {
        // printf("f_mount error: %d\r\n", retsd);
    }
}

/**
 * @brief ������tfcard_stop��ͨ��ȡ��FATFS���������������ֹͣTF����
 */
void tfcard_stop(void)
{
    retsd = FATFS_UnLinkDriver(SDPath);
    if (retsd == FR_OK) {
        printf("FATFS_UnLinkDriver success\r\n");
    } else {
        printf("FATFS_UnLinkDriver error: %d\r\n", retsd);
    }
}
