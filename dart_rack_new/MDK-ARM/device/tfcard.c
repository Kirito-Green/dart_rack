#include "tfcard.h"
#include "fatfs.h"
#include "stdio.h"
#include "motor.h"
#include "user_lib.h"

static FRESULT retsd;

/**
 * @brief 函数“tfcard_start”挂载SD卡文件系统。
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
 * @brief 函数“tfcard_stop”通过取消FATFS驱动程序的链接来停止TF卡。
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
