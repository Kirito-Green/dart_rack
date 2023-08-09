#include "tfcard_transmit_thread.h"
#include "tfcard.h"
#include "cmsis_os.h"
#include "beep.h"
#include "launch_thread.h"
#include "stdio.h"
#include "fatfs.h"
#include "motor.h"
#include "user_lib.h"

static FRESULT retsd;
static DIR SDDir;
static FILINFO fno;
static char *dir_robomaster_name = "0:/robomaster";
static char *file_test_name      = "0:/robomaster/test.txt";
static char *file_motor_name     = "0:/robomaster/motor.txt";
static char *file_measure_name   = "0:/robomaster/measure.txt";
static char *file_referee_name   = "0:/robomaster/referee.txt";
static char wtext[TFCARD_BUFF_SIZE];
static char rtext[TFCARD_BUFF_SIZE];
static char temp_text[TFCARD_BUFF_SIZE];
static uint32_t byteswritten, bytesread; /* File write/read counts */
static int32_t dist_ecd_read[MOTOR_DIST_ECD_NUMBER];
static int32_t dist_ecd_save[MOTOR_DIST_ECD_NUMBER];
static uint8_t cnt;
static uint8_t motor_file_open_flag = 0;
static DartRackStateMachine_e state;
static DartRackMotorMeasure_t measure;

FRESULT tfcard_read_motor(DartRackMotorMeasure_t *measure);
FRESULT tfcard_open_motor(void);
FRESULT tfcard_close_motor(void);
FRESULT tfcard_save_motor(void);

/**
 * @brief ������tfcard_transmit_thread����һ��������ѭ���������ϵͳ��״̬������״ִ̬�в�ͬ�Ĳ�����
 * ����رջ�򿪵���ļ�����������ݱ��浽 TF ���Լ��ӳ�һ��ʱ�䡣
 *
 * @param argument ��argument��������ָ����ܴ��ݸ��̵߳��κθ��Ӳ�����ָ�롣����������£������ᱻʹ�ò��ҿ��Ա����ԡ�
 */
void tfcard_transmit_thread(void const *argument)
{
    /* ������Ƶ����*/
    // tfcard_start();
    // for (;;) {
    //     get_state(&state);
    //     switch (state) {
    //         case DART_RACK_NO_FORCE:
    //             if (motor_file_open_flag) {
    //                 if (tfcard_close_motor() == FR_OK) {
    //                     printf("tfcard close ok\r\n");
    //                     motor_file_open_flag = 0;
    //                     beep_fine();
    //                 } else {
    //                     printf("tfcard close error\r\n");
    //                     beep_alarm();
    //                 }
    //             }
    //             /* --------------------------------- �������ݴ��� --------------------------------- */

    //             /* --------------------------------- ��Ƶ���ݴ��� --------------------------------- */

    //             osDelay(100);
    //             break;
    //         default:
    //             /* --------------------------------- ������ݴ洢 --------------------------------- */
    //             if (!motor_file_open_flag) {
    //                 retsd = tfcard_open_motor();
    //                 if (retsd == FR_OK) {
    //                     printf("tfcard open ok\r\n");
    //                     motor_file_open_flag = 1;
    //                     beep_fine();
    //                 } else {
    //                     printf("tfcard open error: %d\r\n", retsd);
    //                     beep_alarm();
    //                 }
    //             }

    //             retsd = tfcard_save_motor();
    //             if (retsd == FR_OK) {
    //                 beep_fine();
    //                 printf("tfcard save ok\r\n");
    //             } else {
    //                 beep_alarm();
    //                 printf("tfcard save error: %d\r\n", retsd);
    //             }
    //             osDelay(3000);
    //             break;
    //     }
    // }

    /* ���洢 */
    osDelay(500);

    retsd = tfcard_open_motor();
    if (retsd == FR_OK) {
        printf("tfcard open ok\r\n");
        beep_fine();
    } else {
        printf("tfcard open error: %d\r\n", retsd);
        beep_alarm();
    }

    for (;;) {
        retsd = tfcard_save_motor();
        if (retsd == FR_OK) {
            beep_fine();
            printf("tfcard save ok\r\n");
        } else {
            beep_alarm();
            printf("tfcard save error: %d\r\n", retsd);
        }
        osDelay(3000);
    }
}

/**
 * @brief ������tfcard_read_motor���� SD ���ϵ��ļ���ȡ�������ֵ��
 *
 * @param measure ������measure����ָ��DartRackMotorMeasure_t�����ͽṹ��ָ�롣�ýṹ���ܰ������� SD ����ȡ���������ı������ֶΡ�
 *
 * @return FRESULT���͵ı������Ǳ�ʾ�ļ������ɹ���ʧ�ܵĽ���롣
 */
FRESULT tfcard_read_motor(DartRackMotorMeasure_t *measure)
{
    retsd = f_opendir(&SDDir, dir_robomaster_name);
    if (retsd == FR_OK) {
        // printf("f_opendir success\r\n");
        retsd = f_open(&SDFile, file_motor_name, FA_READ);
        if (retsd == FR_OK) {
            // printf("f_open success\r\n");
            retsd = f_read(&SDFile, rtext, sizeof(rtext), (uint32_t *)&bytesread);
            if (retsd == FR_OK) {
                // printf("f_read success\r\n");
                // printf("%s\n", rtext);
                cnt = sscanf(rtext, "%d%s%d%s%d%s%d%s%d%s",
                             &dist_ecd_read[0], temp_text,
                             &dist_ecd_read[1], temp_text,
                             &dist_ecd_read[2], temp_text,
                             &dist_ecd_read[3], temp_text,
                             &dist_ecd_read[4], temp_text);
                if (cnt == MOTOR_SAVE_NUMBER) {
                    // printf("sscanf success\r\n");
                    // printf("%d %d %d %d %d\r\n",
                    //    dist_ecd_read[0],
                    //    dist_ecd_read[1],
                    //    dist_ecd_read[2],
                    //    dist_ecd_read[3],
                    //    dist_ecd_read[4]);

                    motor_update_dist_ecd(dist_ecd_read);
                } else {
                    // printf("sscanf error: %d\r\n", cnt);
                    return retsd;
                }
                retsd = f_close(&SDFile);
                if (retsd == FR_OK) {
                    // printf("f_close success\r\n");
                    retsd = f_closedir(&SDDir);
                    if (retsd == FR_OK) {
                        // printf("f_closedir success\r\n");
                    } else {
                        // printf("f_closedir error: %d\r\n", retsd);
                        return retsd;
                    }
                } else {
                    // printf("f_close error: %d\r\n", retsd);
                    return retsd;
                }
            } else {
                // printf("f_read error: %d\r\n", retsd);
                return retsd;
            }
        } else {
            // printf("f_open error: %d\r\n", retsd);
            return retsd;
        }
    } else {
        // printf("f_opendir error: %d\r\n", retsd);
        return retsd;
    }
    return retsd;
}

/**
 * @brief ������tfcard_open_motor����SD�ϵ�������ļ������ؽ����
 *
 * @return FRESULT ���͵ı�����
 */
FRESULT tfcard_open_motor(void)
{
    retsd = f_opendir(&SDDir, dir_robomaster_name);
    if (retsd == FR_OK) {
        // printf("f_opendir success\r\n");
        retsd = f_open(&SDFile, file_motor_name, FA_CREATE_ALWAYS | FA_WRITE);
        if (retsd == FR_OK) {
            // printf("f_open success\r\n");
        } else {
            // printf("f_open error: %d\r\n", retsd);
            return retsd;
        }
    } else {
        // printf("f_opendir error: %d\r\n", retsd);
        return retsd;
    }
    return retsd;
}

/**
 * @brief ������tfcard_close_motor���ر� SD ���ϵĵ�������ļ���Ŀ¼�����ؽ����
 *
 * @return FRESULT ���͵ı�����
 */
FRESULT tfcard_close_motor(void)
{
    retsd = f_close(&SDFile);
    if (retsd == FR_OK) {
        // printf("f_close success\r\n");
        retsd = f_closedir(&SDDir);
        if (retsd == FR_OK) {
            // printf("f_closedir success\r\n\r\n");
        } else {
            // printf("f_closedir error: %d\r\n\r\n", retsd);
            return retsd;
        }
    } else {
        // printf("f_close error: %d\r\n", retsd);
        return retsd;
    }
    return retsd;
}

/**
 * @brief �ù��ܽ��������ֵ���浽 TF ���С�
 *
 * @return ������retsd����ֵ��������Ϊ FRESULT��
 */
FRESULT tfcard_save_motor(void)
{
    get_motor_dist_ecd(dist_ecd_save);
    sprintf(wtext, "%d #pitch\r\n%d #drag_left\r\n%d #drag_right\r\n%d #load\r\n%d #adjust\r\n",
            dist_ecd_save[0],
            dist_ecd_save[1],
            dist_ecd_save[2],
            dist_ecd_save[3],
            dist_ecd_save[4]);
    // printf("%s\r\n", wtext);
    retsd = f_lseek(&SDFile, 0);
    if (retsd == FR_OK) {
        // printf("f_lseek success\r\n");
        retsd = f_write(&SDFile, wtext, sizeof(wtext), (uint32_t *)&byteswritten);
        if (retsd == FR_OK) {
            // printf("f_write success\r\n");
            retsd = f_sync(&SDFile);
            if (retsd == FR_OK) {
                // printf("f_sync success\r\n");
            } else {
                // printf("f_sync error: %d\r\n", retsd);
            }
        } else {
            // printf("f_write error: %d\r\n", retsd);
        }
    } else {
        // printf("f_lseek error: %d\r\n", retsd);
    }
    return retsd;
}

// FRESULT tfcard_save_measure(void)
// {
// }

// FRESULT tfcard_save_referee(void)
// {
// }
