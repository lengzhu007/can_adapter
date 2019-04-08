//
// Copyright 2018 Horizon Robotics.
//

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <pthread.h>
#include <semaphore.h>
#include <errno.h>
#include <dirent.h>
#include "spi_dbus_hal.h"
#include <sys/time.h>
#include <iostream>

// #define DEBUG

typedef struct {
    unsigned char extend : 1;
    unsigned char end : 1;
    unsigned char start : 1;
    unsigned char count : 5;
} spi_header;

typedef union {
    spi_header element_value;
    unsigned char byte_value;
} spi_header_byte;

#define SPI_SPEED_MIN (0)
#define SPI_SPEED_MAX (35000000)  // 35MHz

#define FRAGMENT_SIZE (64)
#define FRAGMENT_VALID_SIZE (62)

#define SPI_PREAMBLE_OFFSET (0)
#define SPI_HEADER_OFFSET    (1)
#define SPI_DATA_OFFSET      (2)

#define SPI_PREAMBLE (0x47)

#define SPI_IO_NOTIFY_MCU_INIT    _IO(SPI_IOC_MAGIC, 6)
#define SPI_IO_GET_ERROR_STATUS   _IO(SPI_IOC_MAGIC, 7)
#define SPI_IO_TIMESTAMP_TRIG     _IO(SPI_IOC_MAGIC, 8)

/* SPI device name */
// static const char *dev_name = "/dev/infineon32765.0";
static const char *dev_name = "/dev/infineon32766.0";
static const char *dev_real_name = "infineon32766.0";

/* named semaphore path*/
// static const char *sem_path = "spi_dbus_semaphore";
// sem_t *sem = NULL;

/* SPI device manage structure*/
struct spi_device_manage {
    int fd;
    unsigned int ref;
};
pthread_mutex_t global_mutex = PTHREAD_MUTEX_INITIALIZER;
static struct spi_device_manage spi_dev = {-1, 0};

pthread_t pid = 0;
static void *keep_alive_thread(void *arg) {
    char send_buf[128] = {0};
    headerType msg_head;
    int ret = 0;

    memset(&msg_head, 0, sizeof(msg_head));//作用是将某一块内存中的内容全部设置为指定的值,结构体清零
    msg_head.type = 2;//MSG_KEEP_ALIVE
    msg_head.reverved = RESERVED_FIELD;
    memcpy(send_buf, &msg_head, sizeof(msg_head));//从源src所指的内存地址的起始位置开始拷贝n个字节到目标dest所指的内存地址的起始位置中

    while (1) {
        sleep(1);
        if ((ret = spi_dbus_send_frame(send_buf, sizeof(msg_head))) < 0) {
            printf("Send keep alive frame fail\n");
        }
    }

    return 0;
}

/*
* spi_device_exist - test spi device whether exist
* return: if exist return 0, the rest return -1;
*/
static int spi_device_exist(void) {
    DIR *dp = NULL;
    struct dirent *dirp = NULL;

    if (!(dp = opendir("/dev"))) {
        printf("open dir error\n");
        goto open_dir_error;
    }

    while ((dirp = readdir(dp))) {
        if (!strcmp(dirp->d_name, dev_real_name))
            break;
    }

    if (dirp)
        return 0;
    else
        return -1;
open_dir_error:
    return -1;
}

/*
* spi_dbus_init - open and configure spi_dbus module according to specification
* @speed[IN]: SPI communication speed
* @block[IN]: open spi_dbus module with block(block = 1) or unblock(block = 0) mode
* return: 0 on success; if an error occurred return a negative value
*       -1: spi device does not exist
*       -2: initialize fail
*/
int spi_dbus_init(unsigned int speed, int block) {
    int ret = 0;
    char send_buf[128] = {0};
    headerType msg_head;

    // test spi device whether exist
    if (spi_device_exist() < 0) {
        printf("No spi device\n");
        return -1;
    }

    // open device
    pthread_mutex_lock(&global_mutex);
    if (!spi_dev.ref) {
        if (block) {
            spi_dev.fd = open(dev_name, O_RDWR);
        } else {
            spi_dev.fd = open(dev_name, O_RDWR | O_NONBLOCK);
        }

        if (spi_dev.fd < 0) {
            printf("Open device fail\n");
            goto open_error;
        }
        // config spi master speed
        if (speed) {
            if ((speed < SPI_SPEED_MIN) || (speed > SPI_SPEED_MAX)) {
                printf("Invalid SPI speed\n");
                goto open_error;
            }

            if ((ret = ioctl(spi_dev.fd,
                   SPI_IOC_WR_MAX_SPEED_HZ, &speed)) < 0) {
                printf("Config SPI speed fail\n");
                goto open_error;
            }
        }
#if 0
        // open named semaphore
        sem = sem_open(sem_path, O_CREAT | O_EXCL, 0666, 1);
        if (sem == SEM_FAILED) {
            if (errno == EEXIST) {
                sem = sem_open(sem_path, 0);
                printf("just open\n");
                if (sem == SEM_FAILED) {
                    perror("Semaphore open fail");
                    close(spi_dev.fd);
                    goto open_error;
                }
            } else {
                perror("Semaphore open fail");
                close(spi_dev.fd);
                goto open_error;
            }
        }
        printf("sem = %x\n", sem);
#endif
    }
    ++spi_dev.ref;
    pthread_mutex_unlock(&global_mutex);

#if 0
    // info MCU init SPI module
    if ((ret = ioctl(spi_dev.fd, SPI_IO_NOTIFY_MCU_INIT)) < 0) {
        printf("Notify MCU SPI init fail\n");
        printf("%d_%d\n", spi_dev.ref, spi_dev.fd);
        goto ioctl_error;
    }
#endif

#if 0
    // create keep alive thread
    if (ret = pthread_create(&pid, NULL, keep_alive_thread, NULL)) {
        printf("pthread_create fail\n");
        goto pthread_create_error;
    }
#endif

    return 0;
pthread_create_error:
#if 0
    memset(&msg_head, 0, sizeof(msg_head));
    msg_head.type = MSG_CLOSE_SPI;
    msg_head.reverved = RESERVED_FIELD;
    memcpy(send_buf, &msg_head, sizeof(msg_head));
    spi_dbus_send_frame(send_buf, sizeof(msg_head));
#endif
open_error:
    pthread_mutex_unlock(&global_mutex);
    return -2;
}

/*
* spi_dbus_exit - close spi_dbus module
* return: void
*/
void spi_dbus_exit(void) {
    char send_buf[128] = {0};
    headerType msg_head;
    int ret = 0;

    pthread_mutex_lock(&global_mutex);
    if (!--spi_dev.ref) {
#if 0
    memset(&msg_head, 0, sizeof(msg_head));
    msg_head.type = MSG_CLOSE_SPI;
    msg_head.reverved = RESERVED_FIELD;
    memcpy(send_buf, &msg_head, sizeof(msg_head));
    spi_dbus_send_frame(send_buf, sizeof(msg_head));
#endif
    // sem_close(sem);
    // sem_unlink(sem_path);
    // sem = NULL;
    close(spi_dev.fd);
    spi_dev.fd = -1;
#if 0
        pthread_cancel(pid);
#endif
    }
    pthread_mutex_unlock(&global_mutex);
}

/*
* spi_dbus_send_frame - send one user frame with spi_dbus module
* frame[IN]: pointer to buffer which contains user frame need send
* len[IN]: user frame length want to send
* return: 0 on success; or -1 if an error occurred
*/
#define TX_TEMP_BUFFER_LEN (4 *1024)
int spi_dbus_send_frame(const char *frame, int len) {
    int ret = 0;

    if ((!frame) || (len < 0) || (len > TX_TEMP_BUFFER_LEN)) {
        printf("Invalid parameters\n");
        goto error;
    }

    if ((ret = write(spi_dev.fd, frame, len)) < 0) {// 将frame的len 个byte 写到　fd 返回写入的数量　error 返回－１
        printf("Write fail\n");
            goto error;
        }

    return 0;
error:
    return -1;
}

/*
* spi_dbus_recv_frame - receive one user frame with spi_dbus module
* buff[OUT]: pointer to buffer which contains user data from spi_dbus module
* len[IN]: data length want to receive
* return: the number of byte receive on success; or -1 if an error occurred
*/
int spi_dbus_recv_frame(char *buff, int len) {
    int ret = 0;

    if ((!buff) || (len < 0)) {
        printf("Invalid parameters\n");
        goto error;
    }

    if ((ret = read(spi_dev.fd, buff, len)) < 0) {
        printf("Read fail\n");
        goto error;
    }

    return ret;
error:
    return -1;
}

/*
* spi_dbus_recv_frame_multi - receive multi user frame with spi_dbus module
* buff[OUT]: pointer to buffer which contains user data from spi_dbus module
* en[IN]: data length want to receive
* return: the number of byte receive on success; or -1 if an error occurred
*/
int spi_dbus_recv_frame_multi(char *buff, int len) {
    int ret = 0;
    int sum = 0;

    if ((!buff) || (len < 0)) {
        printf("Invalid parameters\n");
        goto error;
    }

    while ((ret = spi_dbus_recv_frame(buff + sum, len - sum)) > 0) {
        sum += ret;
    }

    return sum;
error:
    return -1;
}

/*
* spi_dbus_spi_error_status - get spi error status
* status[OUT]: pointer to structure which contains spi error status
* return: 0 on success; or -1 if an error occurred
*/
int spi_dbus_spi_error_status(
  struct spi_error_status *status) {
    int ret = -1;

    if ((ret = ioctl(spi_dev.fd,
           SPI_IO_GET_ERROR_STATUS, status)) < 0) {
        printf("Get SPI error status fail\n");
        goto error;
    }
    return 0;
error:
    return -1;
}

/*
* spi_dbus_time_sync_indication - trigger A10 to MCU timestamp interrupt
* return: 0 on success; or -1 if an error occurred
*/
int spi_dbus_time_sync_indication(void) {
    int ret = -1;

    if ((ret = ioctl(spi_dev.fd, SPI_IO_TIMESTAMP_TRIG, NULL)) < 0) {//
    //ioctl(int fd, ind cmd, …)； 
    //其中fd是用户程序打开设备时使用open函数返回的文件标示符，
    //cmd是用户程序对设备的控制命令，至于后面的省略号，那是一些补充参数，一般最多一个，这个参数的有无和cmd的意义相关
        printf("Trigger timestamp int fail\n");
        goto error;
    }
    return 0;
error:
    return -1;
}


float GetTimeStamp()//
{
float timestamp;
struct timeval tv;
gettimeofday(&tv,NULL); //gettimeofday(&start,&tz);结果一样
timestamp = tv.tv_sec+tv.tv_usec/1000000;
return timestamp;
}


