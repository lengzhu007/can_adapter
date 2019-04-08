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
#include "spi_dbus_hal.h"


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

#define FRAGMENT_SIZE (64)
#define FRAGMENT_VALID_SIZE (62)

#define SPI_PREAMBLE_OFFSET (0)
#define SPI_HEADER_OFFSET    (1)
#define SPI_DATA_OFFSET      (2)

#define SPI_PREAMBLE (0x47)

/* SPI device name */
static const char *dev_name = "/dev/infineon_slave32764.0";

/* SPI device manage structure*/
struct spi_device_manage {
    int fd;
    unsigned int ref;
};
static struct spi_device_manage spi_dev = {-1, 0};

/*
* spi_slave_dbus_init - open and configure spi_slave_dbus module according to specification
* @block[IN]: open spi_dbus module with block(block = 1) or unblock(block = 0) mode
* return: 0 on success; or -1 if an error occurred
*/
int spi_slave_dbus_init(int block) {
    // open device
    if (!spi_dev.ref) {
        if (block) {
            spi_dev.fd = open(dev_name, O_RDWR);
        } else {
            spi_dev.fd = open(dev_name, O_RDWR | O_NONBLOCK);
        }

        if (spi_dev.fd < 0) {
            perror("Open spi slave device fail:");
            goto open_error;
        }
    }
    ++spi_dev.ref;

    return 0;
open_error:
    return -1;
}

/*
* spi_slave_dbus_exit - close spi_slave_dbus module
* return: void
*/
void spi_slave_dbus_exit(void) {
    if (!--spi_dev.ref) {
        close(spi_dev.fd);
        spi_dev.fd = -1;
    }
}

/*
* spi_dbus_slave_send_frame - send one user frame with spi_slave_dbus module
* frame[IN]: pointer to buffer which contains user frame need send
* len[IN]: user frame length want to send
* return: 0 on success; or -1 if an error occurred
*/
int spi_slave_dbus_send_frame(const char *frame, int len) {
    static char dummy_array[FRAGMENT_SIZE] = {0};
    static spi_header_byte send_spi_header;
    static char spi_frament_send_buf[FRAGMENT_SIZE] = {SPI_PREAMBLE};
    int ret = 0;
    int count = 0;
    int i = 0;
    int last_copy = 0;
#ifdef DEBUG
    int j = 0;
#endif
    struct spi_ioc_transfer io_transfer = {
        .tx_buf = (unsigned int64_t)spi_frament_send_buf,
        .rx_buf = (unsigned int64_t)dummy_array,
        .len = FRAGMENT_SIZE,
    };

    if ((!frame) || (len <= 0)) {
        printf("Invalid parameters\n");
        goto error;
    }

    // calculate fragment count
    count = len / FRAGMENT_VALID_SIZE;
    if (len % FRAGMENT_VALID_SIZE)
        ++count;
    last_copy = len % FRAGMENT_VALID_SIZE;
#ifdef DEBUG
    printf("frame count = %d\n", count);
#endif

    // send SPI fragment
    memset(&send_spi_header, 0, sizeof(send_spi_header));
    for (i = 1; i <= count; ++i) {
        if (i == 1)
            send_spi_header.element_value.start = 1;
        else
            send_spi_header.element_value.start = 0;

        if (i == count)
            send_spi_header.element_value.end = 1;
        else
            send_spi_header.element_value.end = 0;
#ifdef DEBUG
        printf("count = %d\nstart = %d\nend = %d\nextend = %d\n",
            send_spi_header.element_value.count,
            send_spi_header.element_value.start,
            send_spi_header.element_value.end,
            send_spi_header.element_value.extend);
#endif
        spi_frament_send_buf[SPI_HEADER_OFFSET] = send_spi_header.byte_value;
        if ((i == count) && (last_copy != 0)) {
            memcpy(spi_frament_send_buf + SPI_DATA_OFFSET, frame, last_copy);
        } else {
            memcpy(spi_frament_send_buf + SPI_DATA_OFFSET, frame,
                FRAGMENT_VALID_SIZE);
        }

        frame += FRAGMENT_VALID_SIZE;
        ++send_spi_header.element_value.count;
#ifdef DEBUG
        for (j = 0; j < 64; ++j) {
            printf("%x  ", spi_frament_send_buf[j]);
            if (!((j + 1) % 8))
                printf("\n");
        }
        printf("\n");
#endif
        if ((ret = ioctl(spi_dev.fd, SPI_IOC_MESSAGE(1), &io_transfer)) < 0) {
            printf("Send SPI fragment fail\n");
            goto error;
        }
    }

    return 0;
error:
    return -1;
}

/*
* spi_slave_dbus_recv_frame - receive one user frame with spi_slave_dbus module
* buff[OUT]: pointer to buffer which contains user data from spi_slave_dbus module
* len[IN]: data length want to receive
* return: the number of byte receive on success; or -1 if an error occurred
*/
int spi_slave_dbus_recv_frame(char *buff, int len) {
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
* spi_slave_dbus_recv_frame_multi - receive multi user frame with spi_slave_dbus module
* buff[OUT]: pointer to buffer which contains user data from spi_slave_dbus module
* en[IN]: data length want to receive
* return: the number of byte receive on success; or -1 if an error occurred
*/
int spi_slave_dbus_recv_frame_multi(char *buff, int len) {
    int ret = 0;
    int sum = 0;

    if ((!buff) || (len < 0)) {
        printf("Invalid parameters\n");
        goto error;
    }

    while ((ret = spi_slave_dbus_recv_frame(buff + sum, len - sum)) > 0) {
        sum += ret;
    }

    return sum;
error:
    return -1;
}
