//
// Copyright 2018 Horizon Robotics.
//
#include <stdint.h>
#ifndef SRC_SPI_DBUS_HAL_H_
#define SRC_SPI_DBUS_HAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#define RESERVED_FIELD (0x00535049)
#define MSG_MCU_VERSION             0x0100
#define MSG_CAN_FILTER_VERSION      0x0101

// enum a10_to_mcu_cmd {
//   MSG_CAN_FILTER = 1,
//   MSG_KEEP_ALIVE,
//   MSG_CAN_SET_OPMODE,
//   MSG_CMD_M_READ_DATA,
//   MSG_CAN_DATA_TRANSMIT,
// };

// enum mcu_to_a10_cmd {
//   MSG_SPI_INIT_RESPONSE = 101,
//   MSG_CAN_DATA,
//   MSG_RES_S_READ_DATA,
// };

enum debug_cmd {
  // MSG_MODE_DEBUG_ON = 201,
  // MSG_MODE_DEBUG_OFF,
  // MSG_POWCTRL_TEST,
  MSG_CLOSE_SPI,
  MSG_MCU_MASTER_START,
  MSG_INIT_SUCCESS,
};

typedef struct {
  uint16_t type;
  uint16_t length;
  uint32_t reverved;
  uint64_t difff_time;
} headerType;//16Byte

struct spi_error_status {
  unsigned int preamble_error;
  unsigned int rx_tmp_buf_overflow;
  unsigned int rx_data_kfifo_overflow;
};

/*
* spi_dbus_init - open and configure spi_dbus module according to specification
* @speed[IN]: SPI communication speed
* @block[IN]: open spi_dbus module with block(block = 1) or unblock(block = 0) mode
* return: 0 on success; if an error occurred return a negative value
*       -1: spi device does not exist
*       -2: initialize fail
*/
int spi_dbus_init(unsigned int speed, int block);

/*
* spi_dbus_exit - close spi_dbus module
* return: void
*/
void spi_dbus_exit(void);

/*
* spi_dbus_send_frame - send one user frame with spi_dbus module
* frame[IN]: pointer to buffer which contains user frame need send
* len[IN]: user frame length want to send
* return: 0 on success; or -1 if an error occurred
*/
int spi_dbus_send_frame(const char *frame, int len);

/*
* spi_dbus_recv_frame - receive one user frame with spi_dbus module
* buff[OUT]: pointer to buffer which contains user data from spi_dbus module
* len[IN]: data length want to receive
* return: the number of byte receive on success; or -1 if an error occurred
*/
int spi_dbus_recv_frame(char *buff, int len);

/*
* spi_dbus_recv_frame_multi - receive multi user frame with spi_dbus module
* buff[OUT]: pointer to buffer which contains user data from spi_dbus module
* en[IN]: data length want to receive
* return: the number of byte receive on success; or -1 if an error occurred
*/
int spi_dbus_recv_frame_multi(char *buff, int len);
/*
* spi_dbus_spi_error_status - get spi error status
* status[OUT]: pointer to structure which contains spi error status
* return: 0 on success; or -1 if an error occurred
*/
int spi_dbus_spi_error_status(struct spi_error_status *status);
/*
* spi_slave_dbus_init - open and configure spi_slave_dbus module according to specification
* @block[IN]: open spi_dbus module with block(block = 1) or unblock(block = 0) mode
* return: 0 on success; or -1 if an error occurred
*/
int spi_slave_dbus_init(int block);

/*
* spi_slave_dbus_exit - close spi_slave_dbus module
* return: void
*/
void spi_slave_dbus_exit(void);

/*
* spi_dbus_slave_send_frame - send one user frame with spi_slave_dbus module
* frame[IN]: pointer to buffer which contains user frame need send
* len[IN]: user frame length want to send
* return: 0 on success; or -1 if an error occurred
*/
int spi_slave_dbus_send_frame(const char *frame, int len);

/*
* spi_slave_dbus_recv_frame - receive one user frame with spi_slave_dbus module
* buff[OUT]: pointer to buffer which contains user data from spi_slave_dbus module
* len[IN]: data length want to receive
* return: the number of byte receive on success; or -1 if an error occurred
*/
int spi_slave_dbus_recv_frame(char *buff, int len);

/*
* spi_slave_dbus_recv_frame_multi - receive multi user frame with spi_slave_dbus module
* buff[OUT]: pointer to buffer which contains user data from spi_slave_dbus module
* en[IN]: data length want to receive
* return: the number of byte receive on success; or -1 if an error occurred
*/
int spi_slave_dbus_recv_frame_multi(char *buff, int len);
/*
* spi_dbus_time_sync_indication - trigger A10 to MCU timestamp interrupt
* return: 0 on success; or -1 if an error occurred
*/
int spi_dbus_time_sync_indication(void);
double GetTimeStamp();
#ifdef __cplusplus
}
#endif
#endif  // SRC_SPI_DBUS_HAL_H_

