//
// Copyright 2018 Horizon Robotics.
//

#ifndef HOBOT_ADAS_MODULES_SERVICE_HANDLER_MODULE_H_
#define HOBOT_ADAS_MODULES_SERVICE_HANDLER_MODULE_H_

//#include <hobot-adas/data-structure/message.h>
//#include <hobot-adas/data-structure/adas_module.h>
#include <vector>
#include <list>
#include <string>
#include <algorithm>
#include <map>
#include <stdint.h>
#include <asm-generic/int-ll64.h>


class ServiceMsg {
 // friend class MemPool<ServiceMsg>;//MemPool可以访问serviceMsg的成员函数
 public:
  int64_t ts_;//8
  uint16_t type_;//2
  uint16_t length_;//2
  std::vector<char> data_;//>

  ServiceMsg() {
  }
  ~ServiceMsg() {
  }
  void Reset() {
  }
 // bool DeepCopy(const ADASMsg *) {//函数的用处　??????
   // return true;
 // }
};
enum SPI_MSG_TYPE {
  SPI_INIT = 1,
  CAN_FILTER_VERSION = 2,
  CAN_DATA = 3,
};

enum CAN_OP_MODE {
    CAN_OP_OFF = 0,
    CAN_OP_ON = 1,
};
 enum SPI_RESPONSE_STATUS {
    RESPONSE_UNKOWN = 0,
    RESPONSE_NODATA = 1,
    RESPONSE_MultiFrame = 2,
  };
  enum CAN_MASK {
    CAN_MASK_CAN0 = 0,
    CAN_MASK_CAN1 = 1
  };

  enum {
  /* A10->MCU CMD */
  MSG_CAN_FILTER          = 1,
  MSG_KEEP_ALIVE,
  MSG_CAN_SET_OPMODE,
  MSG_CMD_M_READ_DATA,
  MSG_CAN_DATA_TRANSMIT,
  MSG_CMD_RESET,
  MSG_CMD_PREPARE_RST,
  MSG_CMD_RELEASE,
  MSG_TIME_STAMP,
  /* MCU->A10 DATA Transfer */
  MSG_SPI_INIT_RESPONSE   = 101,
  MSG_CAN_DATA,
  MSG_RES_S_READ_DATA,
  /* A10->MCU TEST CMD */
  MSG_TEST_MODE_ON        = 201,
  MSG_TEST_MODE_OFF,
  MSG_POWCTRL_TEST,
  MSG_SPI_SLAVE_CLOSE,
  MSG_SPI_MASTER_TEST,
  MSG_CMD_CAN_TEST,
  MSG_MODE_DEBUG_ON,
  MSG_MODE_DEBUG_OFF,
  MSG_MCU_MASTER_TEST,
  /* MCU->A10 TEST RESPONSE */
  MSG_RES_CAN_TEST        = 301
};


typedef struct {
    uint32_t can_id;
    uint8_t dlc;
    uint8_t direct;
    uint8_t type;
    uint8_t node_id;
} canFltType;

typedef struct {
    /* config CAN throughout, bit0: CAN0, bit1: CAN 1.  */
    /* value: 0 -- do not config, 1 -- config */
    uint8_t  throut;
    uint8_t  rvseThrout;  /* reverse of throut paremeter.  */
    uint8_t  unused[6];
} ctrlParaType;

typedef struct {
  ctrlParaType ctrlPare;
  char canFltVer[8];
  std::vector<canFltType> canFilter;
} canFltCfgType;

  typedef struct {
      int64_t ts_;
      std::vector<struct can_frame> can_frame_;
  } SpiCanFrameType;

#define CAN_MAX_DLEN 8
  struct can_frame {
	__u32 can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
	__u8    can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
	__u8    data[CAN_MAX_DLEN] __attribute__((aligned(8)));

  // int can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
  // unsigned char    can_dlc;
  // /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
  // unsigned char    data[8];
};

typedef std::vector<struct can_frame> OutputCAN;

class ServiceHandler 
//: public ADASModule
 {
#define MSG_SVCHDL_MAX_SIZE     4096u

 public:
  enum OUTPUT_SLOT {
    OUTPUT_SLOT_CAN_INPUT = 0,
    OUTPUT_SLOT_CAN_OUTPUT = 1,
    OUTPUT_SLOT_NONE = 255,
  };
  enum SERVICE_SPI_STATE {
    SERVICE_SPI_STATE_UNINIT = 0,
    SERVICE_SPI_STATE_INIT = 1,
  };
  enum SERVICE_COM_STATE {
    SERVICE_STATE_NO_COM = 0,
    SERVICE_STATE_COM = 1,
  };
  enum KEEP_ALIVE_TYPE {
    KEEP_ALIVE_OFF = 0,
    KEEP_ALIVE_ON = 1,
  };
  enum COM_INIT_STEP {
    COM_INIT_STEP_NONE = 0,
    COM_INIT_STEP_WAIT_RES = 1,
  };
  typedef struct {
    uint16_t type;            //* Message type */2
    uint16_t length;          //* Data length */2
    uint32_t reverved;        //* For SPI identify: 0x00535049 */4
    uint8_t  diff_time[8];    //* Not used now */1
  } msgHeaderType;//9Byte

  typedef struct {
    msgHeaderType header;
    uint8_t       version[8];
  } msgMcuVersionType;

  typedef struct {
    msgHeaderType header;//
    uint8_t       data[MSG_SVCHDL_MAX_SIZE];
  } msgGeneralType;//10Byte

  typedef int (ServiceHandler::*svcFunc)(msgGeneralType * data,
    ServiceMsg * svc_msg);//函数指针

  ServiceHandler();
  ~ServiceHandler();

  //int ADASModuleInit(hobot::RunContext *contex);// override;
  int Init();
  void Reset();
  virtual void Fini();
  
  int SetCANFilterConfig(canFltCfgType * canFltCfg);
  int CANFilterVersionRequest(void);

  //FORWARD_DECLARE(ServiceHandler, 0);
    void ServiceHandlerMcuVerMon(void);
  void ServiceHandlerKeepAliveMon(void);
  void ServiceHandlerTimeStampSyncMon(void);
 int ServiceHandlerRecvParse(msgGeneralType * data, ServiceMsg * svc_msg);
 int ServiceHandlerSetOpMode(uint8_t state);

 bool get_spi_init_inform_() const {return spi_init_inform_;}
 int get_service_spi_state_()const {return service_spi_state_;}
 int get_service_slave_spi_state_() const {return service_slave_spi_state_;}
 int get_service_com_state_() const {return service_com_state_;}
 int get_service_com_init_step_() const {return service_com_init_step_;}
 private:
  void ServiceHandlerSpiComCheck(void);
  bool RegisterServiceRecieve(int index, svcFunc func);

  int ServiceHandlerComInitRes(msgGeneralType * data,
     ServiceMsg * svc_msg);
  int ServiceHandlerCANDataRecieve(msgGeneralType * data,
    ServiceMsg * svc_msg);
  int ServiceHandlerResSReadData(msgGeneralType * data,
    ServiceMsg * svc_msg);
  int ServiceHandlerKeepAlive(uint8_t state);
  
  int ServiceHandlerComInit(void);
  int ServiceHandlerMcuVerRequest(void);
  int ServiceHandlerTimeStampSync(int64_t ts);
  bool ServiceHandlerMcuVerExist(void);


  //int ServiceHandlerRecvParse(msgGeneralType * data, ServiceMsg * svc_msg);
  int64_t ServiceHandlerGetTsMcuSync(uint8_t * ts);

  std::map<int, svcFunc> service_recieve_map_;
  bool spi_init_inform_;
  int service_spi_state_;
  int service_slave_spi_state_;
  int service_com_state_;
  int service_com_init_step_;
};

//typedef MemPool<ServiceMsg> ServiceMsgPool;

//}  // end of namespace HobotADAS

#endif  // HOBOT_ADAS_MODULES_SERVICE_HANDLER_MODULE_H_
