//
// Copyright 2018 Horizon Robotics.
//

#include "service_handler_module.h"
// #include <hobot-adas/data-structure/context.h>
// #include <adas/adas_data_structure.h>
#include <sys/time.h>
#include <map>
#include <utility>
#include <vector>
#include <memory>
#include <string>
#include "spi_dbus_hal.h"
#include <fstream>
#include <iostream>
#include <string.h>
#include <unistd.h>
#define ADAS_FPGA
#define ADAS_ALTERA
#if defined(ADAS_FPGA)

#endif

// namespace HobotADAS {



enum SPI_STATE {
  SPI_STATE_UNINIT,
  SPI_STATE_INIT
};

typedef struct {
  uint32_t id;              // CNA id
  uint8_t  dlc;             // CAN dlc
  uint8_t  unused;
  uint16_t counter;         // CAN frame counter for debug
  uint8_t data[8];         // CAN data
} mcu_can_frame;

static uint16_t TS_SYNC_TIMEOUT = 1000;
static uint16_t TS_SYNC_FRAME_NUM = 1000;
static uint16_t MSG_READ_DATA_HEAD_SIZE = 2;
static uint16_t KEEP_ALIVE_TIMEOUT = 1000;
static uint32_t MCU_VERSION_TIMEOUT = 60000;
static char MCU_VERSION_FILE[64] = "/tmp/mcu_version";


 ServiceHandler::ServiceHandler() {}


// int ServiceHandler::ADASModuleInit(hobot::RunContext * context) {//????????????????
//   VLOG(0) << "ServiceHandler init";
//   Init();
//   ServiceMsgPool::Create(20, 0);
//   return ADAS_OK;
// }

int ServiceHandler::Init() {
// #if defined(ADAS_FPGA) && defined(ADAS_ALTERA)
  int spi_state = 0;
  spi_init_inform_ = false;
  service_spi_state_ = SERVICE_SPI_STATE_UNINIT;
  service_slave_spi_state_ = SERVICE_SPI_STATE_UNINIT;

  spi_state = spi_dbus_init(5000000, 0);
  if (spi_state == -1) {
    std::cout  << " no spi device device" << std::endl;
  } else if (spi_state == -2) {
    std::cout  <<  " spi device init fail"<< std::endl;
    return false;
  } else {
    service_spi_state_ = SERVICE_SPI_STATE_INIT;
    // std::cout<< "010 service_spi_state_   " << service_spi_state_<< std::endl ;
    spi_init_inform_ = true;
  }

  if (service_spi_state_ = SERVICE_SPI_STATE_INIT) {//service_spi_state_ = SERVICE_SPI_STATE_INIT = 1
    int recv_size;//没有初始化　随机的值
    char spi_data[2048];
    ServiceHandlerSetOpMode(CAN_OP_OFF);//CAN_OP_OFF = 0
    int64_t ts = GetTimeStamp();

    // std::cout<< "015 recv_size   " << recv_size<< std::endl ;

    do {
      recv_size = spi_dbus_recv_frame(spi_data, 2048);//接系统信息
      // std::cout<< "020 spi_data   " << spi_data<< std::endl ;
      // std::cout << "025 recv size = " << recv_size << std::endl;
      int64_t te = GetTimeStamp();
      if (te - ts > 10) {
        break;
      }
    } while (recv_size > 0);
  }

  spi_state = spi_slave_dbus_init(0);
  if (spi_state == -1) {
    std::cout  << " no spi device device" << std::endl;
  } else if (spi_state == -2) {
    std::cout  <<  " spi device init fail"<< std::endl;
    return false;
  } else {
    service_slave_spi_state_ = SERVICE_SPI_STATE_INIT;
  }

  RegisterServiceRecieve(MSG_CAN_DATA,
    &ServiceHandler::ServiceHandlerCANDataRecieve);
  RegisterServiceRecieve(MSG_RES_S_READ_DATA,
    &ServiceHandler::ServiceHandlerResSReadData);
// #endif
  return 0;
}

void ServiceHandler::Reset() {
  Fini();
}

void ServiceHandler::Fini() {
// #if defined(ADAS_FPGA) && defined(ADAS_ALTERA)
  if (service_spi_state_ == SERVICE_SPI_STATE_INIT) {
    // ServiceHandlerKeepAlive(KEEP_ALIVE_OFF);
    spi_dbus_exit();
    spi_init_inform_ = false;
    service_spi_state_ = SERVICE_SPI_STATE_UNINIT;
  }
  if (service_slave_spi_state_ == SERVICE_SPI_STATE_INIT) {
    spi_slave_dbus_exit();
    service_slave_spi_state_ = SERVICE_SPI_STATE_UNINIT;
  }
// #endif
}
/////////////////////////////////////////
int ServiceHandler::CANFilterVersionRequest(void) {//发给mcu version request 以便于mcu抓取数据 (全抓，还是部分json )
  int ret = 0;
// #if defined(ADAS_FPGA) && defined(ADAS_ALTERA)
  char *frame = NULL;
  int frame_len = 0;
  headerType *header = NULL;
  std::vector<char> spi_frame;

  /* response A10 filter version*/
  spi_frame.resize(2 + sizeof(headerType));
  header = reinterpret_cast<headerType *>(spi_frame.data());
  header->type = MSG_CMD_M_READ_DATA;
  header->length = 2;

  uint8_t* data = reinterpret_cast<uint8_t *>(spi_frame.data()
    + sizeof(headerType));
  data[0] = (MSG_CAN_FILTER_VERSION >> 8) & 0xff;
  data[1] = MSG_CAN_FILTER_VERSION & 0xff;
  frame = spi_frame.data();
  frame_len = static_cast<int>(spi_frame.size());
  ret = spi_dbus_send_frame(frame, frame_len);
  if (ret < 0) {
    std::cout << "send read CAN filter version fail"<<std::endl;
  }
// #endif
  return ret;
}
/////////////////////////////////////

////////////////////////////////////////////////////////
int ServiceHandler::SetCANFilterConfig(canFltCfgType * canFltCfg) {
  int ret = 0;
// #if defined(ADAS_FPGA) && defined(ADAS_ALTERA)
  char *frame = NULL;
  int frame_len = 0;
  std::vector<char> spi_frame;
  headerType *header = NULL;
  ctrlParaType * ctrlPara = NULL;
  char * canFltVer = NULL;
  canFltType * canFilter = NULL;
  uint8_t *data = NULL;

  /*set can filter, only write once*/
  int can_frame_num = canFltCfg->canFilter.size();
  int spi_frame_size = sizeof(headerType)
                     + sizeof(ctrlParaType)
                     + 8
                     + sizeof(canFltType) * can_frame_num;
  spi_frame.resize(spi_frame_size, 0);

  header = reinterpret_cast<headerType *>(spi_frame.data());
  header->type = MSG_CAN_FILTER;
  header->length = spi_frame_size - sizeof(headerType);

  int tmp_pos = sizeof(headerType);
  ctrlPara = reinterpret_cast<ctrlParaType *>(spi_frame.data() + tmp_pos);
  *ctrlPara = canFltCfg->ctrlPare;

  tmp_pos += sizeof(ctrlParaType);
  canFltVer = reinterpret_cast<char *>(spi_frame.data() + tmp_pos);
  memcpy(canFltVer, canFltCfg->canFltVer, 8);

  tmp_pos += 8;
  canFilter = reinterpret_cast<canFltType *>(spi_frame.data() + tmp_pos);
  for (int i = 0; i < can_frame_num; i++) {
    canFilter[i] = canFltCfg->canFilter[i];
  }

  frame = spi_frame.data();
  frame_len = static_cast<int>(spi_frame.size());

  ret = spi_dbus_send_frame(frame, frame_len);
  if (ret < 0) {
    std::cout << "send CAN filtler fail"<<std::endl;
  }
// #endif
  return ret;
}

///////////////////////////////////////////////////////////////////



int ServiceHandler::ServiceHandlerCANDataRecieve(
  msgGeneralType* msg, ServiceMsg * svc_msg) {//ServiceMsg * svc_msg占多少字节
  int output_slot = OUTPUT_SLOT_NONE;
// #if defined(ADAS_FPGA) && defined(ADAS_ALTERA)
  svc_msg->ts_ = ServiceHandlerGetTsMcuSync(&msg->header.diff_time[0]);
  svc_msg->type_ = CAN_DATA;//=3
  svc_msg->length_ = msg->header.length;
  svc_msg->data_.resize(svc_msg->length_);//9Byte
  memcpy(&svc_msg->data_[0], &msg->data[0], svc_msg->length_);
  output_slot = OUTPUT_SLOT_CAN_INPUT;//0
// #endif
  return output_slot;
}

int64_t ServiceHandler::ServiceHandlerGetTsMcuSync(uint8_t * ts) {
  int64_t mcu_ts;
  mcu_ts = static_cast<int64_t>(ts[0]) << 56 |
           static_cast<int64_t>(ts[1]) << 48 |
           static_cast<int64_t>(ts[2]) << 40 |
           static_cast<int64_t>(ts[3]) << 32 |
           static_cast<int64_t>(ts[4]) << 24 |
           static_cast<int64_t>(ts[5]) << 16 |
           static_cast<int64_t>(ts[6]) << 8 |
           static_cast<int64_t>(ts[7]);
  // VLOG(0) << "mcu_ts: " << mcu_ts;

  return mcu_ts;
}

int ServiceHandler::ServiceHandlerResSReadData(
  msgGeneralType* msg, ServiceMsg * svc_msg) {
    int output_slot = OUTPUT_SLOT_NONE;
// #if defined(ADAS_FPGA) && defined(ADAS_ALTERA)
  uint16_t subSvcType = ((msg->data[0] << 8) & 0xFF00)
                        | msg->data[1];//按位　或　a|b 或运算，？？？？？

  if (subSvcType == MSG_CAN_FILTER_VERSION) {
    svc_msg->type_ = CAN_FILTER_VERSION;
    svc_msg->length_ = msg->header.length - MSG_READ_DATA_HEAD_SIZE;
    svc_msg->data_.resize(svc_msg->length_);
    memcpy(&svc_msg->data_[0], &msg->data[MSG_READ_DATA_HEAD_SIZE],
      svc_msg->length_);
    msg->data[msg->header.length] = '\0';
    // std::cout << "can filter version: " << &msg->data[2] <<std::endl;
    output_slot = OUTPUT_SLOT_CAN_INPUT;
  } else if (subSvcType == MSG_MCU_VERSION) {
    msg->data[msg->header.length] = '\0';
    std::ofstream fout(MCU_VERSION_FILE);
    if (fout) {
      fout << &msg->data[2];
      fout.close();
    } else {
      std::cout << "file open failed" <<std::endl;
    }
    output_slot = OUTPUT_SLOT_NONE;
  } else {
  }
// #endif
  return output_slot;
}

int ServiceHandler::ServiceHandlerKeepAlive(uint8_t state) {
// #if defined(ADAS_FPGA) && defined(ADAS_ALTERA)
    char *frame = NULL;
    int frame_len = 0;
    msgHeaderType *header = NULL;
    std::vector<char> spi_frame;
    int spi_frame_size = 0;

    spi_frame.resize(1 + sizeof(headerType));
    header = reinterpret_cast<msgHeaderType *>(spi_frame.data());
    header->type = MSG_KEEP_ALIVE;
    header->length = 1;
    header->reverved = 0x00535049;
    memset(header->diff_time, 0, 8);

    uint8_t *data = reinterpret_cast<uint8_t *>(spi_frame.data()
                    + sizeof(headerType));
    data[0] = state;
    frame = spi_frame.data();
    frame_len = static_cast<int>(spi_frame.size());
    int ret = spi_dbus_send_frame(frame, frame_len);
    if (ret < 0) {
      std::cout << "send keep alive fail"<<std::endl;
      return -1;
    }
// #endif
    return 0;
}

void ServiceHandler::ServiceHandlerTimeStampSyncMon(void) {
// #if defined(ADAS_FPGA) && defined(ADAS_ALTERA)
  static int64_t ts_begin_sync = 0;
  int64_t ts_cur = GetTimeStamp();

  if ((ts_cur - ts_begin_sync >= TS_SYNC_TIMEOUT) || (ts_begin_sync == 0)) {
    int64_t os_time = 0;
    struct timeval curr_time;
    spi_dbus_time_sync_indication();
    gettimeofday(&curr_time, NULL);//&curr_time 取地址　变成指针
    os_time = (static_cast<int64_t>(curr_time.tv_sec) * 1000000LL//long long 
      + static_cast<int64_t>(curr_time.tv_usec));
      // std::cout << "os_time" <<  os_time <<std::endl;
    ServiceHandlerTimeStampSync(os_time);
    ts_begin_sync = ts_cur;
  }
// #endif
}

int ServiceHandler::ServiceHandlerTimeStampSync(int64_t ts) {
// #if defined(ADAS_FPGA) && defined(ADAS_ALTERA)
    char *frame = NULL;
    int frame_len = 0;
    msgHeaderType *header = NULL;
    std::vector<char> spi_frame;
    int spi_frame_size = 0;

    spi_frame.resize(8 + sizeof(headerType));//17
    header = reinterpret_cast<msgHeaderType *>(spi_frame.data());
    header->type = MSG_TIME_STAMP;
    header->length = 8;
    header->reverved = 0x00535049;
    memset(header->diff_time, 0, 8);//初始化　
    //void *memset(void *s, int ch, size_t n);    //函数解释：将s中当前位置后面的n个字节 （typedef unsigned int size_t ）用 ch 替换并返回 s 。

    uint8_t *data = reinterpret_cast<uint8_t *>(spi_frame.data()
                    + sizeof(headerType));

    data[0] = static_cast<uint8_t>(ts >> 56 & 0xFF);//移位
    data[1] = static_cast<uint8_t>(ts >> 48 & 0xFF);
    data[2] = static_cast<uint8_t>(ts >> 40 & 0xFF);
    data[3] = static_cast<uint8_t>(ts >> 32 & 0xFF);
    data[4] = static_cast<uint8_t>(ts >> 24 & 0xFF);
    data[5] = static_cast<uint8_t>(ts >> 16 & 0xFF);
    data[6] = static_cast<uint8_t>(ts >> 8 & 0xFF);
    data[7] = static_cast<uint8_t>(ts & 0xFF);

    frame = spi_frame.data();
    frame_len = static_cast<int>(spi_frame.size());
    int ret = spi_dbus_send_frame(frame, frame_len);
    if (ret < 0) {
      std::cout << "send time stamp fail"<<std::endl;
      return -1;
    }
// #endif
    return 0;
}

int ServiceHandler::ServiceHandlerMcuVerRequest(void) {
  int ret = 0;
// #if defined(ADAS_FPGA) && defined(ADAS_ALTERA)
  char *frame = NULL;
  int frame_len = 0;
  headerType *header = NULL;
  std::vector<char> spi_frame;

  spi_frame.resize(2 + sizeof(headerType));
  header = reinterpret_cast<headerType *>(spi_frame.data());
  header->type = MSG_CMD_M_READ_DATA;
  header->length = 2;

  uint8_t* data = reinterpret_cast<uint8_t *>(spi_frame.data()
    + sizeof(headerType));
  data[0] = (MSG_MCU_VERSION >> 8) & 0xff;
  data[1] = MSG_MCU_VERSION & 0xff;
  frame = spi_frame.data();
  frame_len = static_cast<int>(spi_frame.size());
  ret = spi_dbus_send_frame(frame, frame_len);
  if (ret < 0) {
    std::cout << "send read MCU version fail"<<std::endl;
  }
// #endif
  return ret;
}

bool ServiceHandler::ServiceHandlerMcuVerExist(void) {
    bool exist = true;
// #if defined(ADAS_FPGA) && defined(ADAS_ALTERA)
    if (access(MCU_VERSION_FILE, 0) == -1) {
        exist = false;
    }
// #endif
    return exist;
}

int ServiceHandler::ServiceHandlerSetOpMode(uint8_t state) {//state  = 0
#if defined(ADAS_FPGA) && defined(ADAS_ALTERA)
    char *frame = NULL;
    int frame_len = 0;
    msgHeaderType *header = NULL;
    std::vector<char> spi_frame;
    int spi_frame_size = 0;

    spi_frame.resize(1 + sizeof(headerType));//17Byte
    header = reinterpret_cast<msgHeaderType *>(spi_frame.data());//vector 增加了data()的用法，它返回内置vecotr所指的数组内存的第一个元素的指针
    header->type = MSG_CAN_SET_OPMODE;//3
    header->length = 1;
    header->reverved = 0x00535049;
    memset(header->diff_time, 0, 8);

    uint8_t *data = reinterpret_cast<uint8_t *>(spi_frame.data()
                    + sizeof(headerType));
    data[0] = state;//将state值存放在　spi_frame.data()　后边　16个字节处　
    frame = spi_frame.data();
    frame_len = static_cast<int>(spi_frame.size());
    int ret = spi_dbus_send_frame(frame, frame_len);//发送　spi_frame＋state  成员都是0

    // std::cout<< "012 spi_frame.data() = " << spi_frame.data() << std::endl;
    //  std::cout<<"013 spi_frame.size()=" << spi_frame.size()
    //  << "  ret =" << ret << std::endl ;
    if (ret < 0) {
      std::cout << "send set op mode fail"<<std::endl;
      return -1;
    }
#endif
    return 0;
}

bool ServiceHandler::RegisterServiceRecieve(int index, svcFunc func) {
  // RegisterServiceRecieve(MSG_CAN_DATA, 102
   // &ServiceHandler::ServiceHandlerCANDataRecieve);
  std::pair<int, svcFunc> serviceRecieve_pair(index, func);
  if (service_recieve_map_.find(index) == service_recieve_map_.end()) {
    service_recieve_map_.insert(serviceRecieve_pair);
  } else {
    std::cout << "duplicate ServiceRecieve, ignore"<<std::endl;
    return false;
  }
  return true;
}

int ServiceHandler::ServiceHandlerRecvParse(msgGeneralType* msg,
  ServiceMsg * svc_msg) {
  int output_slot = OUTPUT_SLOT_NONE;
#if defined(ADAS_FPGA) && defined(ADAS_ALTERA)
  std::map<int, svcFunc>::iterator iter;

  for (iter = service_recieve_map_.begin();
    iter != service_recieve_map_.end(); iter++) {
    if (msg->header.type == iter->first) {
      if (iter->second != NULL) {
          svcFunc func = iter->second;
          output_slot = (this->*func)(msg, svc_msg);
      }
        break;
    }
  }
#endif
  return output_slot;
}

void ServiceHandler::ServiceHandlerKeepAliveMon(void) {
  static int64_t ts_begin_ka = 0;
  int64_t ts_end = GetTimeStamp();

  if ((ts_end - ts_begin_ka >= KEEP_ALIVE_TIMEOUT) || (ts_begin_ka == 0)) {
    ServiceHandlerKeepAlive(KEEP_ALIVE_ON);
    ts_begin_ka = ts_end;
  }
}

void ServiceHandler::ServiceHandlerMcuVerMon(void) {
  static int64_t ts_begin_mcuv = 0;
  int64_t ts_end = GetTimeStamp();

  if ((ts_end - ts_begin_mcuv >= MCU_VERSION_TIMEOUT) || (ts_begin_mcuv == 0)) {
    bool exist = ServiceHandlerMcuVerExist();
    if (exist == false) {
      ServiceHandlerMcuVerRequest();
    }
    ts_begin_mcuv = ts_end;
  }
}


// }  
