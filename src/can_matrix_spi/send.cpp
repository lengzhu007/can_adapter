#include <string.h>
#include <iostream>
#include <unistd.h>
#include <vector>
#include <memory.h>
#include <utility>
#include <memory>
#include "spi_dbus_hal.h"
#include "service_handler_module.h"
#include <fstream>
#include <stdio.h>

int send_can_spi(OutputCAN * outputcan)
{
  //发送给控制模块的信息定义　Trajectory＆Odometry
  // typedef unsigned int vbittype;
  // typedef struct _Trajectory_Start //id = 256　　　10hz
  // {
  //   vbittype Trajectory_version : 8;
  //   vbittype Points_Number : 8;
  //   vbittype Emergency_brake_request : 1;
  //   vbittype Turn_indicator_request : 2;
  //   vbittype NULL_Bit1 : 32; //19-59//长度超了？？？
  //   vbittype NULL_Bit2 : 9;
  //   vbittype Trajectory_Counter : 4;
  // } _Trajectory_Start;

  // typedef struct _Points_ //id =306 -257//50各点
  // {
  //   vbittype Orentation_Yaw : 16;
  //   vbittype Speed : 9;
  //   vbittype Gear : 3;
  //   vbittype Position_X : 18;
  //   vbittype Position_Y : 18;
  // } _Points_;

  // typedef struct _Trajectory_End //id=457
  // {
  //   vbittype Trajectory_version : 8; //
  //   vbittype Points_Number : 8;
  //   vbittype NULL_Bit1 : 32; //20-59;
  //   vbittype NULL_Bit2 : 8;
  //   vbittype Trajectory_Counter : 4;
  // } _Trajectory_End;

  // typedef struct _Vehicle_2D_Odometry //id=458  50hz
  // {
  //   vbittype Position_X : 18; //
  //   vbittype Position_Y : 18;
  //   vbittype Orentation : 16;
  //   vbittype NULL_Bit : 8;          //空位？？？？？？？？？？？？？？  字节顺序　　　52-59
  //   vbittype Odometry_Counter : 4;
  // } _Vehicle_2D_Odometry;

  // typedef struct _Vehicle_3D_Odometry //ID=459
  // {
  //   vbittype Orentation_Roll : 16;
  //   vbittype Orentation_Pitch : 16;
  //   vbittype Position_Z : 18;
  //   vbittype NULL_Bit : 10; //空bit ？？？？？？？？？？？？？？  50-59
  //   vbittype Odometry_Counter : 4;
  // } _Vehicle_3D_Odometry;


//  ServiceHandler *service_handler = new ServiceHandler();
//  service_handler->Init();
//  service_handler->ServiceHandlerSetOpMode(CAN_OP_ON);//可以去掉 
///////////////////////////////////////////////////////////////////////////
  char *frame = NULL;
  int frame_len = 0;
  headerType *header = NULL;
  int spi_frame_size = sizeof(headerType) + 1;
  std::vector<char> spi_frame(spi_frame_size, 0);
// init 中已包含初始化
  // int status = spi_dbus_init(5000000, 0);
  // if (status < 0)
  // {
  //   std::cout << " spi device init fail" << std::endl;
  // }

  // status = spi_slave_dbus_init(0);
  // if (status < 0)
  // {
  //   std::cout << " spi device slave init fail" << std::endl;
  // }
  /*enable MCU CAN */
  spi_frame.resize(1 + sizeof(headerType));
  header = reinterpret_cast<headerType *>(spi_frame.data());
  header->type = MSG_CAN_SET_OPMODE; //
  header->length = 1;
  spi_frame.back() = 1;
  frame = spi_frame.data();
  frame_len = static_cast<int>(spi_frame.size());
  int ret = spi_dbus_send_frame(frame, frame_len);
  if (ret < 0)
  {
    std::cout << "send MCU CAN enble fail" << std::endl;
  }
  else
  {
    // do nothing
  }

  //////////////////////////////////////////////////////////////////////////批量发送can 构建完整的信息　OutputCAN　
  // typedef std::vector<struct can_frame> OutputCAN;
  // OutputCAN *outputcan = new OutputCAN();

  // _Vehicle_2D_Odometry _vehicle_2d_odometry;
  // _vehicle_2d_odometry.Position_X = 0B111101111011110111;//18
  // _vehicle_2d_odometry.Position_Y = 0B000010000100001000;//18
  // _vehicle_2d_odometry.Orentation = 0B0001000100010001;//16
  // _vehicle_2d_odometry.NULL_Bit = 0B11101110;//8
  // _vehicle_2d_odometry.Odometry_Counter = 0B1111; // 4 小端模式存储intel

  // can_frame can_frame_2D;
  // can_frame_2D.can_id = 458;
  // can_frame_2D.can_dlc = 8;

  // memcpy(can_frame_2D.data, &_vehicle_2d_odometry, sizeof(_vehicle_2d_odometry));
  // printf("%02x %02x %02x %02x %02x %02x %02x %02x \n \n", can_frame_2D.data[0], can_frame_2D.data[1], can_frame_2D.data[2], can_frame_2D.data[3], can_frame_2D.data[4], can_frame_2D.data[5], can_frame_2D.data[6], can_frame_2D.data[7]);

  // //can_frame_2D.data[CAN_MAX_DLEN] = _vehicle_2d_odometry;/////???????????????

  // outputcan->push_back(can_frame_2D); //
  ////////////////////////////////////////////////////////
  ///////////////////将OutputCAN　的信息　赋值到　　spi_frame　中　
  /*////////////////////////////////////////////////////////
OutputCAN　数据结构　can_frame1 can_frame2 can_frame3........
spi_frame 的数据结构　
headerType　　can_frame1 can_frame2 can_frame3........
type length  
*/
  ////////////////////////////////////////////////////////
  if (outputcan == NULL)
  {
    return 0;
  }

  int vec_size = outputcan->size();                    //发送的报文数量　
  std::cout << "vec_size = " << vec_size << std::endl; //1
  headerType *header2 = NULL;
  char *frame2 = NULL;
  int frame_len2 = 0;
  int spi_frame_size2 = sizeof(headerType) + vec_size * sizeof(struct can_frame);
  std::vector<char> spi_frame2(spi_frame_size2, 0);

  if (vec_size > 0)
  {
    header2 = reinterpret_cast<headerType *>(spi_frame2.data());
    header2->type = MSG_CAN_DATA_TRANSMIT; ////////////
    header2->length = sizeof(struct can_frame) * vec_size;

    struct can_frame *can_txBuf =
        reinterpret_cast<struct can_frame *>(spi_frame2.data() + sizeof(headerType));

    struct can_frame *tmp_frame;
    int i = 0;
    for (i = 0; i < vec_size; i++)
    {
      tmp_frame = can_txBuf + i;
      memset(tmp_frame, 0, sizeof(struct can_frame));
      tmp_frame->can_id = (*outputcan)[i].can_id;
      tmp_frame->can_dlc = (*outputcan)[i].can_dlc;
      memcpy(tmp_frame->data, (*outputcan)[i].data, (*outputcan)[i].can_dlc);
    }

    frame = spi_frame2.data();
    frame_len = static_cast<int>(spi_frame2.size());
    std::cout << "can_frame = " << sizeof(struct can_frame) << "   frame_len = " << frame_len << std::endl; //1

   
      int ret = spi_slave_dbus_send_frame(frame, frame_len);
      if (ret < 0)
      {
        std::cout << "send can fail " << std::endl;
      }     
    
  }
  // delete outputcan;
  return 0;
}
