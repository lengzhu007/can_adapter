#include "spi_dbus_hal.h"
#include "service_handler_module.h"
#include <memory>
#include "stdlib.h"
#include "string.h"
#include <iostream>
#include <stdio.h>

#define ADAS_FPGA
#define ADAS_ALTERA
int recv_can_spi(SpiCanFrameType & spi_can_frame
                //  , ServiceHandler &service_handler
                 )//可以传进来么??????
{
#if defined(ADAS_FPGA) && defined(ADAS_ALTERA)

  ServiceHandler *service_handler = new ServiceHandler();
    service_handler->RegisterServiceRecieve(MSG_CAN_DATA,
    &ServiceHandler::ServiceHandlerCANDataRecieve);
  service_handler->RegisterServiceRecieve(MSG_RES_S_READ_DATA,
    &ServiceHandler::ServiceHandlerResSReadData);
  // service_handler->Init();
  // service_handler->CANFilterVersionRequest();

  // ///////////////////////////////////////////////////////////////////////////////
  // //给canFilterCfg赋值　　并调用SetCANFilterConfig()函数  
  // canFltCfgType canFilterCfg;
  // canFilterCfg.ctrlPare.throut = 1 << CAN_MASK_CAN0;
  // canFilterCfg.ctrlPare.rvseThrout =
  //     0xFF & (~canFilterCfg.ctrlPare.throut); 

  // static char CAN_FILTER_THROUGHOUT[8] = {'t', 'h', 'r', 'g', 'h', 'o', 'u', 't'};
  // memcpy(canFilterCfg.canFltVer, CAN_FILTER_THROUGHOUT, 8);
  // service_handler->SetCANFilterConfig(&canFilterCfg);  
  // ////////////////////////////////////////////////////////////////////////////////

  // service_handler->ServiceHandlerSetOpMode(CAN_OP_ON);
  // bool spi_init_inform_ = service_handler->get_spi_init_inform_();
  // int service_spi_state_ = service_handler->get_service_spi_state_();
  // int service_slave_spi_state_ = service_handler->get_service_slave_spi_state_();
  // int service_com_state_ = service_handler->get_service_com_state_();
  // int service_com_init_step_ = service_handler->get_service_com_init_step_();
  // int ts_start = GetTimeStamp();



  // if (service_spi_state_ == service_handler->SERVICE_SPI_STATE_INIT)
  // {
   
  //   if (spi_init_inform_ == true)
  //   {     
  //     ServiceMsg *svc_msg = new ServiceMsg();      
  //     svc_msg->type_ = SPI_INIT;
  //     svc_msg->length_ = 0;
  //     spi_init_inform_ = false;
  //   }
  //   else
  //   {
  //   }
    
  //   service_handler->ServiceHandlerTimeStampSyncMon(); //跟mcu通讯
  //   service_handler->ServiceHandlerKeepAliveMon();
  //   service_handler->ServiceHandlerMcuVerMon();

    int recv_size = 0;
    ServiceHandler::msgGeneralType msg_general_rx;
  
   
      int output_slot = service_handler->OUTPUT_SLOT_NONE;
      recv_size = spi_dbus_recv_frame(
          reinterpret_cast<char *>(&msg_general_rx), MSG_SVCHDL_MAX_SIZE); //&msg_general_rx  转换为指针
     
      // if (recv_size != 0)
      //   std::cout << "070 recv_size = " << recv_size << std::endl;
      if (recv_size > 0)
      {
        ServiceMsg *svc_msg = new ServiceMsg();      
        output_slot = service_handler->ServiceHandlerRecvParse(&msg_general_rx, svc_msg);
       if(output_slot != service_handler->OUTPUT_SLOT_NONE)
       {
        
        //////////////////////将数据按照can 存到 spi_can_frame 结构体
        if (svc_msg->type_ == CAN_DATA)
        {
          int frame_num = svc_msg->length_ / sizeof(struct can_frame);
          struct can_frame *mcu_frame =
              reinterpret_cast<struct can_frame *>(&svc_msg->data_[0]);
//  printf("can_data=%2x %2x %2x %2x %2x %2x %2x %2x can_id=%02x \n \n", mcu_frame->data[0], mcu_frame->data[1], mcu_frame->data[2], mcu_frame->data[3], mcu_frame->data[4], mcu_frame->data[5], mcu_frame->data[6], mcu_frame->data[7], mcu_frame->can_id);
          // SpiCanFrameType spi_can_frame;
          spi_can_frame.ts_ = svc_msg->ts_;


          for (int i = 0; i < frame_num; i++)
          {
            spi_can_frame.can_frame_.push_back(mcu_frame[i]);
          }

          // return spi_can_frame；///////////////////////?????????????????
          /////////////////////////////////////遍历元素并打印///////////////////////////
          std::vector<struct can_frame>::iterator iter;
          for (iter = spi_can_frame.can_frame_.begin();
               iter != spi_can_frame.can_frame_.end(); iter++)
          {
            printf("can_data=%02x %02x %02x %02x %02x %02x %02x %02x can_id=%02x \n \n", iter->data[0], iter->data[1], iter->data[2], iter->data[3], iter->data[4], iter->data[5], iter->data[6], iter->data[7], iter->can_id); 
          }
          ////////////////////////////////////////////////////////////////////////////////
         
        }
      }
           delete svc_msg;
      }
   
   
  
#endif
delete service_handler;
  return 0;
}




