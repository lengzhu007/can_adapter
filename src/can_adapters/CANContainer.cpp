/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: CANContainer.cpp
*   Author  : lubing.han
*   Date    : 2017-02-07
*   Describe:
*
********************************************************/

#include "CANContainer.h"
#include "spi_dbus_hal.h"
#include "service_handler_module.h"
#include "recv_send.h"
#include <iostream>
#include <string>
using namespace std;

namespace can_adapter
{

bool stop_can()
{
	spi_dbus_exit();
	spi_slave_dbus_exit();
	return true;
}

ServiceHandler *service_handler = NULL;


bool init_can()
{
	bool flag;
	service_handler = new ServiceHandler();
	flag = service_handler->Init();
	// service_handler->Init();//返回的true 能否用于　　init的返回值
	// if(service_handler->Init()) return true;
	// return false;
	service_handler->CANFilterVersionRequest();

	///////////////////////////////////////////////////////////////////////////////
	//给canFilterCfg赋值　　并调用SetCANFilterConfig()函数
	canFltCfgType canFilterCfg;
	canFilterCfg.ctrlPare.throut = 1 << CAN_MASK_CAN0;
	canFilterCfg.ctrlPare.rvseThrout =
		0xFF & (~canFilterCfg.ctrlPare.throut);

	static char CAN_FILTER_THROUGHOUT[8] = {'t', 'h', 'r', 'g', 'h', 'o', 'u', 't'};
	memcpy(canFilterCfg.canFltVer, CAN_FILTER_THROUGHOUT, 8);
	service_handler->SetCANFilterConfig(&canFilterCfg);
	////////////////////////////////////////////////////////////////////////////////

	service_handler->ServiceHandlerSetOpMode(CAN_OP_ON);
	bool spi_init_inform_ = service_handler->get_spi_init_inform_();
	int service_spi_state_ = service_handler->get_service_spi_state_();
	int service_slave_spi_state_ = service_handler->get_service_slave_spi_state_();
	int service_com_state_ = service_handler->get_service_com_state_();
	int service_com_init_step_ = service_handler->get_service_com_init_step_();
	int ts_start = GetTimeStamp();

	if (service_spi_state_ == service_handler->SERVICE_SPI_STATE_INIT)
	{

		if (spi_init_inform_ == true)
		{
			ServiceMsg *svc_msg = new ServiceMsg();
			svc_msg->type_ = SPI_INIT;
			svc_msg->length_ = 0;
			spi_init_inform_ = false;
			delete svc_msg;
		}
		else
		{
		}

		service_handler->ServiceHandlerTimeStampSyncMon(); //跟mcu通讯
		service_handler->ServiceHandlerKeepAliveMon();
		service_handler->ServiceHandlerMcuVerMon();
		
	}
	
	
	delete service_handler;
	if(flag) return true;
	return false;
}
void send_messages(const vector<CanBase> &send_buff, int can_number)
{

/////////////////将sendbuff 转换为                   outputcan/////////////////
/////////CanBase1 CanBase2  CanBase3                canframe1 canframe2 canframe3
#define MAX_LENGTH 4 * 1024 //???????????????
	if (send_buff.size() == 0)
		return;
	int n = send_buff.size() < MAX_LENGTH ? send_buff.size() : MAX_LENGTH;
	OutputCAN *outputcan = new OutputCAN();
	for (int i = 0; i < n; ++i)
	{
		(*outputcan)[i].can_id = send_buff[i].get_id();
		(*outputcan)[i].can_dlc = 8;

		for (int j = 0; j < 8; ++j)
			(*outputcan)[i].data[j] = 0; //清零

		const BYTE *send_bytes = send_buff[i].get_bytes();
		for (int j = 0; j < 8; ++j)
			(*outputcan)[i].data[j] = send_bytes[j];
	}
	send_can_spi(outputcan); //调用spi 封装好的程序　将can信息发出去　
	delete outputcan;
}

void recv_messages(vector<CanBase> &recv_buff, int can_number)
{
	recv_buff.clear();

	SpiCanFrameType spi_can_frame;

	//  ServiceHandler service_handler;
	// recv_can_spi(spi_can_frame, service_handler); //调用函数 调用spi_can_frame return spi_can_frame
	recv_can_spi(spi_can_frame
	            // , *service_handler
	              ); //调用函数 调用spi_can_frame return spi_can_frame
	// int recv_can_spi(SpiCanFrameType & spi_can_frame,ServiceHandler &service_handler)
	//recv_can_spi --> recv_message --> CANTrans::recv_callback -->CANTrans::start   CANTrans::start -->　init_can() 有　ServiceHandler
	int n = spi_can_frame.can_frame_.size();
	for (int i = 0; i < n; ++i)
	{
		CanBase recvBase(spi_can_frame.can_frame_[i].can_id);
		recvBase.set_time_stamp((double)spi_can_frame.ts_);
		recvBase.set_bytes(spi_can_frame.can_frame_[i].data);
		recv_buff.push_back(recvBase);
	}
}

} // namespace can_adapter
